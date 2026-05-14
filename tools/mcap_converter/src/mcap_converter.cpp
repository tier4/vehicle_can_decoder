// Copyright 2026 TIER IV, Inc.
//
// Offline bag conversion tool. Reads can_msgs/Frame messages from an input
// rosbag2 bag (MCAP or sqlite3, directory or single-file) and writes
// vehicle_can_decoder/SignalGroup messages to an output bag by running the
// same decode → alias → transform pipeline as the live ROS 2 node.
//
// Requires ROS 2 Humble or later. Build:
//   source /opt/ros/humble/setup.bash
//   colcon build --packages-select vehicle_can_decoder
//   source install/setup.bash
//   cd tools/mcap_converter
//   cmake -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j$(nproc)
//
// Usage:
//   ./build/mcap_converter \
//     --input  <bag_or_dir>   \
//     --output <out_dir>      \
//     --dbc    <vehicle.dbc>  \
//     --config <vehicle.yaml> [--config <schema.yaml> ...]

#include "vehicle_can_decoder/dbc_decoder.hpp"
#include "vehicle_can_decoder/signal_pipeline.hpp"
#include "vehicle_can_decoder/signal_router.hpp"
#include "vehicle_can_decoder/signal_transformer.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <vehicle_can_decoder/msg/signal.hpp>
#include <vehicle_can_decoder/msg/signal_group.hpp>

#include <can_msgs/msg/frame.hpp>

#include <rcutils/types/uint8_array.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ── Temporary ament prefix for MCAP schema lookup ────────────────────────────
//
// rosbag2_storage_mcap calls ament_index_cpp::get_package_share_directory() to
// find the .msg file when writing the MCAP schema.  On machines where
// vehicle_can_decoder is not installed as a ROS package, this lookup fails and
// the schema is written as empty — Foxglove then rejects the file with
// "schema encoding '' is not supported".
//
// We create a minimal ament resource index in a temp directory, write the .msg
// file contents as string literals, and prepend the directory to
// AMENT_PREFIX_PATH before opening the writer.  ament_index_cpp reads the env
// var on every call (not cached), so setting it here is sufficient.
namespace ament_prefix
{

static const char kSignalMsg[] =
  "uint16 name_id\n"
  "float32 value\n";

static const char kSignalGroupMsg[] =
  "std_msgs/Header header\n"
  "string domain\n"
  "vehicle_can_decoder/Signal[] signals\n";

inline std::filesystem::path create(const std::filesystem::path & prefix)
{
  namespace fs = std::filesystem;

  // ament_index_cpp::get_package_share_directory() checks for a file at:
  //   <prefix>/share/ament_index/resource_index/packages/<pkg>
  // and returns  <prefix>/share/<pkg>.
  const fs::path pkg_index = prefix / "share/ament_index/resource_index/packages";
  fs::create_directories(pkg_index);
  std::ofstream{pkg_index / "vehicle_can_decoder"};  // empty marker file

  const fs::path msg_dir = prefix / "share/vehicle_can_decoder/msg";
  fs::create_directories(msg_dir);

  {
    std::ofstream f(msg_dir / "Signal.msg");
    f << kSignalMsg;
  }
  {
    std::ofstream f(msg_dir / "SignalGroup.msg");
    f << kSignalGroupMsg;
  }

  return prefix;
}

}  // namespace ament_prefix

// ── Manual CDR serializer for SignalGroup ─────────────────────────────────────
//
// rclcpp::Serialization<SignalGroup> calls rmw_serialize(), which dlopen()s
// libvehicle_can_decoder__rosidl_typesupport_introspection_cpp.so at runtime.
// That library is absent on machines where vehicle_can_decoder is not installed
// as a ROS package (e.g. the data recording system).  Encoding CDR directly
// removes this runtime dependency while producing identical bytes on the wire.
namespace cdr
{

struct Buffer
{
  std::vector<uint8_t> data;

  void align(size_t a)
  {
    const size_t r = data.size() % a;
    if (r) data.insert(data.end(), a - r, 0u);
  }

  void write_uint16(uint16_t v)
  {
    align(2);
    data.push_back(static_cast<uint8_t>(v & 0xFF));
    data.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  }

  void write_int32(int32_t v) { write_uint32(static_cast<uint32_t>(v)); }

  void write_uint32(uint32_t v)
  {
    align(4);
    data.push_back(static_cast<uint8_t>(v & 0xFF));
    data.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    data.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
  }

  void write_float32(float v)
  {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    write_uint32(bits);
  }

  void write_string(const std::string & s)
  {
    write_uint32(static_cast<uint32_t>(s.size() + 1));  // length includes null terminator
    data.insert(data.end(), s.begin(), s.end());
    data.push_back(0);  // null terminator
  }
};

inline void serialize_signal_group(const vehicle_can_decoder::msg::SignalGroup & sg, Buffer & buf)
{
  // OMG CDR §9.3.2 encapsulation header: 0x0001 = CDR_LE (little-endian), 2 padding bytes
  static constexpr std::array<uint8_t, 4> kCdrLeHeader{0x00, 0x01, 0x00, 0x00};
  buf.data.insert(buf.data.end(), kCdrLeHeader.begin(), kCdrLeHeader.end());

  // std_msgs/Header
  buf.write_int32(sg.header.stamp.sec);
  buf.write_uint32(sg.header.stamp.nanosec);
  buf.write_string(sg.header.frame_id);

  buf.write_string(sg.domain);

  buf.write_uint32(static_cast<uint32_t>(sg.signals.size()));
  for (const auto & s : sg.signals) {
    buf.write_uint16(s.name_id);
    buf.write_float32(s.value);
  }
}

}  // namespace cdr

// ── YAML helpers ──────────────────────────────────────────────────────────────
//
// The vehicle config YAML uses ROS 2 parameter conventions:
//
//   vehicle_can_node:
//     ros__parameters:
//       can_topic: /vehicle/from_can_bus
//       domains.chassis.topic: /vehicle/chassis   # dotted key, not nested
//       domains.chassis.can_ids: [256, 257]
//
// Keys that contain dots are stored as-is (YAML does not interpret dots as
// nesting). flatten_yaml only descends into true YAML map nodes.

namespace
{

using ParamMap = std::unordered_map<std::string, YAML::Node>;

void flatten_yaml(const YAML::Node & node, const std::string & prefix, ParamMap & out)
{
  if (node.IsMap()) {
    for (const auto & kv : node) {
      const std::string key =
        prefix.empty() ? kv.first.as<std::string>() : prefix + "." + kv.first.as<std::string>();
      flatten_yaml(kv.second, key, out);
    }
  } else {
    out[prefix] = node;
  }
}

ParamMap load_yaml(const std::string & path)
{
  YAML::Node root = YAML::LoadFile(path);
  if (root["vehicle_can_node"] && root["vehicle_can_node"]["ros__parameters"]) {
    root = root["vehicle_can_node"]["ros__parameters"];
  }
  ParamMap params;
  flatten_yaml(root, "", params);
  return params;
}

std::string get_s(const ParamMap & p, const std::string & k, const std::string & def = "")
{
  const auto it = p.find(k);
  if (it == p.end()) return def;
  try {
    return it->second.as<std::string>();
  } catch (...) {
    return def;
  }
}

std::vector<std::string> get_sv(const ParamMap & p, const std::string & k)
{
  const auto it = p.find(k);
  if (it == p.end() || !it->second.IsSequence()) return {};
  std::vector<std::string> v;
  for (const auto & e : it->second) {
    try {
      v.push_back(e.as<std::string>());
    } catch (...) {
    }
  }
  return v;
}

std::vector<int64_t> get_iv(const ParamMap & p, const std::string & k)
{
  const auto it = p.find(k);
  if (it == p.end() || !it->second.IsSequence()) return {};
  std::vector<int64_t> v;
  for (const auto & e : it->second) {
    try {
      v.push_back(e.as<int64_t>());
    } catch (...) {
    }
  }
  return v;
}

// ── Config ────────────────────────────────────────────────────────────────────

using vehicle_can_decoder::DomainConfig;
using vehicle_can_decoder::TransformConfig;

struct Config
{
  std::string can_topic{"/vehicle/from_can_bus"};

  // CAN-ID-based domain routing (from domain_names / domains.* keys)
  std::vector<DomainConfig> domains;

  // Signal name aliases (DBC name → canonical name)
  std::unordered_map<std::string, std::string> aliases;

  // exprtk transform per signal
  std::unordered_map<std::string, TransformConfig> transforms;

  // Schema-based domain routing (from schema_domain_names / schema.*.signals).
  // Takes precedence over CAN-ID routing when non-empty.
  std::unordered_map<std::string, std::string> signal_to_domain;
  std::unordered_map<std::string, std::string> schema_domain_topics;

  // Signal ID table (from signal_id_names in vehicle_schema.yaml).
  // Used to populate Signal.name_id; 0 when not configured.
  std::unordered_map<std::string, uint16_t> signal_name_to_id;
};

Config build_config(const ParamMap & p)
{
  Config cfg;
  cfg.can_topic = get_s(p, "can_topic", "/vehicle/from_can_bus");

  for (const auto & name : get_sv(p, "domain_names")) {
    DomainConfig dc;
    dc.name = name;
    dc.topic = get_s(p, "domains." + name + ".topic", "/vehicle/" + name);
    for (const int64_t id : get_iv(p, "domains." + name + ".can_ids")) {
      dc.can_ids.insert(static_cast<uint32_t>(id));
    }
    cfg.domains.push_back(std::move(dc));
  }

  for (const auto & src : get_sv(p, "alias_names")) {
    cfg.aliases[src] = get_s(p, "aliases." + src, src);
  }

  for (const auto & sig : get_sv(p, "transform_names")) {
    TransformConfig tc;
    tc.expression = get_s(p, "transforms." + sig + ".expression");
    tc.unit = get_s(p, "transforms." + sig + ".unit");
    cfg.transforms[sig] = std::move(tc);
  }

  for (const auto & name : get_sv(p, "schema_domain_names")) {
    cfg.schema_domain_topics[name] = get_s(p, "schema." + name + ".topic", "/vehicle/" + name);
    for (const auto & sig : get_sv(p, "schema." + name + ".signals")) {
      cfg.signal_to_domain[sig] = name;
    }
  }

  const auto sig_names = get_sv(p, "signal_id_names");
  for (uint16_t i = 0; i < static_cast<uint16_t>(sig_names.size()); ++i) {
    cfg.signal_name_to_id[sig_names[i]] = static_cast<uint16_t>(i + 1);
  }

  return cfg;
}

// ── CLI ───────────────────────────────────────────────────────────────────────

struct Args
{
  std::string input;
  std::string output;
  std::string dbc;
  std::vector<std::string> configs;
};

void print_usage(const char * prog)
{
  std::cerr << "Usage: " << prog << " \\\n"
            << "  --input   <bag_or_dir>   Input bag (MCAP or sqlite3, dir or file)\n"
            << "  --output  <out_dir>      Output directory\n"
            << "  --dbc     <vehicle.dbc>  DBC file\n"
            << "  --config  <config.yaml>  Vehicle config YAML (repeatable)\n";
}

Args parse_args(int argc, char * argv[])
{
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string f = argv[i];
    if (i + 1 >= argc) continue;
    if (f == "--input")
      a.input = argv[++i];
    else if (f == "--output")
      a.output = argv[++i];
    else if (f == "--dbc")
      a.dbc = argv[++i];
    else if (f == "--config")
      a.configs.push_back(argv[++i]);
  }
  return a;
}

}  // namespace

// ── main ──────────────────────────────────────────────────────────────────────

using vehicle_can_decoder::DbcDecoder;
using vehicle_can_decoder::SignalRouter;
using vehicle_can_decoder::SignalTransformer;

int main(int argc, char * argv[])
{
  const Args args = parse_args(argc, argv);
  if (args.input.empty() || args.output.empty() || args.dbc.empty() || args.configs.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  // ── Load and merge YAML configs ───────────────────────────────────────────
  ParamMap params;
  for (const auto & path : args.configs) {
    try {
      for (const auto & kv : load_yaml(path)) {
        params[kv.first] = kv.second;
      }
    } catch (const std::exception & e) {
      std::cerr << "Failed to load config '" << path << "': " << e.what() << "\n";
      return 1;
    }
  }
  const Config cfg = build_config(params);

  // ── Setup signal processing pipeline ──────────────────────────────────────
  DbcDecoder decoder;
  if (!decoder.load(args.dbc)) {
    std::cerr << "Failed to load DBC: " << args.dbc << "\n";
    return 1;
  }
  std::cout << "DBC: " << args.dbc << " (" << decoder.known_ids().size() << " messages)\n";

  SignalRouter router;
  for (const auto & w : router.configure(cfg.domains, cfg.aliases)) {
    std::cerr << "Warning: " << w << "\n";
  }

  SignalTransformer transformer;
  transformer.configure(cfg.transforms);

  // ── Build domain → topic map ───────────────────────────────────────────────
  std::unordered_map<std::string, std::string> domain_to_topic;
  for (const auto & dc : cfg.domains) {
    domain_to_topic[dc.name] = dc.topic;
  }
  for (const auto & [name, topic] : cfg.schema_domain_topics) {
    domain_to_topic[name] = topic;
  }

  // ── Check output does not already exist ───────────────────────────────────
  if (std::filesystem::exists(args.output)) {
    std::cerr << "Output already exists: " << args.output
              << ". Remove it first or choose a different output path.\n";
    return 1;
  }

  // ── Open input bag ─────────────────────────────────────────────────────────
  rosbag2_storage::StorageOptions input_opts;
  input_opts.uri = args.input;
  auto reader = std::make_unique<rosbag2_cpp::Reader>();
  try {
    reader->open(input_opts);
  } catch (const std::exception & e) {
    std::cerr << "Failed to open input bag: " << e.what() << "\n";
    return 1;
  }

  // Match output storage type to input (mcap or sqlite3).
  // rosbag2_cpp::Writer always creates a directory regardless of URI.
  const std::string storage_id = reader->get_metadata().storage_identifier;

  // ── Register vehicle_can_decoder .msg files in a temp ament prefix ───────
  // rosbag2_storage_mcap calls ament_index_cpp::get_package_share_directory()
  // when create_topic() is called to look up the message schema.  If the
  // package is absent the schema is left empty and Foxglove rejects the file.
  // Prepend a temp prefix with the msg files before opening the writer so the
  // lookup succeeds.  ament_index_cpp reads AMENT_PREFIX_PATH on each call.
  const std::filesystem::path tmp_prefix =
    std::filesystem::temp_directory_path() /
    ("vcd_mcap_schema_" + std::to_string(static_cast<long>(::getpid())));
  ament_prefix::create(tmp_prefix);
  {
    const char * existing = std::getenv("AMENT_PREFIX_PATH");
    const std::string updated = tmp_prefix.string() + (existing ? std::string(":") + existing : "");
    ::setenv("AMENT_PREFIX_PATH", updated.c_str(), 1);
  }

  // ── Open output bag ────────────────────────────────────────────────────────
  rosbag2_storage::StorageOptions output_opts;
  output_opts.uri = args.output;
  output_opts.storage_id = storage_id;
  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer->open(output_opts);
  } catch (const std::exception & e) {
    std::cerr << "Failed to open output bag: " << e.what() << "\n";
    std::filesystem::remove_all(tmp_prefix);
    return 1;
  }

  // ── Pre-register all passthrough topics ───────────────────────────────────
  for (const auto & topic_meta : reader->get_all_topics_and_types()) {
    if (topic_meta.name == cfg.can_topic) continue;
    writer->create_topic(topic_meta);
  }

  // ── Register one output topic per domain ──────────────────────────────────
  for (const auto & [domain, topic] : domain_to_topic) {
    rosbag2_storage::TopicMetadata sig_meta;
    sig_meta.name = topic;
    sig_meta.type = "vehicle_can_decoder/msg/SignalGroup";
    sig_meta.serialization_format = "cdr";
    writer->create_topic(sig_meta);
  }

  // ── Declare serializers once outside the loop ─────────────────────────────
  rclcpp::Serialization<can_msgs::msg::Frame> frame_deserializer;

  // groups is declared outside the loop and cleared per CAN frame to avoid
  // per-frame heap allocation.
  std::unordered_map<std::string, vehicle_can_decoder::msg::SignalGroup> groups;

  // ── Conversion loop ────────────────────────────────────────────────────────
  uint64_t frames_in = 0, frames_decoded = 0, frames_unknown = 0;
  uint64_t groups_out = 0, passthrough = 0;

  std::cout << "Converting: " << args.input << " → " << args.output << "\n";

  try {
    while (reader->has_next()) {
      auto bag_msg = reader->read_next();

      // ── Non-CAN topics: pass through verbatim ───────────────────────────
      if (bag_msg->topic_name != cfg.can_topic) {
        writer->write(bag_msg);
        ++passthrough;
        continue;
      }

      ++frames_in;

      // ── Deserialize can_msgs/Frame ────────────────────────────────────
      // In Humble, serialized_data is shared_ptr<rcutils_uint8_array_t>;
      // dereference to get the struct and let SerializedMessage deep-copy it.
      rclcpp::SerializedMessage ser_in(*bag_msg->serialized_data);
      can_msgs::msg::Frame ros_frame;
      frame_deserializer.deserialize_message(&ser_in, &ros_frame);

      // ── Decode via DBC ────────────────────────────────────────────────
      std::array<uint8_t, 8> data_arr{};
      const uint8_t valid_dlc = (ros_frame.dlc <= 8u) ? ros_frame.dlc : 8u;
      std::copy_n(ros_frame.data.begin(), valid_dlc, data_arr.begin());
      const auto decoded = decoder.decode(ros_frame.id, data_arr, valid_dlc);
      if (!decoded) {
        ++frames_unknown;
        continue;
      }
      ++frames_decoded;

      // ── Build per-domain SignalGroups ─────────────────────────────────
      groups.clear();

      for (const auto & raw_sig : *decoded) {
        const auto res = vehicle_can_decoder::resolve_signal(
          raw_sig.name, ros_frame.id, router, cfg.signal_to_domain);
        if (!res.is_assigned()) continue;
        const auto & name = res.name;
        const auto & domain = res.domain;

        const auto tr = transformer.transform(name, raw_sig.value);

        auto & sg = groups[domain];
        if (sg.domain.empty()) {
          sg.header.stamp.sec = ros_frame.header.stamp.sec;
          sg.header.stamp.nanosec = ros_frame.header.stamp.nanosec;
          sg.domain = domain;
        }

        vehicle_can_decoder::msg::Signal s;
        const auto id_it = cfg.signal_name_to_id.find(name);
        s.name_id = (id_it != cfg.signal_name_to_id.end()) ? id_it->second : 0;
        s.value = static_cast<float>(tr.value);
        sg.signals.push_back(s);
      }

      // ── Serialize and write one SignalGroup per domain ────────────────
      for (auto & [domain, sg] : groups) {
        const auto topic_it = domain_to_topic.find(domain);
        if (topic_it == domain_to_topic.end()) continue;

        // Encode CDR directly instead of calling rclcpp::Serialization /
        // rmw_serialize, which would dlopen the introspection type support at
        // runtime — a library absent on machines without vehicle_can_decoder
        // installed as a ROS package.
        cdr::Buffer cdr_buf;
        cdr::serialize_signal_group(sg, cdr_buf);

        auto out_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        out_msg->topic_name = topic_it->second;
        static constexpr int64_t kNsPerSec = 1'000'000'000LL;
        out_msg->time_stamp = static_cast<int64_t>(ros_frame.header.stamp.sec) * kNsPerSec +
                              ros_frame.header.stamp.nanosec;

        // In Humble, SerializedBagMessage::serialized_data is
        // shared_ptr<rcutils_uint8_array_t>. Allocate a new array on the
        // heap, init it, copy the serialized bytes, then assign a
        // shared_ptr with a custom deleter that calls rcutils_uint8_array_fini.
        auto * arr = new rcutils_uint8_array_t;
        *arr = rcutils_get_zero_initialized_uint8_array();
        auto allocator = rcutils_get_default_allocator();
        const auto ret = rcutils_uint8_array_init(arr, cdr_buf.data.size(), &allocator);
        if (ret != RCUTILS_RET_OK || arr->buffer == nullptr) {
          delete arr;
          std::cerr << "OOM: failed to allocate output message buffer\n";
          std::filesystem::remove_all(tmp_prefix);
          return 1;
        }
        std::memcpy(arr->buffer, cdr_buf.data.data(), cdr_buf.data.size());
        arr->buffer_length = cdr_buf.data.size();
        out_msg->serialized_data =
          std::shared_ptr<rcutils_uint8_array_t>(arr, [](rcutils_uint8_array_t * p) {
            rcutils_uint8_array_fini(p);
            delete p;
          });

        writer->write(out_msg);
        ++groups_out;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Error during conversion: " << e.what() << "\n";
    std::filesystem::remove_all(tmp_prefix);
    return 1;
  }

  // ── Summary ───────────────────────────────────────────────────────────────
  std::cout << "Done.\n"
            << "  CAN frames read:   " << frames_in << "\n"
            << "  Decoded:           " << frames_decoded << "\n"
            << "  Unknown CAN ID:    " << frames_unknown << "\n"
            << "  SignalGroups out:  " << groups_out << "\n"
            << "  Passthrough msgs:  " << passthrough << "\n";

  std::filesystem::remove_all(tmp_prefix);
  return 0;
}
