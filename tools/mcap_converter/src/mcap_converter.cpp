// Copyright 2026 TIER IV, Inc.
//
// Offline MCAP conversion tool — fully standalone, no ROS 2 installation required.
//
// Reads can_msgs/Frame messages from an input MCAP and writes
// vehicle_can_decoder/SignalGroup messages to an output MCAP by running the
// same decode → alias → transform pipeline as the live ROS 2 node.
//
// CDR serialization for both message types is implemented inline.
// MCAP file I/O uses the foxglove C++ library (fetched at build time).
//
// Build:
//   cd tools/mcap_converter
//   cmake -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j$(nproc)
//
// Usage:
//   ./build/mcap_converter \
//     --input  <in.mcap>      \
//     --output <out.mcap>     \
//     --dbc    <vehicle.dbc>  \
//     --config <vehicle.yaml> [--config <schema.yaml> ...]

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"
#include "vehicle_can_decoder/dbc_decoder.hpp"
#include "vehicle_can_decoder/signal_router.hpp"
#include "vehicle_can_decoder/signal_transformer.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// ── Minimal CDR reader / writer ───────────────────────────────────────────────
//
// Implements the subset of CDR (Common Data Representation, little-endian)
// needed to deserialize can_msgs/Frame and serialize
// vehicle_can_decoder/SignalGroup without linking against the ROS 2 message
// typesupport libraries.
//
// CDR wire format for ROS 2:
//   Bytes 0-3: encapsulation header  { 0x00, 0x01, 0x00, 0x00 } (CDR-LE)
//   Bytes 4-N: serialized fields, each aligned to its natural size
//   Strings:   uint32 length (includes NUL) then char bytes

namespace cdr
{

class Reader
{
public:
  Reader(const uint8_t * buf, size_t size)
  : buf_(buf), size_(size), pos_(4)  // skip the 4-byte encapsulation header
  {
  }

  uint8_t u8() { return buf_[pos_++]; }

  int32_t i32()
  {
    align(4);
    int32_t v;
    std::memcpy(&v, buf_ + pos_, 4);
    pos_ += 4;
    return v;
  }

  uint32_t u32()
  {
    align(4);
    uint32_t v;
    std::memcpy(&v, buf_ + pos_, 4);
    pos_ += 4;
    return v;
  }

  uint16_t u16()
  {
    align(2);
    uint16_t v;
    std::memcpy(&v, buf_ + pos_, 2);
    pos_ += 2;
    return v;
  }

  std::string str()
  {
    const uint32_t len = u32();
    if (len == 0) return {};
    std::string s(reinterpret_cast<const char *>(buf_ + pos_), len - 1);
    pos_ += len;
    return s;
  }

  bool ok() const { return pos_ <= size_; }

private:
  void align(size_t n) { pos_ = (pos_ + n - 1) & ~(n - 1); }

  const uint8_t * buf_;
  size_t size_;
  size_t pos_;
};

class Writer
{
public:
  Writer()
  {
    buf_ = {0x00, 0x01, 0x00, 0x00};  // CDR little-endian encapsulation header
  }

  void u8(uint8_t v) { buf_.push_back(v); }

  void i32(int32_t v)
  {
    align(4);
    const size_t off = buf_.size();
    buf_.resize(off + 4);
    std::memcpy(buf_.data() + off, &v, 4);
  }

  void u32(uint32_t v)
  {
    align(4);
    const size_t off = buf_.size();
    buf_.resize(off + 4);
    std::memcpy(buf_.data() + off, &v, 4);
  }

  void u16(uint16_t v)
  {
    align(2);
    const size_t off = buf_.size();
    buf_.resize(off + 2);
    std::memcpy(buf_.data() + off, &v, 2);
  }

  void f32(float v)
  {
    align(4);
    const size_t off = buf_.size();
    buf_.resize(off + 4);
    std::memcpy(buf_.data() + off, &v, 4);
  }

  void str(const std::string & s)
  {
    u32(static_cast<uint32_t>(s.size() + 1));
    for (char c : s) buf_.push_back(static_cast<uint8_t>(c));
    buf_.push_back(0);
  }

  const std::vector<uint8_t> & data() const { return buf_; }

private:
  void align(size_t n)
  {
    while (buf_.size() % n) buf_.push_back(0);
  }

  std::vector<uint8_t> buf_;
};

}  // namespace cdr

// ── can_msgs/Frame CDR deserialization ───────────────────────────────────────
//
// Message definition (can_msgs/msg/Frame):
//   std_msgs/Header header   (stamp: {int32 sec, uint32 nanosec}, string frame_id)
//   uint32           id
//   bool             is_rtr
//   bool             is_extended
//   bool             is_error
//   uint8            dlc
//   uint8[8]         data

struct CanFrameMsg
{
  int32_t sec{};
  uint32_t nanosec{};
  uint32_t id{};
  uint8_t dlc{};
  std::array<uint8_t, 8> data{};
};

static CanFrameMsg decode_can_frame(const uint8_t * buf, size_t size)
{
  cdr::Reader r(buf, size);
  CanFrameMsg f;
  f.sec = r.i32();
  f.nanosec = r.u32();
  r.str();  // frame_id (unused)
  f.id = r.u32();
  r.u8();  // is_rtr
  r.u8();  // is_extended
  r.u8();  // is_error
  f.dlc = r.u8();
  for (auto & b : f.data) b = r.u8();
  return f;
}

// ── vehicle_can_decoder/SignalGroup CDR serialization ────────────────────────
//
// Message definition (vehicle_can_decoder/msg/SignalGroup):
//   std_msgs/Header                  header
//   string                           domain
//   vehicle_can_decoder/Signal[]     signals
//
// vehicle_can_decoder/Signal:
//   uint16  name_id
//   float32 value

struct SignalEntry
{
  uint16_t name_id{};
  float value{};
};

static std::vector<uint8_t> encode_signal_group(
  int32_t sec, uint32_t nanosec, const std::string & domain,
  const std::vector<SignalEntry> & signals)
{
  cdr::Writer w;
  w.i32(sec);
  w.u32(nanosec);
  w.str("");  // frame_id
  w.str(domain);
  w.u32(static_cast<uint32_t>(signals.size()));
  for (const auto & s : signals) {
    w.u16(s.name_id);
    w.f32(s.value);
  }
  return w.data();
}

// ── Schema definition embedded as a string ───────────────────────────────────
//
// ros2msg encoding: the full message definition including all dependencies.

static const char kSignalGroupMsgDef[] =
  "std_msgs/Header header\n"
  "string domain\n"
  "vehicle_can_decoder/Signal[] signals\n"
  "\n"
  "================================================================================\n"
  "MSG: std_msgs/Header\n"
  "builtin_interfaces/Time stamp\n"
  "string frame_id\n"
  "\n"
  "================================================================================\n"
  "MSG: builtin_interfaces/Time\n"
  "int32 sec\n"
  "uint32 nanosec\n"
  "\n"
  "================================================================================\n"
  "MSG: vehicle_can_decoder/Signal\n"
  "uint16 name_id\n"
  "float32 value\n";

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
  // Used to populate SignalEntry.name_id; 0 when not configured.
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
            << "  --input   <in.mcap>      Input MCAP file\n"
            << "  --output  <out.mcap>     Output MCAP file\n"
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

  // ── Open input MCAP ────────────────────────────────────────────────────────
  std::ifstream infile(args.input, std::ios::binary);
  if (!infile.is_open()) {
    std::cerr << "Cannot open input: " << args.input << "\n";
    return 1;
  }
  mcap::FileStreamReader fileReader(infile);
  mcap::McapReader reader;
  {
    const auto status = reader.open(fileReader);
    if (!status.ok()) {
      std::cerr << "Failed to open MCAP: " << status.message << "\n";
      return 1;
    }
  }

  // ── Open output MCAP ───────────────────────────────────────────────────────
  mcap::FileWriter fileWriter;
  {
    const auto status = fileWriter.open(args.output);
    if (!status.ok()) {
      std::cerr << "Failed to open output: " << status.message << "\n";
      return 1;
    }
  }
  mcap::McapWriter writer;
  {
    mcap::McapWriterOptions opts("ros2");
    writer.open(fileWriter, opts);
  }

  // ── Register SignalGroup schema ────────────────────────────────────────────
  mcap::Schema signalGroupSchema;
  signalGroupSchema.name = "vehicle_can_decoder/msg/SignalGroup";
  signalGroupSchema.encoding = "ros2msg";
  {
    const auto * begin = reinterpret_cast<const std::byte *>(kSignalGroupMsgDef);
    const auto * end = begin + (sizeof(kSignalGroupMsgDef) - 1);  // exclude NUL
    signalGroupSchema.data = mcap::ByteArray(begin, end);
  }
  writer.addSchema(signalGroupSchema);

  // ── Register one output channel per domain topic ───────────────────────────
  std::unordered_map<std::string, mcap::ChannelId> domain_to_channel;
  for (const auto & [domain, topic] : domain_to_topic) {
    mcap::Channel channel;
    channel.topic = topic;
    channel.messageEncoding = "cdr";
    channel.schemaId = signalGroupSchema.id;
    writer.addChannel(channel);
    domain_to_channel[domain] = channel.id;
  }

  // ── Passthrough channel tracking (lazy creation on first encounter) ────────
  std::unordered_map<mcap::ChannelId, mcap::ChannelId> in_to_out_channel;
  std::unordered_map<mcap::SchemaId, mcap::SchemaId> in_to_out_schema;

  // ── Conversion loop ────────────────────────────────────────────────────────
  uint64_t frames_in = 0, frames_decoded = 0, frames_unknown = 0;
  uint64_t groups_out = 0, passthrough = 0;

  std::cout << "Converting: " << args.input << " → " << args.output << "\n";

  auto view = reader.readMessages();
  for (auto it = view.begin(); it != view.end(); ++it) {
    const mcap::MessageView & mv = *it;
    const mcap::Message & msg = mv.message;
    const std::string & topic = mv.channel->topic;

    // ── Non-CAN topics: pass through verbatim ─────────────────────────────
    if (topic != cfg.can_topic) {
      const auto ch_it = in_to_out_channel.find(msg.channelId);
      mcap::ChannelId out_channel_id;

      if (ch_it == in_to_out_channel.end()) {
        // Create output schema for this input schema (if not yet mapped).
        mcap::SchemaId out_schema_id = 0;
        if (mv.schema && !mv.schema->name.empty()) {
          const auto sc_it = in_to_out_schema.find(mv.channel->schemaId);
          if (sc_it == in_to_out_schema.end()) {
            mcap::Schema out_schema;
            out_schema.name = mv.schema->name;
            out_schema.encoding = mv.schema->encoding;
            out_schema.data = mv.schema->data;
            writer.addSchema(out_schema);
            in_to_out_schema[mv.channel->schemaId] = out_schema.id;
            out_schema_id = out_schema.id;
          } else {
            out_schema_id = sc_it->second;
          }
        }

        mcap::Channel out_channel;
        out_channel.topic = topic;
        out_channel.messageEncoding = mv.channel->messageEncoding;
        out_channel.schemaId = out_schema_id;
        out_channel.metadata = mv.channel->metadata;
        writer.addChannel(out_channel);
        in_to_out_channel[msg.channelId] = out_channel.id;
        out_channel_id = out_channel.id;
      } else {
        out_channel_id = ch_it->second;
      }

      mcap::Message out_msg = msg;
      out_msg.channelId = out_channel_id;
      (void)writer.write(out_msg);
      ++passthrough;
      continue;
    }

    ++frames_in;

    // ── Deserialize can_msgs/Frame from raw CDR bytes ──────────────────────
    const CanFrameMsg can_frame = decode_can_frame(
      reinterpret_cast<const uint8_t *>(msg.data), static_cast<size_t>(msg.dataSize));

    // ── Decode via DBC ─────────────────────────────────────────────────────
    const auto decoded = decoder.decode(can_frame.id, can_frame.data, can_frame.dlc);
    if (!decoded) {
      ++frames_unknown;
      continue;
    }
    ++frames_decoded;

    // ── Build per-domain SignalGroups ──────────────────────────────────────
    std::unordered_map<std::string, std::vector<SignalEntry>> groups;

    for (const auto & raw_sig : *decoded) {
      // Compound key "CAN{id}_{signal}" tried first to disambiguate signals
      // that share the same name across different CAN messages.
      const std::string compound = "CAN" + std::to_string(can_frame.id) + "_" + raw_sig.name;
      const std::string & compound_alias = router.apply_alias(compound);
      const std::string & name =
        (compound_alias != compound) ? compound_alias : router.apply_alias(raw_sig.name);

      // Schema-based routing takes precedence over CAN-ID-based routing.
      std::string domain;
      if (!cfg.signal_to_domain.empty()) {
        const auto domain_it = cfg.signal_to_domain.find(name);
        domain = (domain_it != cfg.signal_to_domain.end()) ? domain_it->second
                                                           : SignalRouter::kUnassignedDomain;
      } else {
        domain = router.domain_for_id(can_frame.id);
      }
      if (domain == SignalRouter::kUnassignedDomain) continue;

      const auto tr = transformer.transform(name, raw_sig.value);

      SignalEntry sig;
      {
        const auto id_it = cfg.signal_name_to_id.find(name);
        sig.name_id = (id_it != cfg.signal_name_to_id.end()) ? id_it->second : 0;
      }
      sig.value = static_cast<float>(tr.value);
      groups[domain].push_back(sig);
    }

    // ── Serialize and write one SignalGroup per domain ────────────────────
    for (const auto & [domain, signals] : groups) {
      const auto ch_it = domain_to_channel.find(domain);
      if (ch_it == domain_to_channel.end()) continue;

      const std::vector<uint8_t> cdr_data =
        encode_signal_group(can_frame.sec, can_frame.nanosec, domain, signals);

      mcap::Message out_msg;
      out_msg.channelId = ch_it->second;
      out_msg.logTime = msg.logTime;
      out_msg.publishTime = msg.publishTime;
      out_msg.sequence = 0;
      out_msg.data = reinterpret_cast<const std::byte *>(cdr_data.data());
      out_msg.dataSize = cdr_data.size();
      (void)writer.write(out_msg);
      ++groups_out;
    }
  }

  reader.close();
  writer.close();

  // ── Summary ───────────────────────────────────────────────────────────────
  std::cout << "Done.\n"
            << "  CAN frames read:   " << frames_in << "\n"
            << "  Decoded:           " << frames_decoded << "\n"
            << "  Unknown CAN ID:    " << frames_unknown << "\n"
            << "  SignalGroups out:  " << groups_out << "\n"
            << "  Passthrough msgs:  " << passthrough << "\n";

  return 0;
}
