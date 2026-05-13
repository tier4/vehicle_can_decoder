# Offline Bag Conversion (Building a Converter Tool)

This document describes how to build a standalone tool that converts a ROS 2 bag containing
raw CAN frames (`can_msgs/Frame`) into a new bag containing the abstracted vehicle signal
topics (`vehicle_can_decoder/msg/SignalGroup`).

The tool lives at `tools/mcap_converter/` and **requires ROS 2 Humble or later** — it uses
`rosbag2_cpp` for reading and writing bags and `rclcpp` serialization for CDR encoding.

## Supported Formats

`rosbag2_cpp::Writer` always creates a directory bag. The storage format (mcap or sqlite3)
is matched to the input bag.

| Format                     | Input | Output                      |
| -------------------------- | ----- | --------------------------- |
| MCAP directory             | ✓     | ✓                           |
| MCAP single-file (.mcap)   | ✓     | directory (mcap storage)    |
| sqlite3 directory          | ✓     | ✓                           |
| sqlite3 single-file (.db3) | ✓     | directory (sqlite3 storage) |

## Core Library Reusability

The conversion components are implemented as a **ROS-independent C++ library** and can be
linked from any standalone executable.

| Component           | ROS dependency | Role in conversion pipeline             |
| ------------------- | -------------- | --------------------------------------- |
| `DbcDecoder`        | None           | CAN frame → raw signal name-value pairs |
| `SignalTransformer` | None           | Raw value → transformed value + unit    |
| `SignalRouter`      | None           | Signal name aliasing and domain lookup  |
| `VehicleCanNode`    | rclcpp         | **Not reusable** — ROS node only        |

## Conversion Pipeline

```text
can_msgs/Frame (from rosbag2_cpp::Reader)
    │  rclcpp::Serialization<can_msgs::msg::Frame>::deserialize_message()
    │  [zero-copy: wraps bag_msg->serialized_data buffer directly]
    ▼
can_msgs::msg::Frame { header.stamp, id, dlc, data[8] }
    │  DbcDecoder::decode()
    ▼
vector<RawSignal> { name, value, can_id }
    │  SignalRouter::apply_alias()        — compound key "CAN{id}_{name}" tried first
    │  schema routing (signal→domain)     — takes precedence over CAN-ID routing
    │  or SignalRouter::domain_for_id()   — fallback CAN-ID-based routing
    │  SignalTransformer::transform()
    ▼
per-domain vehicle_can_decoder::msg::SignalGroup
    │  rclcpp::Serialization<SignalGroup>::serialize_message()
    │  rcutils_uint8_array_init() deep-copy into owned buffer
    ▼
rosbag2_cpp::Writer::write()

Non-CAN topics → writer->write(bag_msg) verbatim (shared_ptr passthrough)
```

## Message Definitions

### Input — `can_msgs/msg/Frame`

```text
std_msgs/Header header   # stamp: {int32 sec, uint32 nanosec}, string frame_id
uint32           id
bool             is_rtr
bool             is_extended
bool             is_error
uint8            dlc
uint8[8]         data
```

### Output — `vehicle_can_decoder/msg/SignalGroup`

```text
std_msgs/Header                  header
string                           domain
vehicle_can_decoder/Signal[]     signals
```

`vehicle_can_decoder/msg/Signal`:

```text
uint16  name_id   # index into signal_id_names table (0 = unknown)
float32 value
```

Signal names are **not embedded in each message** at runtime; they are looked up via the
`signal_id_names` array in the YAML config.

## Build

### Prerequisites

```bash
sudo apt install \
  ros-humble-rosbag2-cpp \
  ros-humble-rosbag2-storage-mcap \
  ros-humble-rosbag2-storage-sqlite3 \
  ros-humble-rclcpp \
  ros-humble-can-msgs
```

### Build order

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_can_decoder
source install/setup.bash

cd tools/mcap_converter
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

`vehicle_can_decoder` must be built and sourced before configuring `tools/mcap_converter`
because the CMakeLists.txt calls `find_package(vehicle_can_decoder REQUIRED)` to resolve
message headers and typesupport.

## Usage

```bash
./build/mcap_converter \
  --input   <bag_or_dir>   \
  --output  <out_dir>      \
  --dbc     <vehicle.dbc>  \
  --config  <vehicle.yaml> [--config <schema.yaml> ...]
```

`--config` is repeatable; later files overwrite earlier keys. All YAML files are merged
into a single parameter map before the pipeline is configured.

The output is always a directory. If `--output` already exists the converter exits with an
error; remove the path first or choose a different name.

## YAML Config Structure

The tool reads the same YAML format as the live ROS 2 node (`vehicle_can_node.ros__parameters`
namespace is stripped automatically if present).

### CAN-ID-based domain routing

```yaml
vehicle_can_node:
  ros__parameters:
    can_topic: /vehicle/from_can_bus
    domain_names: [dynamics, chassis]
    domains.dynamics.topic: /vehicle/dynamics
    domains.dynamics.can_ids: [256, 257] # 0x100, 0x101
    domains.chassis.topic: /vehicle/chassis
    domains.chassis.can_ids: [512, 513] # 0x200, 0x201
```

### Schema-based domain routing (takes precedence over CAN-ID routing)

```yaml
schema_domain_names: [dynamics, chassis]
schema.dynamics.topic: /vehicle/dynamics
schema.dynamics.signals: [VehicleSpeed, Acceleration]
schema.chassis.topic: /vehicle/chassis
schema.chassis.signals: [SteeringAngle, BrakePress]
```

### Signal name aliases

```yaml
alias_names: [VEH_SPD, STR_ANG]
aliases.VEH_SPD: VehicleSpeed
aliases.STR_ANG: SteeringAngle
```

Compound key `CAN{id}_{dbc_name}` is tried first before the bare DBC signal name, allowing
disambiguation of signals that share the same name across different CAN messages.

### Signal transforms

```yaml
transform_names: [VehicleSpeed, SteeringAngle]
transforms.VehicleSpeed.expression: "x / 3.6"
transforms.VehicleSpeed.unit: "m/s"
transforms.SteeringAngle.expression: "x * 3.14159 / 180"
transforms.SteeringAngle.unit: "rad"
```

### Signal ID table

```yaml
signal_id_names: [VehicleSpeed, SteeringAngle, BrakePress]
```

`name_id` in each `Signal` is the 1-based index into this list (0 = name not in table).

## Bag I/O

The tool uses **`rosbag2_cpp`** (`rosbag2_cpp::Reader` / `rosbag2_cpp::Writer`) for bag
reading and writing, and **`rclcpp::Serialization<T>`** for CDR encoding and decoding.
No hand-rolled serialization — message layout changes in upstream packages are handled
automatically by the generated typesupport.

Message schema bytes are **not embedded** in the output bag. Foxglove Studio cannot display
messages in offline mode without a running ROS 2 instance; use `ros2 bag play` with a
Foxglove live connection instead.

## CMakeLists.txt for the Converter Executable

```cmake
cmake_minimum_required(VERSION 3.14)
project(mcap_converter CXX)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(VCD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

include(FetchContent)

# dbcppp and exprtk (same pins as the main package)
FetchContent_Declare(dbcppp ...)
FetchContent_Declare(exprtk ...)

find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(vehicle_can_decoder REQUIRED)

find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# Compile only the three ROS-independent source files from the main package
add_library(vcd_core STATIC
  ${VCD_ROOT}/src/dbc_decoder.cpp
  ${VCD_ROOT}/src/signal_transformer.cpp
  ${VCD_ROOT}/src/signal_router.cpp)
target_include_directories(vcd_core PUBLIC ${VCD_ROOT}/include)
target_link_libraries(vcd_core PRIVATE libdbcppp exprtk)

add_executable(mcap_converter src/mcap_converter.cpp)
target_link_libraries(mcap_converter vcd_core yaml-cpp::yaml-cpp)
ament_target_dependencies(mcap_converter
  rosbag2_cpp rosbag2_storage rclcpp can_msgs vehicle_can_decoder)
```

## Using the Library in a Custom Tool

`DbcDecoder`, `SignalRouter`, and `SignalTransformer` have no ROS 2 dependency and can be
used directly in any C++ executable. The YAML config loading helpers are **not** part of
the installed library — they live in `tools/mcap_converter/src/mcap_converter.cpp` — but
they are self-contained and straightforward to copy into your own tool.

### Step 1 — Load and merge YAML configs

The helpers below mirror what `mcap_converter` does internally. `load_yaml` flattens a
YAML file into a flat `ParamMap` (dotted keys, e.g. `"domains.chassis.can_ids"`), and
`build_config` converts that map into typed structs.

```cpp
#include <yaml-cpp/yaml.h>
#include "vehicle_can_decoder/dbc_decoder.hpp"
#include "vehicle_can_decoder/signal_router.hpp"
#include "vehicle_can_decoder/signal_transformer.hpp"

using namespace vehicle_can_decoder;
using ParamMap = std::unordered_map<std::string, YAML::Node>;

// Recursively flatten a YAML map into dotted keys.
void flatten_yaml(const YAML::Node & node, const std::string & prefix, ParamMap & out)
{
    if (node.IsMap()) {
        for (const auto & kv : node) {
            const std::string key = prefix.empty()
                ? kv.first.as<std::string>()
                : prefix + "." + kv.first.as<std::string>();
            flatten_yaml(kv.second, key, out);
        }
    } else {
        out[prefix] = node;
    }
}

// Load one YAML file. Strips the vehicle_can_node.ros__parameters namespace
// automatically so the same files work for both this tool and the live node.
ParamMap load_yaml(const std::string & path)
{
    YAML::Node root = YAML::LoadFile(path);
    if (root["vehicle_can_node"] && root["vehicle_can_node"]["ros__parameters"])
        root = root["vehicle_can_node"]["ros__parameters"];
    ParamMap params;
    flatten_yaml(root, "", params);
    return params;
}

// Merge multiple config files — later files overwrite earlier keys.
ParamMap merged;
for (const std::string & path : config_paths) {
    for (const auto & kv : load_yaml(path))
        merged[kv.first] = kv.second;
}
```

### Step 2 — Build typed config from the merged ParamMap

These small helpers convert `ParamMap` entries to typed values:

```cpp
auto get_s = [](const ParamMap & p, const std::string & k, const std::string & def = "") {
    const auto it = p.find(k);
    return (it == p.end()) ? def : it->second.as<std::string>(def);
};
auto get_sv = [](const ParamMap & p, const std::string & k) {
    std::vector<std::string> v;
    const auto it = p.find(k);
    if (it != p.end() && it->second.IsSequence())
        for (const auto & e : it->second) v.push_back(e.as<std::string>());
    return v;
};
auto get_iv = [](const ParamMap & p, const std::string & k) {
    std::vector<int64_t> v;
    const auto it = p.find(k);
    if (it != p.end() && it->second.IsSequence())
        for (const auto & e : it->second) v.push_back(e.as<int64_t>());
    return v;
};

// Domains
std::vector<DomainConfig> domains;
for (const auto & name : get_sv(merged, "domain_names")) {
    DomainConfig dc;
    dc.name  = name;
    dc.topic = get_s(merged, "domains." + name + ".topic", "/vehicle/" + name);
    for (int64_t id : get_iv(merged, "domains." + name + ".can_ids"))
        dc.can_ids.insert(static_cast<uint32_t>(id));
    domains.push_back(std::move(dc));
}

// Aliases
std::unordered_map<std::string, std::string> aliases;
for (const auto & src : get_sv(merged, "alias_names"))
    aliases[src] = get_s(merged, "aliases." + src, src);

// Transforms
std::unordered_map<std::string, TransformConfig> transforms;
for (const auto & sig : get_sv(merged, "transform_names"))
    transforms[sig] = {get_s(merged, "transforms." + sig + ".expression"),
                       get_s(merged, "transforms." + sig + ".unit")};
```

### Step 3 — Initialize library components

```cpp
DbcDecoder decoder;
if (!decoder.load(dbc_path))
    throw std::runtime_error("Failed to load DBC: " + dbc_path);

SignalRouter router;
for (const auto & w : router.configure(domains, aliases))
    std::cerr << "Warning: " << w << "\n";

SignalTransformer transformer;
transformer.configure(transforms);
```

### Step 4 — Per-frame conversion

```cpp
// can_id, data, dlc come from whichever source delivers CAN frames.
const auto raw_signals = decoder.decode(can_id, data, dlc);
if (!raw_signals) return;  // unknown CAN ID — skip

for (const RawSignal & raw : *raw_signals) {
    // Compound key "CAN{id}_{name}" tried first to disambiguate signals
    // whose DBC names collide across different CAN messages.
    const std::string compound = "CAN" + std::to_string(can_id) + "_" + raw.name;
    const std::string & compound_alias = router.apply_alias(compound);
    const std::string & name =
        (compound_alias != compound) ? compound_alias : router.apply_alias(raw.name);

    const std::string & domain = router.domain_for_id(can_id);
    if (domain == SignalRouter::kUnassignedDomain) continue;

    // tr.value — transformed value (= raw.value if no expression configured)
    // tr.unit  — unit string from TransformConfig ("" if not configured)
    const TransformResult tr = transformer.transform(name, raw.value);

    // Use name, domain, tr.value, tr.unit as needed.
}
```

### CMakeLists.txt

**Standalone executable (no ROS 2 required)** — compile the three source files directly,
the same way `tools/mcap_converter` does:

```cmake
get_filename_component(VCD_ROOT "/path/to/vehicle_can_decoder" ABSOLUTE)

add_library(vcd_core STATIC
  ${VCD_ROOT}/src/dbc_decoder.cpp
  ${VCD_ROOT}/src/signal_transformer.cpp
  ${VCD_ROOT}/src/signal_router.cpp)
target_include_directories(vcd_core PUBLIC ${VCD_ROOT}/include)
target_link_libraries(vcd_core PRIVATE libdbcppp exprtk)  # fetched via FetchContent

find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

add_executable(my_tool src/my_tool.cpp)
target_link_libraries(my_tool vcd_core yaml-cpp::yaml-cpp)
```

If your tool also reads or writes ROS 2 bags, use `rosbag2_cpp` instead of the foxglove
mcap library — see `tools/mcap_converter` for the pattern.

**ROS 2 package (ament)** — link against the installed library:

```cmake
find_package(vehicle_can_decoder REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(my_tool src/my_tool.cpp)
ament_target_dependencies(my_tool vehicle_can_decoder)
target_link_libraries(my_tool yaml-cpp::yaml-cpp)
```

## Timestamp Handling

| Source                     | How to obtain                                                 |
| -------------------------- | ------------------------------------------------------------- |
| Input CAN frame time       | `can_msgs/Frame` header stamp (`sec` + `nanosec`)             |
| `SignalGroup.header.stamp` | Set from the same stamp — do **not** use wall clock           |
| `time_stamp` in output bag | Set from the CAN frame header stamp (nanoseconds)             |
| `publishTime`              | Not preserved — `rosbag2_cpp` has a single `time_stamp` field |

This ensures the output bag is fully reproducible regardless of the processing environment
or machine speed.

## Console Output

The converter prints a summary on completion:

```text
Converting: input_bag/ → out/
Done.
  CAN frames read:   12345
  Decoded:           11800
  Unknown CAN ID:    545
  SignalGroups out:  23600
  Passthrough msgs:  6789
```
