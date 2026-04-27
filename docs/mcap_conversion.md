# Offline MCAP Conversion (Building a Converter Tool)

This document describes how to build a standalone tool that converts an MCAP file containing
raw CAN frames (`can_msgs/Frame`) into a new MCAP file containing the abstracted vehicle
signal topics (`vehicle_can_decoder/msg/SignalGroup`), without a running ROS runtime.

The tool lives at `tools/mcap_converter/` and has **no ROS2 installation dependency** — it
uses the Foxglove C++ MCAP library directly and implements CDR serialization inline.

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
can_msgs/Frame (from MCAP)
    │  inline CDR deserialize (cdr::Reader)
    ▼
CanFrameMsg { sec, nanosec, id, dlc, data[8] }
    │  DbcDecoder::decode()
    ▼
vector<RawSignal> { name, value, can_id }
    │  SignalRouter::apply_alias()        — compound key "CAN{id}_{name}" tried first
    │  schema routing (signal→domain)     — takes precedence over CAN-ID routing
    │  or SignalRouter::domain_for_id()   — fallback CAN-ID-based routing
    │  SignalTransformer::transform()
    ▼
per-domain SignalGroup { header, domain, Signal[] }
    │  inline CDR serialize (cdr::Writer)
    ▼
output MCAP (original logTime / publishTime preserved)

Non-CAN topics → copied verbatim (schemas and channels duplicated lazily)
```

## Message Definitions

### Input — `can_msgs/msg/Frame` (deserialized inline)

```text
std_msgs/Header header   # stamp: {int32 sec, uint32 nanosec}, string frame_id
uint32           id
bool             is_rtr
bool             is_extended
bool             is_error
uint8            dlc
uint8[8]         data
```

### Output — `vehicle_can_decoder/msg/SignalGroup` (serialized inline)

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

The tool uses **plain CMake** (no ament / colcon). Dependencies are fetched at build time.

```bash
cd tools/mcap_converter
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Optional compression support

Install system libraries before configuring to enable reading lz4/zstd-compressed bags:

```bash
sudo apt install liblz4-dev libzstd-dev
```

CMake detects them automatically and sets `MCAP_COMPRESSION_LZ4` /
`MCAP_COMPRESSION_ZSTD`.

## Usage

```bash
./build/mcap_converter \
  --input   <in.mcap>      \
  --output  <out.mcap>     \
  --dbc     <vehicle.dbc>  \
  --config  <vehicle.yaml> [--config <schema.yaml> ...]
```

`--config` is repeatable; later files overwrite earlier keys. All YAML files are merged
into a single parameter map before the pipeline is configured.

## YAML Config Structure

The tool reads the same YAML format as the live ROS2 node (`vehicle_can_node.ros__parameters`
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

## MCAP I/O

The tool uses the **Foxglove C++ MCAP library** (`mcap/reader.hpp`, `mcap/writer.hpp`),
fetched at build time from `https://github.com/foxglove/mcap.git` at tag
`releases/cpp/v1.4.1`. No `rclcpp::init` or ROS2 installation is required.

CDR serialization is implemented inline (`cdr::Reader` / `cdr::Writer` in
`src/mcap_converter.cpp`) — the 4-byte CDR-LE encapsulation header is handled, and each
field is aligned to its natural size boundary.

## CMakeLists.txt for the Converter Executable

```cmake
cmake_minimum_required(VERSION 3.14)
project(mcap_converter CXX)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(VCD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

include(FetchContent)

# Foxglove MCAP (header-only, no ROS2 dependency)
FetchContent_Declare(mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG        releases/cpp/v1.4.1
  GIT_SHALLOW    TRUE)
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
  FetchContent_Populate(mcap)
endif()
add_library(mcap INTERFACE)
target_include_directories(mcap INTERFACE "${mcap_SOURCE_DIR}/cpp/mcap/include")

# dbcppp and exprtk (same pins as the main package)
FetchContent_Declare(dbcppp ...)
FetchContent_Declare(exprtk ...)

find_package(yaml-cpp REQUIRED)

# Compile only the three ROS-independent source files from the main package
add_library(vcd_core STATIC
  ${VCD_ROOT}/src/dbc_decoder.cpp
  ${VCD_ROOT}/src/signal_transformer.cpp
  ${VCD_ROOT}/src/signal_router.cpp)
target_include_directories(vcd_core PUBLIC ${VCD_ROOT}/include)
target_link_libraries(vcd_core PRIVATE libdbcppp exprtk)

add_executable(mcap_converter src/mcap_converter.cpp)
target_link_libraries(mcap_converter vcd_core mcap yaml-cpp::yaml-cpp)
```

## Using the Library in a Custom Tool

`DbcDecoder`, `SignalRouter`, and `SignalTransformer` have no ROS2 dependency and can be
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

**Standalone executable (no ROS2 required)** — compile the three source files directly,
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

**ROS2 package (ament)** — link against the installed library:

```cmake
find_package(vehicle_can_decoder REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(my_tool src/my_tool.cpp)
ament_target_dependencies(my_tool vehicle_can_decoder)
target_link_libraries(my_tool yaml-cpp::yaml-cpp)
```

## Timestamp Handling

| Source                     | How to obtain                                       |
| -------------------------- | --------------------------------------------------- |
| Input CAN frame time       | `can_msgs/Frame` header stamp (`sec` + `nanosec`)   |
| `SignalGroup.header.stamp` | Set from the same stamp — do **not** use wall clock |
| MCAP `logTime`             | Copied from the input message's `msg.logTime`       |
| MCAP `publishTime`         | Copied from the input message's `msg.publishTime`   |

This ensures the output MCAP is fully reproducible regardless of the processing environment
or machine speed.

## Console Output

The converter prints a summary on completion:

```text
DBC: vehicle.dbc (42 messages)
Converting: input.mcap → output.mcap
Done.
  CAN frames read:   12345
  Decoded:           11800
  Unknown CAN ID:    545
  SignalGroups out:  23600
  Passthrough msgs:  6789
```
