# vehicle_can_decoder

## ROS2 CAN-to-Topic Abstraction Layer

A config-driven ROS2 Humble C++ node that reads raw SocketCAN frames, decodes them using DBC files, applies value transformations, and publishes normalized vehicle signals to ROS2 topics.

**Key Feature**: Add new vehicles or signals by editing YAML and DBC files — no recompilation needed.

```text
can_msgs/Frame topic → DbcDecoder → SignalTransformer → SignalRouter → ROS2 Topics
  (/vehicle/from_can_bus)  (dbcppp)   (exprtk math)     (domains)     (SignalGroup)
                                                                        (std_msgs/Float64)
```

---

## Quick Start

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select vehicle_can_decoder
source install/setup.bash
```

### 2. Provide a CAN Frame Source

```bash
# Real hardware
ros2 launch ros2_socketcan socket_can_receiver.launch.xml \
  interface:=can0 topic_name:=/vehicle/from_can_bus

# Virtual CAN for testing
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 launch ros2_socketcan socket_can_receiver.launch.xml \
  interface:=vcan0 topic_name:=/vehicle/from_can_bus
```

### 3. Create a Vehicle Config

```bash
cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml ~/my_vehicle.yaml
# Edit ~/my_vehicle.yaml: set can_topic, domains, aliases, transforms
```

### 4. Launch

```bash
ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:=/absolute/path/to/my_vehicle.yaml \
  dbc_file:=/absolute/path/to/vehicle.dbc
```

### 5. Verify

```bash
ros2 topic echo /vehicle/schema
ros2 topic echo /vehicle/chassis
ros2 topic echo /vehicle/diagnostics
```

---

## Output Topics

| Topic                       | Type               | Description                                               |
| --------------------------- | ------------------ | --------------------------------------------------------- |
| `/vehicle/schema`           | `VehicleSchema`    | Signal name/unit lookup table; transient_local QoS        |
| `/vehicle/decoded_can`      | `SignalGroup`      | All decoded signals per tick                              |
| `/vehicle/<domain>`         | `SignalGroup`      | Per-domain signals (if `schema_publish_per_domain: true`) |
| `/vehicle/signals/<suffix>` | `std_msgs/Float64` | Promoted individual signals                               |
| `/vehicle/diagnostics`      | `SignalDiagnostic` | Frame counts, timeouts, errors                            |

---

## Dependencies

| Package                                   | Purpose                                        |
| ----------------------------------------- | ---------------------------------------------- |
| **rclcpp**, **std_msgs**, **ament_cmake** | ROS2 Humble                                    |
| **dbcppp**                                | DBC file parsing (fetched via CMake)           |
| **exprtk**                                | Math expression evaluation (fetched via CMake) |
| **ament_cmake_gtest**                     | Unit tests                                     |

---

## Documentation

| Topic                        | Link                                                   |
| ---------------------------- | ------------------------------------------------------ |
| Architecture & Message Types | [docs/architecture.md](docs/architecture.md)           |
| Configuration Reference      | [docs/configuration.md](docs/configuration.md)         |
| Adding a New Vehicle         | [docs/adding_a_vehicle.md](docs/adding_a_vehicle.md)   |
| Testing with Virtual CAN     | [docs/testing.md](docs/testing.md)                     |
| MCAP Offline Conversion      | [docs/mcap_conversion.md](docs/mcap_conversion.md)     |
| Abstracted Schema Design     | [docs/abstracted_schema.md](docs/abstracted_schema.md) |
| Troubleshooting              | [docs/troubleshooting.md](docs/troubleshooting.md)     |
| Performance Notes            | [docs/performance.md](docs/performance.md)             |
| Known Issues                 | [docs/known_issues.md](docs/known_issues.md)           |

> **Attribution**: CAN message and signal names used in this project's DBC files and
> documentation are derived solely from publicly available open-source repositories
> ([commaai/opendbc](https://github.com/commaai/opendbc),
> [astuff/pacmod_dbc](https://github.com/astuff/pacmod_dbc)).
> No proprietary or confidential OEM data is included.
> See [docs/abstracted_schema.md](docs/abstracted_schema.md#data-sources-and-attribution) for details.

---

## License

Apache License 2.0
