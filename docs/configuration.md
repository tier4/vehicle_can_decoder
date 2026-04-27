# Configuration Reference

Configuration is via YAML. See `config/example_vehicle.yaml` for a complete example.

## Top-Level Parameters

```yaml
vehicle_can_node:
  ros__parameters:
    # ── Basic Identity ────────────────────────────────────
    can_topic: "/vehicle/from_can_bus" # Source topic for can_msgs/Frame
    # dbc_file is passed via the launch argument dbc_file:=

    # ── Timing ────────────────────────────────────────────
    signal_timeout_ms: 500 # Mark signal stale after this many ms
    diagnostics_rate_hz: 1.0 # How often to publish diagnostics
    schema_republish_interval_s: 1.0 # How often to re-publish VehicleSchema

    # ── Topics ────────────────────────────────────────────
    publish_all_signals: true # Publish firehose topic?
    all_signals_topic: "/vehicle/decoded_can" # Decoded CAN topic name
    diagnostics_topic: "/vehicle/diagnostics" # Diagnostics topic name
```

## Domain Configuration

Domains group related CAN IDs into a single ROS2 topic:

```yaml
# List of domain names (required)
domain_names: ["chassis", "powertrain"]

# Per-domain configuration
domains.chassis.topic: "/vehicle/chassis"
domains.chassis.can_ids: [0x100, 0x101, 0x102] # Hex or decimal

domains.powertrain.topic: "/vehicle/powertrain"
domains.powertrain.can_ids: [512, 513] # Decimal (same as 0x200, 0x201)
```

Signals from CAN IDs not in any domain are assigned to the `"unassigned"` domain.

## Signal Aliases

Rename DBC signal names to semantic names:

```yaml
alias_names: ["RAW_SIGNAL_A", "RAW_SIGNAL_B"]

aliases.RAW_SIGNAL_A: "steering_angle"
aliases.RAW_SIGNAL_B: "wheel_speed_front_left"
```

The alias is used in all downstream processing (transforms, promoted topics, etc.).

## Signal Transforms

Apply math expressions to raw decoded values (unit conversions, scaling, etc.):

```yaml
transform_names: ["VEHICLE_SPEED", "steering_angle"]

transforms.VEHICLE_SPEED.expression: "x / 3.6" # km/h → m/s
transforms.VEHICLE_SPEED.unit: "m/s"

transforms.steering_angle.expression: "x * pi / 180.0" # deg → rad
transforms.steering_angle.unit: "rad"
```

**Transform syntax** (exprtk):

- `x` is the raw DBC physical value
- Math constants: `pi`, `e`
- Operators: `+`, `-`, `*`, `/`, `^` (power), `%` (modulo)
- Functions: `sin()`, `cos()`, `tan()`, `sqrt()`, `abs()`, `min()`, `max()`, etc.

If a signal has no transform configured, its raw value is passed through unchanged.

## Promoted Signals

Publish selected signals as `std_msgs/Float64` for simple downstream consumers:

```yaml
promoted_signal_names: ["VEHICLE_SPEED", "steering_angle"]

promoted_signals.VEHICLE_SPEED.topic_suffix: "vehicle_speed"
promoted_signals.steering_angle.topic_suffix: "steering_angle"
```

Topic: `/vehicle/signals/<topic_suffix>` (e.g., `/vehicle/signals/vehicle_speed`)

Values are the transformed values (after applying the signal's transform, if any).

## DBC File Placement

DBC files are **not committed** to version control (often NDA-restricted). Place them on the system running the node and pass the path via the `dbc_file` launch argument:

```bash
mkdir -p /opt/vehicle_dbcs
cp vehicle_a.dbc /opt/vehicle_dbcs/

ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:=/path/to/config.yaml \
  dbc_file:=/opt/vehicle_dbcs/vehicle_a.dbc
```

The `dbc_file` argument must be an absolute path to an existing file. The node will fail to start if the DBC file cannot be opened.

When using both a schema YAML and a DBC-specific YAML, pass them together in the launch file
(see `config/vehicle_schema.yaml` and `config/pacmod_v3.yaml` for an example).
