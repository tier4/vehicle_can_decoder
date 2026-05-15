# Configuration Reference

Configuration is via YAML. See `config/example_vehicle.yaml` for an all-in-one example,
or `config/vehicle_schema.yaml` + `config/pacmod_v3.yaml` for the split-schema layout.

## Top-Level Parameters

```yaml
vehicle_can_node:
  ros__parameters:
    # ── CAN source ────────────────────────────────────────
    can_topic: "/vehicle/from_can_bus" # Source topic for can_msgs/Frame

    # dbc_file is always passed via the launch argument dbc_file:=
    # (never stored in the YAML config)

    # ── Timing ────────────────────────────────────────────
    signal_timeout_ms: 500 # Mark signal stale after this many ms (must be > 0)
    diagnostics_rate_hz: 1.0 # How often to publish diagnostics (must be > 0)
    schema_republish_interval_s: 1.0 # How often to re-publish VehicleSchema (must be > 0)

    # ── Firehose topic ─────────────────────────────────────
    publish_all_signals: true # Publish firehose SignalGroup topic?
    all_signals_topic: "/vehicle/decoded_can" # Firehose topic name

    # ── Diagnostics ────────────────────────────────────────
    diagnostics_topic: "/vehicle/diagnostics"

    # ── Schema mode ────────────────────────────────────────
    # Set schema_domain_names to enable schema mode (see "Schema Mode" section below).
    schema_publish_per_domain: true # Publish per-domain topics in schema mode
```

## Operating Modes

The node supports two routing modes. They are mutually exclusive — use one or the other per deployment.

### Mode A — CAN-ID Domain Routing (all-in-one config)

Signals are routed to domains based on CAN frame IDs. Any signal whose frame ID does not
match any domain's `can_ids` list is silently discarded (it is not published anywhere).

```yaml
# List of domain names (required)
domain_names: ["chassis", "powertrain"]

# Per-domain configuration
domains.chassis.topic: "/vehicle/chassis"
domains.chassis.can_ids: [0x100, 0x101, 0x102] # Hex or decimal

domains.powertrain.topic: "/vehicle/powertrain"
domains.powertrain.can_ids: [512, 513] # Decimal (same as 0x200, 0x201)
```

### Mode B — Schema Mode (split schema + DBC-specific config)

Signals are routed by canonical signal name rather than CAN ID. Signals not listed in any
`schema.<domain>.signals` are silently discarded.

```yaml
# Semantic version of the schema (major.minor.patch)
schema_version: "1.1.0"

# List of schema domain names — enables schema mode when non-empty
schema_domain_names: ["dynamics", "operation", "body"]

# Per-domain configuration
schema.dynamics.topic: "/vehicle/decoded_can"
schema.dynamics.signals:
  - dynamics.speed.longitudinal
  - dynamics.wheel_speed.front_left
  - dynamics.wheel_speed.front_right

schema.operation.topic: "/vehicle/decoded_can"
schema.operation.signals:
  - operation.steering.report.angle
  - operation.throttle.report.position
  - operation.brake.report.position

schema.body.topic: "/vehicle/decoded_can"
schema.body.signals:
  - body.lights.turn_signal
  - body.lights.hazard

# Suppress per-domain topics — all schema signals go to the firehose only
schema_publish_per_domain: false
```

#### Signal ID and Unit ID Tables

Used to populate `Signal.name_id` and `SignalEntry.unit_id` in published messages.
These tables enable compact bag recording without string fields per sample.

```yaml
# List position (1-indexed) becomes name_id in Signal.msg.
# NEVER reorder or remove — append only to preserve bag compatibility.
signal_id_names:
  - dynamics.speed.longitudinal # id=1
  - dynamics.wheel_speed.front_left # id=2

# Unit for each signal, parallel to signal_id_names.
# Values must appear in unit_id_names below.
signal_id_unit_names:
  - m/s # id=1
  - m/s # id=2

# List position (1-indexed) becomes unit_id in SignalEntry.msg.
# NEVER reorder or remove — append only.
unit_id_names:
  - "" # id=1  dimensionless / normalized
  - m/s # id=2
  - rad/s # id=3
  - rad # id=4
```

`signal_id_unit_names` must have the same length as `signal_id_names`. A warning is logged
for any unit name not found in `unit_id_names`; the signal's `unit_id` is left as 0.

## Signal Aliases

Rename DBC signal names to semantic names. Applies in both modes.

```yaml
alias_names: ["RAW_SIGNAL_A", "RAW_SIGNAL_B"]

aliases.RAW_SIGNAL_A: "steering_angle"
aliases.RAW_SIGNAL_B: "wheel_speed_front_left"
```

The alias is used in all downstream processing (transforms, promoted topics, schema routing).

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

Values are the post-transform values. Promoted signals are published regardless of whether
the signal is assigned to a domain.

## DBC File Placement

DBC files are **not committed** to version control (often NDA-restricted). Place them on the
system running the node and pass the path via the `dbc_file` launch argument:

```bash
mkdir -p /opt/vehicle_dbcs
cp vehicle_a.dbc /opt/vehicle_dbcs/

# All-in-one config
ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:=/path/to/my_vehicle.yaml \
  dbc_file:=/opt/vehicle_dbcs/vehicle_a.dbc

# Split schema + DBC-specific config
ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  schema_file:=/path/to/vehicle_schema.yaml \
  config_file:=/path/to/pacmod_v3.yaml \
  dbc_file:=/opt/vehicle_dbcs/vehicle_a.dbc
```

The `dbc_file` argument must be an absolute path to an existing file. The node will fail
to start if the DBC file cannot be opened.

## Parameter Layering

When using the launch file, parameters are merged in this order (later entries win):

```
launch defaults < schema_file < config_file < dbc_file (CLI argument)
```

This means `dbc_file` always takes precedence, and individual parameters (e.g. `can_topic`)
can be overridden from the CLI without editing any YAML file:

```bash
ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:=/path/to/config.yaml \
  dbc_file:=/path/to/vehicle.dbc \
  signal_timeout_ms:=1000 \
  diagnostics_rate_hz:=2.0
```
