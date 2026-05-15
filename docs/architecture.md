# Architecture

## Components

| Module                | Purpose                                                                                   | Key Classes                            |
| --------------------- | ----------------------------------------------------------------------------------------- | -------------------------------------- |
| **DbcDecoder**        | Loads DBC files; decodes CAN frames to signal name-value pairs                            | `DbcDecoder`, `RawSignal`              |
| **SignalTransformer** | Compiles and evaluates exprtk math expressions for value transforms                       | `SignalTransformer`, `TransformConfig` |
| **SignalRouter**      | Routes signals to domains; applies signal name aliases                                    | `SignalRouter`, `DomainConfig`         |
| **TimeoutMonitor**    | Tracks signal freshness; detects stale signals                                            | `TimeoutMonitor`                       |
| **CanReader**         | Provides the `CanFrame` struct (used by DbcDecoder and MCAP tooling)                      | `CanFrame`                             |
| **VehicleCanNode**    | ROS 2 node: subscribes to `can_msgs/Frame`, orchestrates all components, publishes topics | `VehicleCanNode`                       |

## Signal Flow

```text
can_msgs/Frame topic → DbcDecoder → SignalTransformer → SignalRouter → ROS 2 Topics
  (/vehicle/from_can_bus)  (dbcppp)   (exprtk math)     (domains)     (SignalGroup)
                                                                        (std_msgs/Float64)
```

1. **`can_msgs/Frame` message arrives** on the subscribed topic
2. **DbcDecoder** looks up the CAN ID in the loaded DBC and decodes signals
3. **SignalRouter** applies aliases and determines the domain
4. **SignalTransformer** applies per-signal math transformations
5. **TimeoutMonitor** checks if the signal is fresh (within `signal_timeout_ms`)
6. **VehicleCanNode** batches signals by domain and publishes to ROS 2 topics

## Message Types

### Signal.msg

A single decoded signal (compact, schema-indexed):

- `name_id` (`uint16`) — 1-indexed key into `VehicleSchema.signal_table`; resolve name and unit via `signal_table[name_id-1]`
- `value` (`float32`) — Transformed value

Name and unit strings are **not** carried per-frame. Resolve them once from the `/vehicle/schema` topic:

```text
signal_name = schema.signal_table[signal.name_id - 1].name
unit_id     = schema.signal_table[signal.name_id - 1].unit_id
unit_string = schema.unit_table[unit_id - 1]
```

### SignalEntry.msg

One row in the signal lookup table:

- `id` (`uint16`) — 1-indexed ID matching `Signal.name_id`
- `name` (`string`) — Canonical signal name (e.g., `"dynamics.speed.longitudinal"`)
- `unit_id` (`uint16`) — 1-indexed reference into `VehicleSchema.unit_table`

### VehicleSchema.msg

Published at startup and periodically on `/vehicle/schema` with transient_local QoS (see `schema_republish_interval_s`):

- `header` — Standard ROS 2 header
- `schema_version` (`string`) — Semantic version (major bump = breaking bag change)
- `signal_table[]` — Array of `SignalEntry`; index by `name_id - 1`
- `unit_table[]` (`string[]`) — Unit strings; index by `unit_id - 1`

### SignalGroup.msg

Grouped signals from one domain:

- `header` — ROS 2 standard header with timestamp
- `domain` — Domain name (e.g., `"chassis"`); `"all"` for the firehose topic
- `signals[]` — Array of `Signal.msg`

### SignalDiagnostic.msg

Node health (published at `diagnostics_rate_hz`):

- `header` — ROS 2 standard header with timestamp
- `can_topic` — ROS 2 topic name providing CAN frames
- `frames_received` — Total `can_msgs/Frame` messages received
- `frames_decoded` — Frames successfully matched to a DBC message
- `frames_unknown` — Frames with no matching DBC message
- `decode_errors` — Frames matched but failed to decode signals
- `timed_out_signals[]` — Names of signals currently flagged as stale by `TimeoutMonitor`
