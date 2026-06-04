# Adding a New Vehicle

1. **Get the DBC file** for the vehicle and place it in a safe location (keep it out of version control if NDA-restricted):

   ```bash
   mkdir -p ~/vehicle_dbcs
   cp /path/to/new_vehicle.dbc ~/vehicle_dbcs/
   ```

2. **Create a vehicle config**. Two layouts are supported:

   **A) All-in-one** — copy and edit the example config:

   ```bash
   cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml \
      ~/vehicle_dbcs/new_vehicle.yaml
   ```

   **B) Split schema + DBC-specific** — use the shared schema with a new DBC-specific file:

   ```bash
   cp ~/ros2_ws/src/vehicle_can_decoder/config/pacmod_v3.yaml \
      ~/vehicle_dbcs/new_vehicle.yaml
   # Adjust alias_names and transform_names for the new DBC
   ```

3. **Edit the config file**:
   - Set `can_topic` to the ROS 2 topic publishing `can_msgs/Frame` (default: `/vehicle/from_can_bus`)
   - **Mode A**: List CAN message IDs and group them into `domain_names`
   - **Mode B**: Add `alias_names` and `transform_names` to map DBC signals to canonical names
   - For each signal, add a `transforms` entry if value conversion is needed
   - Add promoted signals if simple consumers need them

   See [configuration.md](configuration.md) for full parameter reference.

4. **Launch** with the new config:

   ```bash
   # Mode A (all-in-one)
   ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
     config_file:=~/vehicle_dbcs/new_vehicle.yaml \
     dbc_file:=~/vehicle_dbcs/new_vehicle.dbc

   # Mode B (split schema + DBC-specific)
   ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
     schema_file:=~/ros2_ws/src/vehicle_can_decoder/config/vehicle_schema.yaml \
     config_file:=~/vehicle_dbcs/new_vehicle.yaml \
     dbc_file:=~/vehicle_dbcs/new_vehicle.dbc
   ```

5. **Verify** by listening to topics:

   ```bash
   # Mode A: per-domain topics (e.g. /vehicle/chassis) as configured in domain_names
   ros2 topic echo /vehicle/chassis

   # Mode B: all signals go to the firehose topic by default
   ros2 topic echo /vehicle/decoded_can
   ```
