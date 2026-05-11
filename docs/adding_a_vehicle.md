# Adding a New Vehicle

1. **Get the DBC file** for the vehicle and place it in a safe location (keep it out of version control if NDA-restricted):

   ```bash
   mkdir -p ~/vehicle_dbcs
   cp /path/to/new_vehicle.dbc ~/vehicle_dbcs/
   ```

2. **Create a vehicle config** by copying and customizing the example:

   ```bash
   cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml \
      ~/vehicle_dbcs/new_vehicle.yaml
   ```

3. **Edit the config file**:
   - Set `can_topic` to the ROS 2 topic publishing `can_msgs/Frame` (default: `/vehicle/from_can_bus`)
   - List all CAN message IDs from the DBC and group them into domains
   - For each signal, add a `transforms` entry if value conversion is needed
   - Add promoted signals if simple consumers need them

   See [configuration.md](configuration.md) for full parameter reference.

4. **Launch** with the new config:

   ```bash
   ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
     config_file:=~/vehicle_dbcs/new_vehicle.yaml \
     dbc_file:=~/vehicle_dbcs/new_vehicle.dbc
   ```

5. **Verify** by listening to topics:

   ```bash
   ros2 topic echo /vehicle/chassis
   ```
