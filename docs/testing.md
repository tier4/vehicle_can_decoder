# Testing with Virtual CAN

## 1. Set Up Virtual CAN

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## 2. Bridge vcan0 to a ROS2 Topic

```bash
ros2 launch ros2_socketcan socket_can_receiver.launch.xml \
  interface:=vcan0 topic_name:=/vehicle/from_can_bus
```

## 3. Send Test Frames

Use `cansend` (from `can-utils`) while the bridge is running:

```bash
# Send a frame with ID 0x100 and 8 bytes of data
cansend vcan0 100#0102030405060708

# Monitor while sending
candump vcan0 &
cansend vcan0 100#1122334455667788
```

## 4. Run Unit Tests

```bash
cd ~/ros2_ws
colcon test --packages-select vehicle_can_decoder --event-handlers console_direct+
```

## 5. Run with Virtual Interface

```bash
cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml ~/test_vehicle.yaml
# Edit test_vehicle.yaml: set can_topic: "/vehicle/from_can_bus"

ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:=~/test_vehicle.yaml \
  dbc_file:=/path/to/test.dbc
```
