# Troubleshooting

## Node fails to start: "No such file or directory"

- **Cause**: DBC file path does not exist or is relative
- **Fix**: Pass an absolute path via `dbc_file:=` in the launch command

## No CAN frames received

- **Check the bridge**: Verify `ros2_socketcan` (or your CAN source) is publishing to the topic
- **Verify traffic**: `ros2 topic hz /vehicle/from_can_bus` should show a non-zero rate
- **Check topic name**: `can_topic` in config must match the topic published by the bridge

## Stale signals reported in diagnostics (`timed_out_signals`)

- **Cause**: CAN messages not arriving within `signal_timeout_ms`
- **Fix**: Increase `signal_timeout_ms` or verify CAN traffic is present for those signals

## Decoding schema signal names from Signal.msg

- `Signal.name_id` is 0 when the signal has no schema entry — check that `signal_id_names` in your schema YAML covers the signal
- Use `/vehicle/schema` (transient_local) to resolve `name_id` to a name and unit string

## Transform expressions fail to compile

- **Cause**: Syntax error in exprtk expression or reference to undefined variable
- **Fix**: Check `transforms.*.expression` syntax; only `x` (the raw value) is available

## Topics not publishing

- Check node is running: `ros2 node list`
- Check topic names match config: `ros2 topic list`
- Check for errors in node output: `ros2 launch ... --launch-prefix="gdb -ex run --args"`
