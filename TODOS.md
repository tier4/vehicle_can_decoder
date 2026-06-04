# TODOs

## Schema embedding for Foxglove offline display

The `tools/mcap_converter` writes `.msg` definition files into a temporary ament prefix and
prepends it to `AMENT_PREFIX_PATH` before opening the output bag. This enables
`rosbag2_storage_mcap` to embed message schemas so Foxglove Studio can display messages
in offline mode without a running ROS 2 instance.

If schema embedding stops working after a `rosbag2_storage_mcap` upgrade, check whether
the plugin's schema-embedding behavior has changed and update the ament prefix workaround
in `tools/mcap_converter/src/mcap_converter.cpp` accordingly.
