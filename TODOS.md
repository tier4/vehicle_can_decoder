# TODOs

## Schema embedding for Foxglove offline display

Investigate `rosbag2_storage_mcap`'s schema configuration option to re-enable
message definition embedding in output bags. Currently the output bag has no
embedded schema, so Foxglove Studio cannot display messages without a running
ROS 2 instance. Workaround: use `ros2 bag play` + Foxglove live connection.

Blocked on: understanding the rosbag2_storage_mcap plugin's schema embedding API.
