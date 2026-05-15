# A group of decoded CAN signals belonging to the same domain.
#
# Published on:
#   /vehicle/<domain>        - per-domain topic (e.g. /vehicle/chassis)
#   /vehicle/decoded_can     - firehose topic containing every decoded signal (domain="all")

std_msgs/Header header

# Domain name as defined in the YAML config (e.g. "chassis", "powertrain").
# Set to "all" on the firehose topic.
string domain

vehicle_can_decoder/Signal[] signals
