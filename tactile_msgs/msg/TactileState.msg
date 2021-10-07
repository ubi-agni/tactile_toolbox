# Message to publish (raw) tactile data of one or more arrays of tactile sensors,
# i.e. a tactile skin
# For now this considers normal forces only. As is, this applies to
# UNIBI's myrmex, finger, and palm sensors as well as SynTouch.

# specify the timestamp (frame not used)
Header header

# a list of individual, named sensor arrays
sensor_msgs/ChannelFloat32[] sensors
