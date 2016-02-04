# Message to publish tactile contact information extracted from raw tactile data.
# This msg provides a list of contact points and corresponding
# contact wrenches, both represented w.r.t the link reference frame.

# specify the link frame the data is referencing to
Header header
# list of all contacts
tactile_msgs/TactileContact[] contacts
