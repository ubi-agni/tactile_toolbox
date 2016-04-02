# Message describing the net effect of a single contact

# specify link frame and timestamp of contact
Header header

# unique name of the contact
string name
# contact point location
geometry_msgs/Point position
# surface normal at contact point
geometry_msgs/Vector3 normal
# wrench applied through this contact point
# this wrench should be represented in reference frame, not in contact point frame
geometry_msgs/Wrench wrench
