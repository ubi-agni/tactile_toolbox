# tactile_toolbox - Handling tactile sensors in ROS

This software package adds tactile messages, a tactile sensor description to URDF, as well as visualization tools to ROS.

* [tactile_msgs](tactile_msgs): ROS messages to publish [raw tactile data](tactile_msgs/msg/TactileState.msg) as well as [contact information](tactile_msgs/msg/TactileContact.msg).
* [urdf_tactile](urdf_tactile): extension to URDF to describe tactile sensors, requires a modified [urdf package](https://github.com/ubi-agni/robot_model) and [urdfdom 0.5](https://github.com/ubi-agni/urdfdom).
* [tactile_merger](tactile_merger): Compute [contact information](tactile_msgs/msg/TactileContacts.msg) from [raw tactile data](tactile_msgs/msg/TactileState.msg) and taxel geometry.
* [tactile_state_publisher](tactile_state_publisher): Re-publish [raw tactile data](tactile_msgs/msg/TactileState.msg), allowing to merge different sources into a new publisher.
* [tactile_pcl](tactile_pcl): Compute and publish Tactile Point Cloud data from [raw tactile data](tactile_msgs/msg/TactileState.msg).
* [rviz_tactile_plugins](rviz_tactile_plugins): rviz visualization tools for [raw tactile data](tactile_msgs/msg/TactileState.msg) and [contact information](tactile_msgs/msg/TactileContacts.msg).
