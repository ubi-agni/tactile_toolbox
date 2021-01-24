# tactile_toolbox - Handling tactile sensors in ROS

This software package adds tactile messages, a tactile sensor description to URDF, as well as visualization tools to ROS. It comprises the following ROS packages:

* [tactile_msgs](tactile_msgs/README.md): ROS messages to describe [raw tactile data](tactile_msgs/msg/TactileState.msg) as well as high-level [contact information](tactile_msgs/msg/TactileContact.msg).
* [tactile_state_publisher](tactile_state_publisher)
  * `tactile_state_publisher`: Re-publish [TactileState msgs](tactile_msgs/msg/TactileState.msg), allowing to merge different sources into a new topic.
  * `tactile_bias`: Re-publish [TactileState msgs](tactile_msgs/msg/TactileState.msg) allowing to reset tactile bias on demand (tare)
* [tactile_calibration](tactile_calibration)
  * `tactile_state_calibrator`: maps raw tactile values to calibrated ones
* [urdf_tactile](urdf_tactile/README.md): extension to URDF to describe tactile sensors, requires modified [`urdf`](https://github.com/ubi-agni/urdf) and [`urdfdom`](https://github.com/ubi-agni/urdfdom) packages.
* [tactile_merger](tactile_merger): Map [`tactile_msgs/TactileState`](tactile_msgs/msg/TactileState.msg) onto [`tactile_msgs/TactileContacts`](tactile_msgs/msg/TactileContacts.msg), i.e. contact position, force, and normal, merging contact data of all taxels per link into a single wrench vector
* [tactile_pcl](tactile_pcl): Compute and publish Tactile Point Cloud data from [`tactile_msgs/TactileContacts`](tactile_msgs/msg/TactileState.msg).
* [rviz_tactile_plugins](rviz_tactile_plugins): rviz visualization tools for [raw tactile data](tactile_msgs/msg/TactileState.msg) and [contact information](tactile_msgs/msg/TactileContacts.msg).

## Install Instructions

### Pre-Requirements

* Ubuntu Bionic with ROS Melodic:
  ```bash
  sudo apt install python-wstool python-rosdep python-catkin-tools
  ```
* or Ubuntu Focal with ROS Noetic:
  ```bash
  sudo apt install python3-wstool python3-rosdep python3-catkin-tools
  ```

### Setup and build catkin workspace
```bash
export CATKIN_WS=~/catkin_ws  # Change to your preferred location
export ROS_DISTRO=melodic     # Change to your ROS distro

mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin config --extend /opt/ros/${ROS_DISTRO}

# Fetch sources from rosinstall file
cd src
wstool init
wstool merge https://raw.githubusercontent.com/ubi-agni/tactile_toolbox/melodic-devel/rosinstall
wstool update

# Install dependencies
rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

# Build the workspace
catkin build

# Source the workspace
source $CATKIN_WS/devel/setup.bash
```
