tactile_msgs
============

This package contains generic messages to store almost all types of tactile data
to facilitate the developments of tools using/processing this tactile data


## Concept

### TactileState

The design is based on the successful [sensor_msgs/JointState.msg](sensor_msgs/JointState.msg)

Tactile raw data might come from **different sensors**, on different topics.
Hence it is important to have messages that can be easily **merged together**.
The **sensor field** stores each **tactile array** coming from one sensor.

The main difference with *joint_state* is that the different sensors might have different data vectors.
For this reason a [sensor_msgs/ChannelFloat32.msg](sensor_msgs/ChannelFloat32.msg) message is used per sensor, 
it contains a **name** and a **data vector**.

The **name** of the sensor publishing the data should be **unique**, to permit merging,
the same way joint_names are be unique.

This **name** is used in the *robot_description* to **define** each sensor position and characteristics
through a **sensor** tag and some geometries and property fields.

A *tactile_state_publisher* similar to the *joint_state_publisher* can take care of multiple sources 
and merge them into a single topic */tactile_states*


### TactileContact

A [TactileContact.msg](TactileContact.msg) is a **processed** and maybe **calibrated** tactile **information** about the location, the normal and the wrench of the contact.

Usually the raw data array of a tactile sensor will be post-processed
 to better locate the center of contact by weighted average of neighbour tactile cells.

If the sensor is not flat, the normal can also be extrapolated from normals of neighbour cells.
 
The [TactileContact.msg](TactileContact.msg) is designed to accommodate a single calibrated cell
or a processed weighted average over an array of cells, or any blob location on a larger array.

### TactileContacts

Contains one [TactileContact.msg](TactileContact.msg) or more to store multiple contacts per *frame_id*.

## Compatibility

The messages of some common tactile sensors have been evaluated to check compatibility with the tactile_msgs

* The MID Tactile fingertips of University Bielefeld use [sr_robot_msgs/UBI0All]() that are fully compatible
* The tactile skin/flesh of University Bielefeld uses [sr_robot_msgs/MidProxAll]() that are fully compatible
* Biotac tactile tip of SyntouchLLC use [sr_robot_msgs/BiotacAll]() that are fully compatible if the naming of the raw data contains a suffix matching the individual sensors of a tip (pac pdc tac tdc hall).
* Roboskin tactile system (Patent No. I0128764) uses 12 x 16bit raw data array per sensor that are fully compatible
* The F/T tips of King's College London, installed on the Shadow hand use [sr_robot_msgs/ShadowContactStateStamped]() which are compatible with TactileContacts with a re-encoding of the separate components into the wrench. ```wrench/torque = Ltorque*normal, wrench/force = Fnormal*normal + tangential_force```. Single components can be recovered using the normal.
* Most arrays can fit the single vector of the [sensor_msgs/ChannelFloat32.msg](sensor_msgs/ChannelFloat32.msg) and have the meaning/ordering described in a _robot_description_
