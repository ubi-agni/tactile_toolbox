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
