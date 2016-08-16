/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Contact controller
 * Author: Lars Oetermann
 * original of bumper controller Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

#include <map>
#include <string>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>

#include <tf/tf.h>

#include <gazebo_ros_contact/gazebo_ros_contact.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosContact)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosContact::GazeboRosContact() : SensorPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosContact::~GazeboRosContact()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosContact::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->parentSensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(_parent);
  if (!this->parentSensor)
  {
    ROS_ERROR("Contact sensor parent is not of type ContactSensor");
    return;
  }

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name: "
  //   << this->bumper_topic_name_ << std::endl;
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->GetElement("bumperTopicName"))
    this->bumper_topic_name_ =
      _sdf->GetElement("bumperTopicName")->Get<std::string>();

  // "transform contact/collisions pose, forces to this body (link) name: "
  //   << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  this->contact_pub_ = this->rosnode_->advertise<tactile_msgs::TactileContact>(
    std::string(this->bumper_topic_name_), 1);

  // Initialize
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread(
      boost::bind(&GazeboRosContact::ContactQueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(
     boost::bind(&GazeboRosContact::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosContact::OnContact()
{
  if (this->contact_pub_.getNumSubscribers() <= 0)
    return;

  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();

  // define frame_name and stamp
  this->tactile_contact_msg.header.frame_id = this->frame_name_;
  this->tactile_contact_msg.header.stamp = ros::Time(contacts.time().sec(),
                              contacts.time().nsec());

  // get reference frame (body(link)) pose and subtract from it to get
  // relative force, torque, position and normal vectors
  math::Pose pose, frame_pose;
  math::Quaternion rot, frame_rot;
  math::Vector3 pos, frame_pos;
  /* This code was commented out by gazebo. Do we need this?
  if (myFrame)
  {
    frame_pose = myFrame->GetWorldPose();  //-this->myBody->GetCoMPose();
    frame_pos = frame_pose.pos;
    frame_rot = frame_pose.rot;
  }
  else
  */
  {
    // no specific frames specified, use identity pose, keeping
    // relative frame at inertial origin
    frame_pos = math::Vector3(0, 0, 0);
    frame_rot = math::Quaternion(1, 0, 0, 0);  // gazebo u,x,y,z == identity
    frame_pose = math::Pose(frame_pos, frame_rot);
  }

  geometry_msgs::Wrench total_wrench;
  total_wrench.force.x = 0;
  total_wrench.force.y = 0;
  total_wrench.force.z = 0;
  total_wrench.torque.x = 0;
  total_wrench.torque.y = 0;
  total_wrench.torque.z = 0;

  geometry_msgs::Point total_position;
  total_position.x = 0;
  total_position.y = 0;
  total_position.z = 0;
  double total_normal_lengths = 0;

  geometry_msgs::Point average_position;
  average_position.x = 0;
  average_position.y = 0;
  average_position.z = 0;
  geometry_msgs::Vector3 total_normal;
  total_normal.x = 0;
  total_normal.x = 0;
  total_normal.x = 0;


  // Loop over Collisions
  // GetContacts returns all contacts on the collision body
  unsigned int contactsPacketSize = contacts.contact_size();
  for (unsigned int i = 0; i < contactsPacketSize; ++i)
  {

    gazebo::msgs::Contact contact = contacts.contact(i);

    // Loop over Contacts
    unsigned int contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j)
    {

      // Get force, torque and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)
      math::Vector3 force = frame_rot.RotateVectorReverse(math::Vector3(
                              contact.wrench(j).body_1_wrench().force().x(),
                            contact.wrench(j).body_1_wrench().force().y(),
                            contact.wrench(j).body_1_wrench().force().z()));
      math::Vector3 torque = frame_rot.RotateVectorReverse(math::Vector3(
                            contact.wrench(j).body_1_wrench().torque().x(),
                            contact.wrench(j).body_1_wrench().torque().y(),
                            contact.wrench(j).body_1_wrench().torque().z()));

      // vector sum of forces and torques
      total_wrench.force.x  += force.x;
      total_wrench.force.y  += force.y;
      total_wrench.force.z  += force.z;
      total_wrench.torque.x += torque.x;
      total_wrench.torque.y += torque.y;
      total_wrench.torque.z += torque.z;

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      math::Vector3 normal = frame_rot.RotateVectorReverse(
          math::Vector3(contact.normal(j).x(),
                        contact.normal(j).y(),
                        contact.normal(j).z()));


      // vector sum of normals
      total_normal.x += normal.x;
      total_normal.y += normal.y;
      total_normal.z += normal.z;

      double normal_length = normal.GetLength();

      // transform contact positions into relative frame
      // set contact positions
      gazebo::math::Vector3 position = frame_rot.RotateVectorReverse(
          math::Vector3(contact.position(j).x(),
                        contact.position(j).y(),
                        contact.position(j).z()) - frame_pos);


      // average position weighted on normal length
      total_position.x = position.x * normal_length;
      total_position.y = position.y * normal_length;
      total_position.z = position.z * normal_length;

      total_normal_lengths += normal_length;

    }
  }

  // compute average but avoid division by zero
  if(total_normal_lengths != 0) {
      average_position.x = total_position.x / total_normal_lengths;
      average_position.y = total_position.y / total_normal_lengths;
      average_position.z = total_position.z / total_normal_lengths;
  }

  // fill and publish message
  tactile_contact_msg.name = bumper_topic_name_;
  tactile_contact_msg.position = average_position;
  tactile_contact_msg.normal = total_normal;
  tactile_contact_msg.wrench = total_wrench;


  this->contact_pub_.publish(this->tactile_contact_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosContact::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
