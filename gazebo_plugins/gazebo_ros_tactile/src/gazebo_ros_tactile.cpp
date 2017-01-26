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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

/*
 * FRAME info
 * Gazebo provides positions and normals of contact in world frame
 * BUT Gazebo provides force and torques in link frame
 */

#include <map>
#include <string>
#include <vector>

#include <gazebo/common/Exception.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_ros_tactile/gazebo_ros_tactile.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tf/tf.h>

#include <urdf/sensor.h>
#include <urdf_tactile/tactile.h>
// #include <ros/console.h>
// #include <urdf_tactile/taxel_info_iterator.h>
#include <urdf_tactile/cast.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTactile)

// //////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTactile::GazeboRosTactile() : SensorPlugin()
{
}

// //////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTactile::~GazeboRosTactile()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

// //////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTactile::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parentSensor = dynamic_pointer_cast<sensors::ContactSensor>(_parent);
  if (!this->parentSensor)
  {
    ROS_ERROR("Contact sensor parent is not of type ContactSensor");
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = _parent->WorldName();
#else
  std::string worldName = _parent->GetWorldName();
#endif
  this->world_ = physics::get_world(worldName);

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name: "
  //   << this->bumper_topic_name_ << std::endl;
  this->bumper_topic_name_ = "bumper_states";
  this->tactile_topic_name_ = "tactile_states";
  if (_sdf->HasElement("bumperTopicName"))
    this->bumper_topic_name_ = _sdf->GetElement("bumperTopicName")->Get<std::string>();
  if (_sdf->HasElement("tactileTopicName"))
    this->tactile_topic_name_ = _sdf->GetElement("tactileTopicName")->Get<std::string>();

// "get the body (link) name to which the sensor is attached"
#if GAZEBO_MAJOR_VERSION >= 7
  std::string parentName = _parent->ParentName();
#else
  std::string parentName = _parent->GetParentName();
#endif
  local_name_ = parentName.substr(parentName.find_last_of(':') + 1);
  ROS_DEBUG_STREAM("contact plugin belongs to link named: " << local_name_);

  // by default use collision_name as parentName:local_name_ _collision
  collision_name_ = parentName + "::" + local_name_ + "_collision";

  // try access the real collision name used for the contact sensor (one level up from the plugin sdf)
  if (_sdf->GetParent()->HasElement("contact"))
  {
    if (_sdf->GetParent()->GetElement("contact")->HasElement("collision"))
    {
      collision_name_ = _sdf->GetParent()->GetElement("contact")->GetElement("collision")->Get<std::string>();
      collision_name_ = parentName + "::" + collision_name_;
    }
  }

  // "transform contact/collisions pose, forces to this body (link) name: "
  //   << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  // Because forces and torques are in local frame, a transformation is needed
  // Because position and normals are given in world another transformation is needed

  // Two frames are needed. The local frame, of the current link
  // and the frame of the user selected frame

  // lock in case a model is being spawned
  // boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
  physics::Model_V all_models = world_->GetModels();

  // if frameName specified is "world", "/map" or "map" report back
  // inertial values in the gazebo world.
  if (this->my_link_ == NULL && this->frame_name_ != "world" && this->frame_name_ != "/map" && this->frame_name_ != "ma"
                                                                                                                    "p")
  {
    // look through all models in the world, search for body
    // name that matches frameName
    for (physics::Model_V::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
    {
      if (*iter)
        this->my_link_ = boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->frame_name_));
      if (this->my_link_)
        break;
    }

    // not found
    if (!this->my_link_)
    {
      ROS_INFO("gazebo_ros_bumper plugin: frameName: %s does not exist"
               " using world",
               this->frame_name_.c_str());
    }
  }

  if (this->local_link_ == NULL)
  {
    // look through all models in the world, search for body
    // name that matches local link name
    for (physics::Model_V::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
    {
      if (*iter)
        this->local_link_ = boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->local_name_));
      if (this->local_link_)
        break;
    }

    // not found
    if (!this->local_link_)
    {
      ROS_FATAL("gazebo_ros_bumper plugin: local link: %s does not exist", this->local_name_.c_str());
      return;
    }
  }

  // if local name is the same as the desired frame name, skip force/torque transform
  skip_local_transform_ = false;
  if (this->frame_name_ == this->local_name_)
    skip_local_transform_ = true;

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

  // Begin parsing
  ROS_INFO_STREAM_ONCE("bumper_topic_name_" << bumper_topic_name_);
  this->sensors = urdf::parseSensorsFromParam("robot_description", urdf::getSensorParser("tactile"));

  this->taxelPositions.clear();
  this->taxelNormals.clear();

  this->numOfSensors = 0;
  for (auto it = sensors.begin(), end = sensors.end(); it != end; ++it)
  {
    urdf::tactile::TactileSensorConstSharedPtr sensor = urdf::tactile::tactile_sensor_cast(it->second);
    if (!sensor)
      continue;  // some other sensor than tactile

    this->numOfTaxels.push_back(sensor->taxels_.size());
    this->taxelNormals.push_back(std::vector<gazebo::math::Vector3>(
      numOfTaxels[this->numOfSensors], gazebo::math::Vector3(0, 0, 0)));  // [i].resize(numOfTaxels[i]);
    this->taxelPositions.push_back(std::vector<gazebo::math::Vector3>(
      numOfTaxels[this->numOfSensors], gazebo::math::Vector3(0, 0, 0)));  // [i].resize(numOfTaxels[i]);
    sensor_msgs::ChannelFloat32 channel;
    channel.values.resize(numOfTaxels[this->numOfSensors]);
    this->tactile_state_msg_.sensors.push_back(channel);
    for (unsigned int j = 0; j < numOfTaxels[this->numOfSensors]; j++)
    {
      this->taxelPositions[this->numOfSensors][j] =
        gazebo::math::Vector3(sensor->taxels_[j]->origin.position.x, sensor->taxels_[j]->origin.position.y,
                              sensor->taxels_[j]->origin.position.z);

      // normal=rotation times zAxis
      urdf::Vector3 zAxis(0, 0, 1);
      urdf::Vector3 urdfTaxelNormal = (sensor->taxels_[j]->origin.rotation) * zAxis;
      this->taxelNormals[this->numOfSensors][j] =
        gazebo::math::Vector3(urdfTaxelNormal.x, urdfTaxelNormal.y, urdfTaxelNormal.z);
    }
    this->tactile_state_msg_.sensors[numOfSensors].name = "ffdistal";
    // "Sensor_" + std::to_string(numOfSensors); // >> numOfSensors;
    this->numOfSensors++;
  }


  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(std::string(this->bumper_topic_name_), 1);

  this->tactile_pub_ = this->rosnode_->advertise<tactile_msgs::TactileState>(std::string(this->tactile_topic_name_), 1);

  // Initialize
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosTactile::ContactQueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(boost::bind(&GazeboRosTactile::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
  
  // Initialize sensitivity of Sensor
  this->forceSensitivity = 0.1f;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTactile::OnContact()
{
  if (this->tactile_pub_.getNumSubscribers() <= 0 && contact_pub_.getNumSubscribers() <= 0)
    return;

  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  /// \TODO: need a time for each Contact in i-loop, they may differ
  this->tactile_state_msg_.header.frame_id = this->frame_name_;
#if GAZEBO_MAJOR_VERSION >= 7
  common::Time meastime = this->parentSensor->LastMeasurementTime();
#else
  common::Time meastime = this->parentSensor->GetLastMeasurementTime();
#endif
  this->tactile_state_msg_.header.stamp = ros::Time(meastime.sec, meastime.nsec);
  this->contact_state_msg_.header.frame_id = this->frame_name_;
  this->contact_state_msg_.header.stamp = ros::Time(meastime.sec, meastime.nsec);

  // get reference frame (body(link)) pose and subtract from it to get
  // relative force, torque, position and normal vectors
  math::Pose pose, frame_pose, local_pose;
  math::Quaternion rot, frame_rot, local_rot;
  math::Vector3 pos, frame_pos, local_pos;

  //float forceSensitivity = 0.1f;
  // Get local link orientation
  if (local_link_)
  {
    local_pose = local_link_->GetWorldPose();
    local_pos = local_pose.pos;
    local_rot = local_pose.rot;
  }
  // Get frame orientation if frame_id is given
  if (my_link_)
  {
    frame_pose = my_link_->GetWorldPose();  // -this->myBody->GetCoMPose();->GetDirtyPose();
    frame_pos = frame_pose.pos;
    frame_rot = frame_pose.rot;
  }
  else
  {
    // no specific frames specified, use identity pose, keeping
    // relative frame at inertial origin
    frame_pos = math::Vector3(0, 0, 0);
    frame_rot = math::Quaternion(1, 0, 0, 0);  // gazebo u,x,y,z == identity
    frame_pose = math::Pose(frame_pos, frame_rot);
  }

  // ROS_INFO_STREAM_THROTTLE(1.0, "frame_pos" << frame_pos.x << "frame_rot" <<
  // frame_rot.x);

  // set contact states size
  this->contact_state_msg_.states.clear();

  unsigned int contactsPacketSize = contacts.contact_size();
  // Deklarationen
  unsigned int contactGroupSize;
  // ROS_INFO_STREAM_THROTTLE(1.0, "contactGroupSize" << contactGroupSize);
  double normalForceScalar;
  double stdDev = 0.005;
  double distance;
  double critDist = 1.0;

  double p = 1.0;  // Multiplicator
  const double pi = 3.14159265359;
  double minForce = 0.0;
  double p_sum = 0.0;

  float finalProjectedForce;

  
  //for every timestep init the sensor to zero
   for (unsigned int m = 0; m < this->numOfSensors; m++)
      {                                                          // Loop over Sensors
        for (unsigned int k = 0; k < this->numOfTaxels[m]; k++)  // Loop over taxels
        {
          this->tactile_state_msg_.sensors[m].values[k] = 0.0f;
        }
      }
  
  for (unsigned int i = 0; i < contactsPacketSize; ++i)
  {
    // Create a ContactState
    gazebo_msgs::ContactState state;

    gazebo::msgs::Contact contact = contacts.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i:(" << i << "/" << contactsPacketSize
           << ")     my geom:" << state.collision1_name
           << "   other geom:" << state.collision2_name << "         time:"
           << ros::Time(contact.time().sec(), contact.time().nsec())
           << std::endl;
    state.info = stream.str();

    // is body 1 the collision body attached to the sensor plugin ?
    bool switch_body = false;
    if (state.collision2_name == collision_name_)
      switch_body = true;

    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();

    // sum up all wrenches for each DOF
    geometry_msgs::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j)
    {
      
      // Get force, torque. They are in local frame already.
      // forward transform them to world and then
      // and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)

      ROS_DEBUG_STREAM("body1 name: " << contact.wrench(j).body_1_name()
                                      << ", body2 name: " << contact.wrench(j).body_2_name());

      // select the correct body
      gazebo::msgs::Wrench source_wrench;
      if (switch_body)
      {
        ROS_DEBUG("using body2");
        source_wrench = contact.wrench(j).body_2_wrench();
      }
      else
        source_wrench = contact.wrench(j).body_1_wrench();

      math::Vector3 force;
      math::Vector3 torque;
      // apply transform to force/torque only if needed
      if (skip_local_transform_)
      {
        force = math::Vector3(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z());
        torque = math::Vector3(source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z());
      }
      else
      {
        force = frame_rot.RotateVectorReverse(local_rot.RotateVector(
          math::Vector3(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z())));
        torque = frame_rot.RotateVectorReverse(local_rot.RotateVector(
          math::Vector3(source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z())));
      }

      // set wrenches
      geometry_msgs::Wrench wrench;
      wrench.force.x = force.x;
      wrench.force.y = force.y;
      wrench.force.z = force.z;
      wrench.torque.x = torque.x;
      wrench.torque.y = torque.y;
      wrench.torque.z = torque.z;
      state.wrenches.push_back(wrench);

      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // ///////////////////////BEGIN OF FORCE TRANSFORMTION
      // transform contact positions into relative frame
      // set contact positions
      gazebo::math::Vector3 position = frame_rot.RotateVectorReverse(
        math::Vector3(contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) - frame_pos);
      geometry_msgs::Vector3 contact_position;
      contact_position.x = position.x;
      contact_position.y = position.y;
      contact_position.z = position.z;
      state.contact_positions.push_back(contact_position);

      // rotate normal from world into user specified frame.
      // frame_rot is identity if world is used.
      math::Vector3 normal = frame_rot.RotateVectorReverse(
        math::Vector3(contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
      // set contact normals
      geometry_msgs::Vector3 contact_normal;
      contact_normal.x = normal.x;
      contact_normal.y = normal.y;
      contact_normal.z = normal.z;
      state.contact_normals.push_back(contact_normal);

      // set contact depth, interpenetration
      state.depths.push_back(contact.depth(j));

      // //////////////////////////////////END OF FORCE TRANSFORMATION
      
      normalForceScalar =
                (contact_normal.x * force.x +
                 contact_normal.y * force.y +
                 contact_normal.z * force.z); //* (-1.0);

      for (unsigned int m = 0; m < this->numOfSensors; m++)
      {                                                          // Loop over Sensors
        for (unsigned int k = 0; k < this->numOfTaxels[m]; k++)  // Loop over taxels
        {
          // sensor_msgs::ChannelFloat32 &tSensor =
          // this->tactile_state_msg_.sensors[m];

          // calc distance between force-ap and taxelcenter
          distance =
            sqrt(pow((position.x - taxelPositions[m][k].x), 2) + pow((position.y - taxelPositions[m][k].y), 2) +
                 pow((position.z - taxelPositions[m][k].z), 2));

          if (distance > critDist)
          {
            finalProjectedForce = 0.0f;  // bzw continue;
                                         // nächster Durchlauf
          }

          else
          {

            if (normalForceScalar > 0)
            {
              // Normalverteilung erzeugen
              p = exp(-(distance * distance / (2 * stdDev * stdDev)))  /
                  sqrt(2 * pi * stdDev * stdDev);
              finalProjectedForce = p * normalForceScalar;
              p_sum += p;
            }
            else
            {
              finalProjectedForce = 0.0f;
            }
          }

          this->tactile_state_msg_.sensors[m].values[k] += finalProjectedForce;
        }  // END FOR Taxels
      }    // END FOR Sensors
    }      // END FOR contactGroupSize

    state.total_wrench = total_wrench;
    this->contact_state_msg_.states.push_back(state);
  }  // END FOR contactsPacketSize
  
  //devide by sum of weights and check if sensed force is bigger minForce
  for (unsigned int e = 0; e < this->numOfSensors; e++)
  {
    for (unsigned int f = 0; f < this->numOfTaxels[e]; f++)
    {
      if (p_sum > 0)
      {
        this->tactile_state_msg_.sensors[e].values[f] /= p_sum;
      
        if (this->tactile_state_msg_.sensors[e].values[f] < minForce)
        {
          this->tactile_state_msg_.sensors[e].values[f] = 0.0f;
        }
      } 
      else
      {
        this->tactile_state_msg_.sensors[e].values[f] = 0.0;
      }
    }
  }
  this->contact_pub_.publish(this->contact_state_msg_);
  this->tactile_pub_.publish(this->tactile_state_msg_);


}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosTactile::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo
