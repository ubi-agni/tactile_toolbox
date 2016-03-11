/**
 * @file   tactile_state_publisher.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 17 2016
 *
 * @brief  tactile state publisher
 */

#include "tactile_state_publisher/tactile_state_publisher.hpp"
#include <urdf_parser/urdf_parser.h>
#include <urdf_sensor/tactile.h>
#include <boost/thread/locks.hpp>
#include <map>
#include <vector>
#include <string>
#include <algorithm>


TactileStatePublisher::TactileStatePublisher():
  publish_rate_(DEFAULT_PUBLISH_RATE)
{
  // configure the tsp from URDF and param server
  config();

  // init publisher/subscribers
  init();
}

void TactileStatePublisher::config()
{
  // read sensors from URDF
  if (urdf_model_.initParam("/robot_description"))
  {
    createSensorDataMap();
  }
  
  // read parameters
  ros::NodeHandle nh_priv("~");    
  double rate;
  if(nh_priv.getParam("publish_rate", rate))
  {
    publish_rate_= ros::Rate(rate);
  }
    
  XmlRpc::XmlRpcValue source_list_raw;
  nh_priv.getParam("source_list", source_list_raw);
  // parse the parameter list
  if(source_list_raw.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // iterate on all the elements
    for (int32_t index = 0; index < source_list_raw.size(); ++index)
    {
      // check the source is well formatted:
      if (source_list_raw[index].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string source_name = static_cast<std::string> (source_list_raw[index]);
        source_list_.push_back(source_name);
      }
    }
  }
  else
  {
    ROS_ERROR("source list is not an array but type %d", source_list_raw.getType());
  }
}

void TactileStatePublisher::createSensorDataMap()
{
  // prepare the recurrent tactile message
  std::map<std::string, urdf::SensorSharedPtr>::iterator it;

  // loop over all the sensor found in the URDF
  for (it = urdf_model_.sensors_.begin(); it != urdf_model_.sensors_.end(); it++)
  {
    boost::shared_ptr<urdf::Tactile> tactile_sensor_ptr = boost::static_pointer_cast<urdf::Tactile>(it->second->sensor);
    if (!tactile_sensor_ptr) continue;  // some other sensor than tactile

    sensor_msgs::ChannelFloat32 sensor_data;
    sensor_data.name = it->second->name;

    if (tactile_sensor_ptr->_array)
    {
      sensor_data.values.resize(tactile_sensor_ptr->_array->rows * tactile_sensor_ptr->_array->cols);
    }
    else if (tactile_sensor_ptr->_taxels.size())
    {
      // find highest taxel index used
      unsigned int maxIdx = 0;
      for (std::vector<urdf::TactileTaxelSharedPtr>::const_iterator it = tactile_sensor_ptr->_taxels.begin(),
           end = tactile_sensor_ptr->_taxels.end(); it != end; ++it)
      {
        if (maxIdx < (*it)->idx)
          maxIdx = (*it)->idx;
      }
      sensor_data.values.resize(maxIdx+1);
    }

    tactile_msg_.sensors.push_back(sensor_data);
    // add the index to sensor_data_map
    sensor_data_map_[sensor_data.name] = tactile_msg_.sensors.size() - 1;
  }
}

void TactileStatePublisher::init()
{
  // initialize publisher
  tactile_pub_ = nh_.advertise<tactile_msgs::TactileState>("tactile_states", 5);

  // intialize subscribers from source list if any
  for (size_t i = 0; i < source_list_.size(); ++i)
  {
    boost::shared_ptr<ros::Subscriber> tactile_subscriber(new ros::Subscriber(nh_.subscribe(source_list_[i], 1,
                                                                             &TactileStatePublisher::tactile_state_cb, this)));
    tactile_subs_.push_back(tactile_subscriber);
  }
}

void TactileStatePublisher::tactile_state_cb(const tactile_msgs::TactileStateConstPtr& msg)
{
  // loop on the sensors
  for (size_t i = 0; i < msg->sensors.size(); ++i)
  {
    const std::string &name = msg->sensors[i].name;
    std::map<std::string, size_t>::iterator it = sensor_data_map_.find(name);
    if (it == sensor_data_map_.end()) continue;

    // store new data in the tactile_msg
    const sensor_msgs::ChannelFloat32::_values_type &src = msg->sensors[i].values;
    sensor_msgs::ChannelFloat32::_values_type &dst = tactile_msg_.sensors[it->second].values;
    if (dst.size() <= src.size())
    {
      boost::unique_lock<boost::shared_mutex> lock(mutex_);
      std::copy(src.begin(), src.begin() + dst.size(), dst.begin());
    }
  }
}

void TactileStatePublisher::publish()
{
  {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
    tactile_msg_.header.stamp = ros::Time::now();
    tactile_pub_.publish(tactile_msg_);
  }
  publish_rate_.sleep();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_state_publisher");

  TactileStatePublisher tsp;

  while (ros::ok())
  {
    tsp.publish();
    ros::spinOnce();
  }

  return 0;
}
