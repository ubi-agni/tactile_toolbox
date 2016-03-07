/**
 * @file   tactile_state_publisher.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 17 2016
 *
 * @brief  tactile state publisher
 */

#include "tactile_state_publisher/tactile_state_publisher.hpp"
#include <urdf_parser/urdf_parser.h>
#include <boost/thread/locks.hpp>
#include <string>


using namespace std;


TactileStatePublisher::TactileStatePublisher():
  publish_rate_(DEFAULT_PUBLISH_RATE)
{
  // configure the tsp
  //READ FROM URDF
  config();
    
  // init publisher/subscribers
  init();
  
}

bool TactileStatePublisher::sensorParser(std::string param)
{
  /*std::string xml_string;
  // gets the location of the robot description on the parameter server
  std::string full_param;
  if (!nh_.searchParam(param, full_param)){
    ROS_ERROR("Could not find parameter %s on parameter server", param.c_str());
    return false;
  }

  // read the robot description from the parameter server
  if (!nh_.getParam(full_param, xml_string)){
    ROS_ERROR("Could not read parameter %s on parameter server", full_param.c_str());
    return false;
  }
  urdf_model.reset(urdf::Model::parseURDF(xml_string));
  */
  ROS_WARN_STREAM(param);
  urdf_model.initParam(param) ;
  
   
  /* 
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if (xml_doc.Error())
  {
    //ROS_ERROR(xml_doc.ErrorDesc());
    xml_doc.ClearError();
    return false;
  }

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    ROS_ERROR("Could not find the 'robot' element in the xml file");
    return false;
  }
  
   // Get all sensor elements
  for (TiXmlElement* sensor_xml = robot_xml->FirstChildElement("sensor"); sensor_xml; sensor_xml = sensor_xml->NextSiblingElement("sensor"))
  {
    SensorSharedPtr sensor;
    sensor.reset(new urdf::Sensor);

    try 
    {
      urdf::parseSensor(*sensor, sensor_xml);
      if (getSensor(sensor->name))
      {
        ROS_ERROR("sensor '%s' is not unique.", sensor->name.c_str());
        sensor.reset();
      }
      else
      {
        sensor_map_.insert(make_pair(sensor->name, sensor));
        ROS_DEBUG("tsp successfully added a new sensor '%s'", sensor->name.c_str());
      }
    }
    * */
    //catch (urdf::ParseError &/*e*/) {
    //  ROS_ERROR("sensor xml is not initialized correctly");
    //  sensor.reset();
   // }
  //}
  
  
  
  return true;
}

void TactileStatePublisher::config()
{
  if (sensorParser("/robot_description"))
  {
    createSensorDataMap();
  }
}

void TactileStatePublisher::createSensorDataMap()
{
  // prepare the recurrent tactile message
  
  std::map<std::string, urdf::SensorSharedPtr>::iterator it;
   
  // loop over all the sensor found in the URDF
  for (it = urdf_model.sensors_.begin(); it != urdf_model.sensors_.end(); it++)
  {
    sensor_msgs::ChannelFloat32 sensor_data;
    sensor_data.name = it->second->name;
    boost::shared_ptr<urdf::Tactile> tactile_sensor_ptr = boost::static_pointer_cast<urdf::Tactile>(it->second->sensor);
    unsigned int nb_taxels = tactile_sensor_ptr->tactile_taxels.size();
    unsigned int nb_arrays = tactile_sensor_ptr->tactile_arrays.size();
    // check if only taxels or only arrays are present
    if (nb_taxels * nb_arrays == 0 and nb_taxels != nb_arrays)
    {
      // initialize the data field with the correct sensor data size
      if (nb_taxels)
      {
        
        sensor_data.values.resize(tactile_sensor_ptr->getTaxelDataSize());
      }
      else
      {
        if (nb_arrays == 1)
        {
          sensor_data.values.resize(tactile_sensor_ptr->getArrayDataSize());
        }
        else
        {
          ROS_ERROR("tsp: multi arrays is not yet supported");
        }
      }
      tactile_msg_.sensors.push_back(sensor_data);
      // add to sensor_data_map
      sensor_data_map_[sensor_data.name] = &(tactile_msg_.sensors.back());
    }
    else
    {
      ROS_ERROR("tsp: mix of taxels and arrays is not yet supported");
    }
  }
}

void TactileStatePublisher::init()
{
  // init sensor_list
  // FROM URDF
  //source_list_.push_back("/rh/tactile_tip");
  //source_list_.push_back("/rh/tactile_mid");
  //source_list_.push_back("/rh/tactile_prox");
  source_list_.push_back("/rh/tactile_mid");
  source_list_.push_back("/rh/tactile_prox");
  
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
  boost::unique_lock<boost::shared_mutex> lock(mutex_);
  std::map<std::string, sensor_msgs::ChannelFloat32*>::iterator it;
  
  // loop on the sensors
  for (size_t i = 0; i < msg->sensors.size(); ++i)
  {
    std::string name = msg->sensors[i].name;
    it = sensor_data_map_.find(name);
    if(it != sensor_data_map_.end())
    {
      //store new data in the tactile_msg
      ROS_ASSERT(it->second->values.size() == msg->sensors[i].values.size());
      it->second->values = msg->sensors[i].values;
    }
    //else do nothing
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
