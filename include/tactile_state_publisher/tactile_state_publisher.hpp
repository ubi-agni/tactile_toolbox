/**
 * @file   tactile_state_publisher.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 09 2016
 *
 * @brief  tactile state publisher
 */


#ifndef _TACTILE_STATE_PUBLISHER_HPP_
#define _TACTILE_STATE_PUBLISHER_HPP_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <tactile_msgs/TactileState.h> 
#include <sensor_msgs/ChannelFloat32.h> 

#include <urdf/model.h>
#include <urdf_sensor/sensor.h>

#include <string>
#include <vector>

#define DEFAULT_PUBLISH_RATE  100.0

class TactileStatePublisher
{
public:
  TactileStatePublisher();

  /**
   * publish the current tactile values
   */
  void publish();

private:

  ros::NodeHandle nh_;
  urdf::Sensor sensor_;
  ros::Rate publish_rate_;
  ros::Publisher tactile_pub_;
  std::vector<std::string> source_list_; // to store the source list
  std::vector<boost::shared_ptr<ros::Subscriber> > tactile_subs_;
  tactile_msgs::TactileState tactile_msg_;

protected:
  mutable boost::shared_mutex mutex_; // multiple reads / one write mutex
  // typedef shared pointers
  //typedef boost::shared_ptr<urdf::Sensor> SensorSharedPtr;
  urdf::Model urdf_model;

  std::map<std::string, sensor_msgs::ChannelFloat32*> sensor_data_map_;
 // std::map<std::string, SensorSharedPtr> sensor_map_;
  
  /**
   * initiliaze subscribers and publisher
   */
  void init();
  void createSensorDataMap();
  void config();

  /**
   * retrieve sensor from sensor_map
   */
 /* SensorSharedPtr getSensor(const std::string& name) const
  {
    SensorSharedPtr ptr;
    if (this->sensor_map_.find(name) == this->sensor_map_.end())
      ptr.reset();
    else
      ptr = this->sensor_map_.find(name)->second;
    return ptr;
  };
  */
  /**
   * generic tactile callback
   */
  void tactile_state_cb(const tactile_msgs::TactileStateConstPtr& msg);

};


#endif //_TACTILE_STATE_PUBLISHER_HPP_
