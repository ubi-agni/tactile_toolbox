/**
 * @file   tactile_state_publisher.h
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 09 2016
 *
 * @brief  tactile state publisher
 */

#pragma once
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <tactile_msgs/TactileState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <urdf_parser/sensor_parser.h>

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
  bool valid() const;

private:

  ros::NodeHandle nh_;
  ros::Rate publish_rate_;
  ros::Publisher tactile_pub_;
  std::vector<std::string> source_list_; // to store the source list
  std::vector<boost::shared_ptr<ros::Subscriber> > tactile_subs_;
  tactile_msgs::TactileState tactile_msg_;

protected:
  mutable boost::shared_mutex mutex_; // multiple reads / one write mutex

  std::map<std::string, size_t> sensor_data_map_;

  /**
   * initiliaze subscribers and publisher
   */
  void init();
  void createSensorDataMap(const urdf::SensorMap &sensors);
  void config();

  /**
   * generic tactile callback
   */
  void tactile_state_cb(const tactile_msgs::TactileStateConstPtr& msg);

};
