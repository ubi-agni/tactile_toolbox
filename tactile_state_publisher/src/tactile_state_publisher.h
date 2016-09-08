/**
 * @file   tactile_state_publisher.h
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 09 2016
 *
 * @brief  tactile state publisher
 */

#pragma once
#include <ros/ros.h>

#include <memory>
#include <boost/thread/shared_mutex.hpp>
#include <tactile_msgs/TactileState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <urdf_parser/sensor_parser.h>

#include <string>
#include <vector>

#define DEFAULT_PUBLISH_RATE  100.0


class TactileStatePublisher
{
  ros::NodeHandle nh_;
  std::vector<std::string> source_list_; //! names of source topics
  std::vector<std::shared_ptr<ros::Subscriber> > tactile_subs_; //! source subscribers

  ros::Publisher tactile_pub_; //! publisher
  ros::Rate publish_rate_; //! publishing rate

  /// unique output msg
  tactile_msgs::TactileState tactile_msg_;
  /// mutex on tactile_msgs_ allowing multiple reads, single write
  mutable boost::shared_mutex msg_mutex_;
  /// mapping channel names to indices into tactile_msgs_.sensors
  std::map<std::string, size_t> sensor_data_map_;

public:
  TactileStatePublisher();

  /// publish the current tactile values
  void publish();

  bool valid() const;

private:
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
