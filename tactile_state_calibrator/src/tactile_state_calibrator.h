/**
 * @file   tactile_state_calibrator.h
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Jan 17 2020
 *
 * @brief  tactile state calibrator
 */

#pragma once
#include <ros/ros.h>

#include <memory>
#include <tactile_msgs/TactileState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tactile_filters/PieceWiseLinearCalib.h>

#include <string>
#include <vector>

class TactileStateCalibrator
{
  ros::NodeHandle nh_;
  ros::Subscriber tactile_sub_; //! source subscriber
  ros::Publisher tactile_pub_; //! publisher

public:
  TactileStateCalibrator(const std::string calib_filename);
  ~TactileStateCalibrator();

private:
  /**
   * initiliaze subscribers and publisher
   */
  void init(const std::string &calib_filename);

  /**
   * generic tactile callback
   */
  void tactile_state_cb(const tactile_msgs::TactileStateConstPtr& msg);
  
  tactile::PieceWiseLinearCalib *calib_;
};
