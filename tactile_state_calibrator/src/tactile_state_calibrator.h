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
#include <tactile_filters/Calibration.h>

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
  
  enum calib_type
  {
    RAW=0,
    PWL=1
  };

private:
  /**
   * initiliaze subscribers and publisher
   */
  void init(const std::string &calib_filename);

  /**
   * fill calibs_ vector from a map of index
   */
  bool fill_calibs(std::map<int, tactile::Calibration*> &map);
  /**
   * extract index of taxels a calib applies to
   */
  bool extract_range(const YAML::Node &node, std::map<int, tactile::Calibration*> &map, tactile::Calibration* p);

  /**
   * generic tactile callback
   */
  void tactile_state_cb(const tactile_msgs::TactileStateConstPtr& msg);
  /**
   * wrappwr to call calibration map operator with different calib
   */ 
  float map(float val, tactile::Calibration *c);
  
  std::vector<tactile::Calibration *> calibs_;
  tactile::Calibration * single_calib_;
};
