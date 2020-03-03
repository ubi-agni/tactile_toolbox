/**
 * @file   tactile_state_calibrator.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Jan 17 2020
 *
 * @brief  tactile state calibrator
 */

#include "tactile_state_calibrator.h"
#include <vector>
#include <string>
#include <algorithm>


using namespace tactile;

TactileStateCalibrator::TactileStateCalibrator(const std::string calib_filename)
{
  // init publisher/subscribers
  init(calib_filename);
}

void TactileStateCalibrator::init(const std::string &calib_filename)
{
  if (!calib_filename.empty()) {
    calib_ = new PieceWiseLinearCalib(PieceWiseLinearCalib::load(calib_filename));
    if (calib_ == NULL)
      throw ("unable to create PieceWiseLinearCalib");
  }
  else
  {
    throw ("calibration file cannot be empty");
    return;
  }

  // initialize publisher
  tactile_pub_ = nh_.advertise<tactile_msgs::TactileState>("out_tactile_states", 5);

  // initialize subscriber
  tactile_sub_ = nh_.subscribe("in_tactile_states", 5, &TactileStateCalibrator::tactile_state_cb, this);
}

void TactileStateCalibrator::tactile_state_cb(const tactile_msgs::TactileStateConstPtr& msg)
{
  tactile_msgs::TactileState out_msg;
  out_msg = *msg;
  // TODO: Guillaume, use a different calib file for each type of sensor (maybe regex on the name)
  for (size_t i = 0; i < msg->sensors.size(); ++i)
  {
    std::transform(msg->sensors[i].values.begin(), msg->sensors[i].values.end(), out_msg.sensors[i].values.begin(),
                   std::bind(&PieceWiseLinearCalib::map, calib_, std::placeholders::_1));
  }
  tactile_pub_.publish(out_msg);
}

TactileStateCalibrator::~TactileStateCalibrator()
{
  // unregister
  tactile_sub_.shutdown();
  // cleanup
  if (calib_ != NULL)
    delete calib_;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_state_calibrator");
   // read parameters
  ros::NodeHandle nh_priv("~");
  std::string calib_filename = "";
  if(!nh_priv.getParam("calib", calib_filename))
  {
    ROS_ERROR_STREAM("No calibration file provided");
    return EFAULT;
  }

  try {
    TactileStateCalibrator tsc(calib_filename);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
    return EFAULT;
  }
  return 0;
}
