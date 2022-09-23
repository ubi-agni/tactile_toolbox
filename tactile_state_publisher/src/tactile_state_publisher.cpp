/**
 * @file   tactile_state_publisher.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 17 2016
 *
 * @brief  tactile state publisher
 */

#include "tactile_state_publisher.h"
#include <urdf_tactile/sensor.h>

#include <boost/thread/locks.hpp>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

using namespace urdf::tactile;

TactileStatePublisher::TactileStatePublisher() : publish_rate_(DEFAULT_PUBLISH_RATE)
{
	// configure the tsp from URDF and param server
	config();

	// init publisher/subscribers
	init();
}

void TactileStatePublisher::config()
{
	/* avoid throwing of two exceptions in a row:
	   http://stackoverflow.com/questions/17838830/throwing-an-exception-while-handling-an-exception
	   Due to a bug in pluginlib, the unloading of the lib might throw on destruction of SensorParserMap.
	*/
	try {
		createSensorDataMap(parseSensorsFromParam("robot_description"));
	} catch (const std::exception &e) {
		ROS_ERROR_STREAM(e.what());
		return;
	}

	// read parameters
	ros::NodeHandle nh_priv("~");
	double rate;
	if (nh_priv.getParam("publish_rate", rate)) {
		publish_rate_ = ros::Rate(rate);
	}

	XmlRpc::XmlRpcValue source_list_raw;
	nh_priv.getParam("source_list", source_list_raw);
	// parse the parameter list
	if (source_list_raw.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		// iterate on all the elements
		for (int32_t index = 0; index < source_list_raw.size(); ++index) {  // NOLINT(modernize-loop-convert)
			// check the source is well formatted:
			if (source_list_raw[index].getType() == XmlRpc::XmlRpcValue::TypeString) {
				std::string source_name = static_cast<std::string>(source_list_raw[index]);
				source_list_.push_back(source_name);
			}
		}
	} else {
		ROS_ERROR("source list is not an array but type %d", source_list_raw.getType());
	}
}

void TactileStatePublisher::createSensorDataMap(const SensorMap &sensors)
{
	// loop over all the sensor found in the URDF
	for (const auto &item : sensors) {
		const TactileSensorSharedPtr &tactile = item.second;
		int sensor_idx = -1;

		// find if the channel exists in the map
		std::map<std::string, size_t>::iterator sensor_it = sensor_data_map_.find(tactile->channel_);
		if (sensor_it == sensor_data_map_.end()) {
			// if not, create a sensor_msgs for this channel.
			sensor_msgs::ChannelFloat32 sensor_data;
			sensor_data.name = tactile->channel_;  // size will be updated later
			tactile_msg_.sensors.push_back(sensor_data);
			sensor_idx = tactile_msg_.sensors.size() - 1;
			// add the index to sensor_data_map
			sensor_data_map_[sensor_data.name] = sensor_idx;
		} else {
			sensor_idx = sensor_it->second;
		}

		if (tactile->array_) {
			// TODO: Guillaume handle the fact that only one array can exist per tactile channel
			tactile_msg_.sensors[sensor_idx].values.resize(tactile->array_->rows * tactile->array_->cols);
		} else if (!tactile->taxels_.empty()) {
			// find highest taxel index used
			unsigned int max_idx = 0;
			for (auto &taxel : tactile->taxels_) {
				if (max_idx < taxel->idx)
					max_idx = taxel->idx;
			}
			// resize only if this channel was smaller
			if (tactile_msg_.sensors[sensor_idx].values.size() < max_idx + 1)
				tactile_msg_.sensors[sensor_idx].values.resize(max_idx + 1);
		}
	}
}

void TactileStatePublisher::init()
{
	// initialize publisher
	tactile_pub_ = nh_.advertise<tactile_msgs::TactileState>("tactile_states", 5);

	// initialize subscribers from source list if any
	for (auto &topic : source_list_) {
		std::shared_ptr<ros::Subscriber> tactile_subscriber(
		    new ros::Subscriber(nh_.subscribe(topic, 1, &TactileStatePublisher::tactile_state_cb, this)));
		tactile_subs_.push_back(tactile_subscriber);
	}
}

void TactileStatePublisher::tactile_state_cb(const tactile_msgs::TactileStateConstPtr &msg)
{
	// loop on the sensors
	for (const auto &sensor : msg->sensors) {
		const std::string &name = sensor.name;
		std::map<std::string, size_t>::iterator it = sensor_data_map_.find(name);
		if (it == sensor_data_map_.end())
			continue;

		// store new data in the tactile_msg
		const sensor_msgs::ChannelFloat32::_values_type &src = sensor.values;
		sensor_msgs::ChannelFloat32::_values_type &dst = tactile_msg_.sensors[it->second].values;
		if (dst.size() <= src.size()) {
			boost::unique_lock<boost::shared_mutex> lock(msg_mutex_);
			std::copy(src.begin(), src.begin() + dst.size(), dst.begin());
		}
	}
}

void TactileStatePublisher::publish()
{
	{
		boost::shared_lock<boost::shared_mutex> lock(msg_mutex_);
		tactile_msg_.header.stamp = ros::Time::now();
		tactile_pub_.publish(tactile_msg_);
	}
	publish_rate_.sleep();
}

bool TactileStatePublisher::valid() const
{
	return !tactile_subs_.empty() && !sensor_data_map_.empty();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_PACKAGE_NAME);

	try {
		TactileStatePublisher tsp;
		if (!tsp.valid())
			return EINVAL;

		while (ros::ok()) {
			tsp.publish();
			ros::spinOnce();
		}
	} catch (const std::exception &e) {
		ROS_ERROR_STREAM(e.what());
		return EFAULT;
	}
	return 0;
}
