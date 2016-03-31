
#pragma once

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tactile_msgs/TactileContacts.h>

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/spinner.h>

#include <boost/regex.hpp>

class TactileToPointCloud {

public:

	typedef tactile_msgs::TactileContact TactileContactMSG;
	typedef tactile_msgs::TactileContacts TactileContactsMSG;
	typedef tactile_msgs::TactileContactsPtr TactileContactsMSGPtr;
	typedef sensor_msgs::PointCloud2 PCMSG;

	TactileToPointCloud();
	virtual ~TactileToPointCloud();

	void init(std::string const &tactile_topic = "/tactile_contact_states",
						std::string const &pointcloud_topic = "/tactile_pcl",
						std::string const &base_tf = "/world");

	void setNameFilter(boost::regex const &expr, bool filter_negative = false);

	void convert(TactileContactsMSGPtr const &in_msg, PCMSG &out_msg);

	void publish(const PCMSG &out_msg);

	void start();

	void stop();

protected:

	void callback(TactileContactsMSGPtr const &data);

	std::string m_base_tf;

	ros::NodeHandle m_node;
	ros::CallbackQueue m_queue;
	ros::Subscriber m_subscriber;
	ros::Publisher m_publisher;
	ros::AsyncSpinner m_spinner;

	tf::TransformListener m_tf_listener;

	boost::regex m_expr;
	bool m_filter_negative;
};
