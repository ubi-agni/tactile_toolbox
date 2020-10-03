#include "pcl_collector.h"
#include "conversions.h"

#include <boost/thread/locks.hpp>
#include <urdf/sensor.h>
#include <urdf/model.h>

using namespace tactile_msgs;

namespace tactile {

ros::Duration PCLCollector::timeout_;

PCLCollector::PCLCollector(const std::string &target_frame, const double threshold)
   : tf_buffer_()
   , tf_listener_(tf_buffer_)
   , threshold_(threshold)
{
	initFromRobotDescription();
	setTargetFrame(target_frame);
}

static
void loadRobotDescription(std::string &xml_string, const std::string &param)
{
	ros::NodeHandle nh;

	// gets the location of the robot description on the parameter server
	std::string full_param;
	if (!nh.searchParam(param, full_param)){
		throw std::runtime_error("Could not find parameter " + param + " on parameter server");
	}

	// read the robot description from the parameter server
	if (!nh.getParam(full_param, xml_string)){
		throw std::runtime_error("Could not read parameter " + param + " on parameter server");
	}
}

void PCLCollector::initFromRobotDescription(const std::string &param)
{
	try {
		std::string xml_string;
		loadRobotDescription(xml_string, param);

		// fetch robot_root_frame
		urdf::Model model;
		if (model.initString(xml_string))
			robot_root_frame_ = model.root_link_->name;
		else
			ROS_WARN_STREAM("failed to parse " << param);

		// fetch sensor descriptions
		sensors_ = parseSensors(xml_string, urdf::getSensorParser("tactile"));
	} catch (const std::exception &e) {
		ROS_WARN_STREAM("failed to parse robot description:" << e.what());
	}
}

// template specializations for different types
template <>
void PCLCollector::process<TactileContact>(const TactileContactConstPtr &msg)
{
	ContactPoint contact;

	const geometry_msgs::Vector3 &f = msg->wrench.force;
	contact.intensity = Eigen::Vector3d(f.x, f.y, f.z).norm();

	if (contact.intensity < threshold_)
		return;

	contact.x = msg->position.x;
	contact.y = msg->position.y;
	contact.z = msg->position.z;

	contact.normal_x = msg->normal.x;
	contact.normal_y = msg->normal.y;
	contact.normal_z = msg->normal.z;

	try {
		boost::unique_lock<boost::mutex> lock(*this);
		addPoint(contact, tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, timeout_));
	} catch (const tf2::TransformException &ex) {
		ROS_WARN("Failure %s", ex.what());
	}
}

void PCLCollector::addPoint(ContactPoint &point, const geometry_msgs::TransformStamped &transform)
{
	// transform the point using transform
	tf2::doTransform(point, point, transform);
	// and push it into the cloud
	pcl_.push_back(point);
}

void PCLCollector::setTargetFrame(const std::string &frame)
{
	// use robot's root frame as fallback if frame is empty
	target_frame_ = frame.empty() ? robot_root_frame_ : frame;
	// update tf filter
	if (tf_filter_) tf_filter_->setTargetFrame(target_frame_);
}

void PCLCollector::clear()
{
	pcl_.clear();
}

const pcl::PointCloud<PCLCollector::ContactPoint> &PCLCollector::pcl()
{
	return pcl_;
}

} // namespace tactile
