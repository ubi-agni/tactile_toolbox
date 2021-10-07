#pragma once

#include <tactile_msgs/TactileState.h>
#include <tactile_msgs/TactileContacts.h>
#include <urdf/sensor.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>

#include <boost/thread/mutex.hpp>

namespace tactile {

/** The PCL collector accumulates tactile contact points into a sensor_msgs::PointCloud2.
 *  It can receive different input msgs (TactileState, TactileContact, TactileContacts).
 *  These are first filtered by a ThrottleFilter */
class PCLCollector : public boost::mutex
{
public:
	typedef pcl::PointXYZINormal ContactPoint;

	PCLCollector(const std::string &target_frame = "", const double threshold = 0.0);
	void initFromRobotDescription(const std::string &param = "robot_description");

	template <typename M, typename F>
	void setSource(F &f, unsigned int queue_size)
	{
		// connect F to a tf filter that signals to process()
		tf2_ros::MessageFilter<M> *tf_filter =
		    new tf2_ros::MessageFilter<M>(f, tf_buffer_, target_frame_, queue_size, nullptr);
		tf_filter_.reset(tf_filter);
		tf_filter->registerCallback(&PCLCollector::process<M>, this);
	}

	void setTargetFrame(const std::string &frame);
	const std::string &targetFrame() const { return target_frame_; }

	void clear();
	const pcl::PointCloud<ContactPoint> &pcl();

protected:
	template <typename M>
	void process(const typename message_filters::Subscriber<M>::MConstPtr &msg);
	void addPoint(ContactPoint &point, const geometry_msgs::TransformStamped &transform);

protected:
	std::string robot_root_frame_;
	urdf::SensorMap sensors_;  //< tactile sensors

	std::string target_frame_;  //< target frame, the PCL should be expressed in
	pcl::PointCloud<ContactPoint> pcl_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	// hold back messages until transform becomes available
	boost::shared_ptr<tf2_ros::MessageFilterBase> tf_filter_;

	static ros::Duration timeout_;
	double threshold_;
};

}  // namespace tactile
