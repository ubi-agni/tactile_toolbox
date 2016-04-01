#include "pcl_collector.h"
#include "conversions.h"

#include <boost/thread/locks.hpp>

using namespace tactile_msgs;

namespace tactile {

ros::Duration PCLCollector::timeout_;

PCLCollector::PCLCollector(const std::string &target_frame)
   : target_frame_(target_frame)
   , tf_buffer_()
   , tf_listener_(tf_buffer_)
{
}

template <typename M>
void PCLCollector::setSource(message_filters::Subscriber<M> &sub)
{
	tf2_ros::MessageFilter<M> *f = new tf2_ros::MessageFilter<M>(sub, tf_buffer_, target_frame_, 10, NULL);
	tf_filter_.reset(f);
	f->registerCallback(&PCLCollector::process<M>, this);
}

// instantiate templates
template void PCLCollector::setSource<TactileContact>(message_filters::Subscriber<TactileContact>&);

// template specializations for different types
template <>
void PCLCollector::process<TactileContact>(const TactileContactConstPtr &msg)
{
	ContactPoint contact;
	contact.x = msg->position.x;
	contact.y = msg->position.y;
	contact.z = msg->position.z;

	contact.normal_x = msg->normal.x;
	contact.normal_y = msg->normal.y;
	contact.normal_z = msg->normal.z;

	const geometry_msgs::Vector3 &f = msg->wrench.force;
	contact.intensity = Eigen::Vector3d(f.x, f.y, f.z).norm();

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
	if (frame.empty()) {
		//TODO init pcl_ structure: load robot_description (for target_frame_ and raw sensor info)
	}
	target_frame_ = frame;
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
