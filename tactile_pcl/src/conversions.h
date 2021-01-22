#pragma once

#include <ros/message_traits.h>
namespace ros {
namespace message_traits {

// template specializations to grant access to frame_id and stamp
template <typename T>
struct FrameId<typename tf2::Stamped<T>, void>
{
	static std::string* pointer(tf2::Stamped<T>& m) { return &m.frame_id_; }
	static std::string const* pointer(const tf2::Stamped<T>& m) { return &m.frame_id_; }
	static std::string value(const tf2::Stamped<T>& m) { return m.frame_id_; }
};

template <typename T>
struct TimeStamp<typename tf2::Stamped<T>, void>
{
	static ros::Time* pointer(tf2::Stamped<T>& m) { return &m.stamp_; }
	static ros::Time const* pointer(const tf2::Stamped<T>& m) { return &m.stamp_; }
	static ros::Time value(const tf2::Stamped<T>& m) { return m.stamp_; }
};

}  // namespace message_traits
}  // namespace ros

#include <Eigen/Geometry>
namespace tf2 {

// template specialization to allow tf2 applying transforms to our data type
template <>
void doTransform<tactile::PCLCollector::ContactPoint>(const tactile::PCLCollector::ContactPoint& in,
                                                      tactile::PCLCollector::ContactPoint& out,
                                                      const geometry_msgs::TransformStamped& t)
{
	Eigen::Quaternionf rotation(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y,
	                            t.transform.rotation.z);
	Eigen::Translation3f translation(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
	out.getVector3fMap() = translation * (rotation * in.getVector3fMap());
	out.getNormalVector3fMap() = rotation * in.getNormalVector3fMap();
	out.intensity = in.intensity;
}

}  // namespace tf2
