#include "pcl_collector.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>

using namespace tactile;

int main(int argc, char **argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("tactile_pcl", 10);
	PCLCollector collector(nh_priv.param<std::string>("frame", ""));

	message_filters::Subscriber<tactile_msgs::TactileContact> sub(nh, "tactile_contact_states", 10);
	collector.setSource(sub);

	sensor_msgs::PointCloud2 msg;
	ros::Rate rate(nh_priv.param("rate", 100.));
	while (ros::ok())
	{
		ros::spinOnce();
		{
			boost::unique_lock<boost::mutex> lock(collector);
			pcl::toROSMsg(collector.pcl(), msg);
			msg.header.frame_id = collector.targetFrame();
			collector.clear();
		}
		msg.header.stamp = ros::Time::now();
		msg.header.seq++;
		pub.publish(msg);
		rate.sleep();
	}
	return 0;
}
