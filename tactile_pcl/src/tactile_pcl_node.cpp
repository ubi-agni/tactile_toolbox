#include "pcl_collector.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include "contact_forwarder.h"

using namespace tactile;

void run(ros::Publisher &pub, PCLCollector &collector, ros::Rate &rate) {
	sensor_msgs::PointCloud2 msg;
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
}

int main(int argc, char **argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("tactile_pcl", 10);
	PCLCollector collector(nh_priv.param<std::string>("frame", ""));
	ros::Rate rate(nh_priv.param("rate", 100.));

	switch (1) {
	case 0: {
		message_filters::Subscriber<tactile_msgs::TactileContact> sub(nh, "tactile_contact_state", 100);
		collector.setSource<tactile_msgs::TactileContact>(sub, 10);
		run(pub, collector, rate);
	} break;
	case 1: {
		message_filters::Subscriber<tactile_msgs::TactileContacts> sub(nh, "tactile_contact_states", 10);
		ContactForwarder forwarder(sub);
		collector.setSource<tactile_msgs::TactileContact>(forwarder, 100);
		run(pub, collector, rate);
	} break;
	case 2: {
//		message_filters::Subscriber<tactile_msgs::TactileState> sub(nh, "tactile_states", 10);
//		collector.setSource(sub);
//		run(pub, collector, rate);
	} break;
	}

	return 0;
}
