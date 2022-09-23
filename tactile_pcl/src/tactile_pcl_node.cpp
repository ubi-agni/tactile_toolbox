#include "pcl_collector.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include "contact_forwarder.h"

using namespace tactile;

void run(ros::Publisher &pub, PCLCollector &collector, ros::Rate &rate)
{
	sensor_msgs::PointCloud2 msg;
	bool send_empty = false;  // should we send an empty message?
	while (ros::ok()) {
		ros::spinOnce();
		{
			boost::unique_lock<boost::mutex> lock(collector);
			pcl::toROSMsg(collector.pcl(), msg);
			msg.header.frame_id = collector.targetFrame();
			collector.clear();
		}
		if (!msg.data.empty() || send_empty) {
			send_empty = !msg.data.empty();  // only send a single empty message in a row
			msg.header.stamp = ros::Time::now();
			msg.header.seq++;
			pub.publish(msg);
		}
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("tactile_pcl", 10);
	ros::Rate rate(nh_priv.param("rate", 100.));
	double threshold = nh_priv.param<double>("threshold", 0.0);
	auto frame = nh_priv.param<std::string>("frame", "");

	switch (1) {
		case 0: {
			message_filters::Subscriber<tactile_msgs::TactileContact> sub(nh, "tactile_contact_state", 100);
			PCLCollector collector(sub, 10, frame, threshold);
			run(pub, collector, rate);
		} break;
		case 1: {
			message_filters::Subscriber<tactile_msgs::TactileContacts> sub(nh, "tactile_contact_states", 10);
			ContactForwarder forwarder(sub);
			PCLCollector collector(forwarder, 100, frame, threshold);
			run(pub, collector, rate);
		} break;
		case 2: {
			//		message_filters::Subscriber<tactile_msgs::TactileState> sub(nh, "tactile_states", 10);
			//		PCLCollector collector(sub, 10, frame, threshold);
			//		run(pub, collector, rate);
		} break;
	}

	return 0;
}
