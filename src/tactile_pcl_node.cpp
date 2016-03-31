#include <iostream>

#include <ros/ros.h>
#include "tactile_pcl.h"

int main(int argc, char **argv) {

	ros::init(argc,argv,"tactile_pcl_node");

	TactileToPointCloud node;

	std::string base_tf = "/world";
	std::string tactile_topic = "/tactile_contact_states";
	std::string cloud_topic = "/tactile_pcl";

	node.init(tactile_topic,cloud_topic,base_tf);
	node.start();

	ros::waitForShutdown();
	return 0;
}
