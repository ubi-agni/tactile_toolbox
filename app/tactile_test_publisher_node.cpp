
#include <ros/ros.h>
#include <ros/publisher.h>

#include <tactile_msgs/TactileContacts.h>

int main(int argc, char **argv) {
	ros::init(argc,argv,"tactile_test_publisher");

	ros::NodeHandle n;
	ros::Publisher publ = n.advertise<tactile_msgs::TactileContacts>(std::string("/tactile_contact_states"),10);

	tactile_msgs::TactileContacts msgs;
	msgs.header = std_msgs::Header();

	for (int i = 0; i < 10; ++i) {
		for (int k = 0; k < 10; ++k) {
			tactile_msgs::TactileContact contact;
			contact.name = "lh_lfdistal";
			contact.normal.x = 1.0; contact.normal.y = 0.0; contact.normal.z = 0.0;
			contact.position.x = 0.0; contact.position.y = 100*k; contact.position.z = 100*i;
			msgs.contacts.push_back(contact);
		}
	}

	ros::Rate loop_rate(500.0);

	while(ros::ok()) {
		publ.publish(msgs);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
