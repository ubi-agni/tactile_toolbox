
#include <tactile_pcl/tactile_pcl.h>

#include <pcl_conversions/pcl_conversions.h>

// /////////////////////////////////////////////////////////////////////////////////////////////////

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> CloudT;

// /////////////////////////////////////////////////////////////////////////////////////////////////

float normal_len(PointT const &p) {
	return std::sqrt(p.normal_x*p.normal_x+p.normal_y*p.normal_y+p.normal_z*p.normal_z);
}

float normalizedNormal(PointT &p) {
	float len = normal_len(p);
	p.normal_x = p.normal_x/len;
	p.normal_y = p.normal_y/len;
	p.normal_z = p.normal_z/len;
	return len;
}

// /////////////////////////////////////////////////////////////////////////////////////////////////

TactileToPointCloud::TactileToPointCloud()
	: m_node(), m_queue(), m_spinner(1,&m_queue), m_tf_listener(m_node) {
	m_node.setCallbackQueue(&m_queue);
}

TactileToPointCloud::~TactileToPointCloud() {}

void TactileToPointCloud::init(std::string const &tactile_topic,
															 std::string const &pointcloud_topic,
															 std::string const &base_tf) {

	m_base_tf = base_tf;

	if (!tactile_topic.empty()) {
		std::string ros_topic = m_node.resolveName(tactile_topic);
		std::cout << "ros_topic: " << ros_topic << std::endl;
		m_subscriber = m_node.subscribe(ros_topic,10,&TactileToPointCloud::callback,this);
	}

	if (!pointcloud_topic.empty()) {
		std::string ros_topic = m_node.resolveName(pointcloud_topic);
		m_publisher = m_node.advertise<sensor_msgs::PointCloud2>(ros_topic,10);
	}
}

void TactileToPointCloud::start() {
	m_spinner.start();
}

void TactileToPointCloud::stop() {
	m_spinner.stop();
}

void TactileToPointCloud::setNameFilter(boost::regex const &expr, bool filter_negative) {
	m_expr = expr;
	m_filter_negative = filter_negative;
}

void TactileToPointCloud::convert(TactileContactsMSGPtr const &in_msg, PCMSG &out_msg) {

	TactileContactsMSG::_contacts_type &contacts = in_msg->contacts;
	CloudT cloud;

	for (uint i = 0; i < contacts.size(); ++i) {
		TactileContactMSG &contact = contacts[i];
		const TactileContactMSG::_name_type &name = contact.name;
		if (!m_expr.empty() && !(boost::regex_match(name,m_expr)^m_filter_negative)) {
			continue;
		}
		const TactileContactMSG::_position_type &pos = contact.position;
		const TactileContactMSG::_normal_type &normal = contact.normal;
		const TactileContactMSG::_wrench_type &wrench = contact.wrench;
		tf::Vector3 p(pos.x,pos.y,pos.z);
		tf::Vector3 n(normal.x,normal.y,normal.z);

		try {
			tf::StampedTransform transform;
			m_tf_listener.lookupTransform(name,m_base_tf,ros::Time(0),transform);
			p = transform(p);
			n = transform.getBasis()*n;
		} catch(std::runtime_error const &e) {
			std::cerr << "[WARNING] Unable to get transformation for '" << name << "' and '" << m_base_tf << "'!" << std::endl;
			std::cerr << e.what() << std::endl;
		}

		PointT pcl_p;
		pcl_p.x = p.getX(); pcl_p.y = p.getY(); pcl_p.z = p.getZ();
		pcl_p.normal_x = n.getX(); pcl_p.normal_y = n.getY(); pcl_p.normal_z = n.getZ();
		pcl_p.intensity = normalizedNormal(pcl_p);
		cloud.push_back(pcl_p);
		//TODO wrench?
	}

	// this cloud is not ordered
	cloud.height = 1;
	cloud.width = cloud.points.size();

	// convert to ros message
	pcl::toROSMsg(cloud,out_msg);
	out_msg.header.stamp = in_msg->header.stamp;
	out_msg.header.frame_id = in_msg->header.frame_id;
	out_msg.header.seq = in_msg->header.seq;
}

void TactileToPointCloud::publish(const PCMSG &out_msg) {
	m_publisher.publish(out_msg);
}

void TactileToPointCloud::callback(TactileContactsMSGPtr const &data) {

	PCMSG out_msg;
	convert(data,out_msg);
	publish(out_msg);
}

