#pragma once

#include <message_filters/pass_through.h>
#include <tactile_msgs/TactileContacts.h>
namespace tactile {

class ContactForwarder : public message_filters::SimpleFilter<tactile_msgs::TactileContact>
{
public:
	template<typename F>
	ContactForwarder(F& f)
	{
		connectInput(f);
	}

	template<class F>
	void connectInput(F& f)
	{
		incoming_connection_.disconnect();
		incoming_connection_ = f.registerCallback(message_filters::SimpleFilter<tactile_msgs::TactileContacts>::EventCallback(boost::bind(&ContactForwarder::cb, this, _1)));
	}

private:
	void cb(const ros::MessageEvent<const tactile_msgs::TactileContacts>& evt)
	{
		const boost::shared_ptr<const tactile_msgs::TactileContacts> &msg = evt.getConstMessage();
		for (tactile_msgs::TactileContacts::_contacts_type::const_iterator
		     it = msg->contacts.begin(), end = msg->contacts.end(); it != end; ++it) {
			MConstPtr contact(msg, &(*it));
			this->signalMessage(EventType(contact, evt.getConnectionHeaderPtr(), evt.getReceiptTime()));
		}
	}

	message_filters::Connection incoming_connection_;
};

} // namespace tactile
