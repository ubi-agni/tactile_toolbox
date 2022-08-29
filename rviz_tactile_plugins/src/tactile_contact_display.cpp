/*
 * Copyright (C) 2016, Bielefeld University, CITEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
 */

#include "rviz_tactile_plugins/tactile_contact_display.h"

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/default_plugin/wrench_visual.h>
#include <tf/tf.h>
#include <QApplication>
#include <QTimer>
#include <boost/thread/locks.hpp>
#include <functional>

namespace rviz {
namespace tactile {

TactileContactTopicProperty::TactileContactTopicProperty(const QString &name, const QString &default_value,
                                                         const QString &description, rviz::Property *parent,
                                                         const char *changed_slot, QObject *receiver)
  : rviz::RosTopicProperty(name, default_value, "", description, parent, changed_slot, receiver)
{}

void TactileContactTopicProperty::fillTopicList()
{
	QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
	clearOptions();

	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

	// Loop through all published topics
	ros::master::V_TopicInfo::iterator it;
	for (it = topics.begin(); it != topics.end(); ++it) {
		const ros::master::TopicInfo &topic = *it;

		// Only add topics whose type matches.
		if (topic.datatype == "tactile_msgs/TactileContact" || topic.datatype == "tactile_msgs/TactileContacts") {
			addOptionStd(topic.name);
		}
	}
	sortOptions();
	QApplication::restoreOverrideCursor();
}

TactileContactDisplay::TactileContactDisplay() : Display(), full_update_(true)
{
	topic_property_ =
	    new TactileContactTopicProperty("Topic", "tactile_contact_states", "", this, SLOT(onTopicChanged()));

	tf_prefix_property_ =
	    new StringProperty("TF Prefix", "",
	                       "Usually the robot link names are the same as the tf frame names. "
	                       "This option allows you to set a prefix. Mainly useful for multi-robot situations.",
	                       this, SLOT(onTFPrefixChanged()));

	at_contact_point_property_ = new rviz::BoolProperty("Display wrench in contact frame?", true, "", this);

	timeout_property_ = new rviz::FloatProperty("Display timeout", 1, "", this);

	force_color_property_ = new rviz::ColorProperty("Force Color", QColor(204, 51, 51), "Color to draw force arrows.",
	                                                this, SLOT(triggerFullUpdate()));

	torque_color_property_ = new rviz::ColorProperty(
	    "Torque Color", QColor(204, 204, 51), "Color to draw the torque arrows.", this, SLOT(triggerFullUpdate()));

	alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1 is fully opaque.", this,
	                                          SLOT(triggerFullUpdate()));

	scale_property_ = new rviz::FloatProperty("Overall Scale", 0.02, "", this, SLOT(triggerFullUpdate()));
	force_scale_property_ =
	    new rviz::FloatProperty("Force Arrow Scale", 1.0, "", scale_property_, SLOT(triggerFullUpdate()), this);
	torque_scale_property_ =
	    new rviz::FloatProperty("Torque Arrow Scale", 1.0, "", scale_property_, SLOT(triggerFullUpdate()), this);
	width_property_ = new rviz::FloatProperty("Arrow Width", 0.5, "", scale_property_, SLOT(triggerFullUpdate()), this);
	hide_small_values_property_ =
	    new rviz::BoolProperty("Hide Small Values", true, "Hide small values", this, SLOT(triggerFullUpdate()));
}

TactileContactDisplay::~TactileContactDisplay()
{
	unsubscribe();
}

void TactileContactDisplay::subscribe()
{
	if (!isEnabled() || topic_property_->getTopicStd().empty())
		return;

	try {
		last_msg_ = ros::Time();
		setStatus(StatusProperty::Warn, "Topic", "No message received yet.");

		const std::string &topic = topic_property_->getTopicStd();

		// infer topic's msg type
		ros::master::V_TopicInfo topics;
		ros::master::getTopics(topics);
		auto it = topics.begin(), end = topics.end();
		for (; it != end && it->name != topic; ++it)
			;
		if (it == end) {
			setStatus(StatusProperty::Error, "Topic", "Not yet published, cannot infer msg type");
			// try again in a second
			QTimer::singleShot(1000, this, &TactileContactDisplay::subscribe);
			return;
		}

		if (it->datatype == "tactile_msgs/TactileContact")
			sub_ = nh_.subscribe(topic, 100, &TactileContactDisplay::processMessage, this);
		else if (it->datatype == "tactile_msgs/TactileContacts")
			sub_ = nh_.subscribe(topic, 5, &TactileContactDisplay::processMessages, this);
		else
			// should not happen due to type filtering in TactileContactTopicProperty
			throw ros::Exception(std::string("unhandled msg type: " + it->datatype));
	} catch (const ros::Exception &e) {
		setStatus(StatusProperty::Error, "Topic", e.what());
	}
}

void TactileContactDisplay::unsubscribe()
{
	sub_.shutdown();
	contacts_.clear();
}

void TactileContactDisplay::setTopic(const QString &topic, const QString & /*datatype*/)
{
	topic_property_->setString(topic);
}

void TactileContactDisplay::onInitialize() {}

void TactileContactDisplay::reset()
{
	// amongst others, this method is called when time was reset
	ros::Time now = ros::Time::now();
	if (now < last_update_) {
		ROS_WARN_STREAM("Detected jump back in time of " << (last_update_ - now).toSec() << "s. Clearing contacts.");
		contacts_.clear();
	} else
		// If time was reset, don't clear display statuses via Display::reset()
		Display::reset();
}

void TactileContactDisplay::onEnable()
{
	subscribe();
}

void TactileContactDisplay::onDisable()
{
	unsubscribe();
}

void TactileContactDisplay::onTopicChanged()
{
	unsubscribe();
	subscribe();
	context_->queueRender();
}

void TactileContactDisplay::onTFPrefixChanged()
{
	clearStatuses();
	context_->queueRender();
}

void TactileContactDisplay::triggerFullUpdate()
{
	full_update_ = true;
	context_->queueRender();
}

void TactileContactDisplay::processMessage(const tactile_msgs::TactileContact &msg)
{
	const auto &id = std::make_pair(msg.header.frame_id, msg.name);
	auto it = contacts_.find(id);
	if (it != contacts_.end()) {
		tactile_msgs::TactileContact &m = it->second.first;
		m = msg;
	} else {
		contacts_.insert(std::make_pair(std::cref(id), std::make_pair(msg, WrenchVisualPtr())));
	}
}

void TactileContactDisplay::processMessage(const tactile_msgs::TactileContact::ConstPtr &msg)
{
	setStatus(StatusProperty::Ok, "Topic", "Ok");
	last_msg_ = ros::Time::now();
	boost::unique_lock<boost::mutex> lock(mutex_);
	processMessage(*msg);
}

void TactileContactDisplay::processMessages(const tactile_msgs::TactileContacts::ConstPtr &msg)
{
	setStatus(StatusProperty::Ok, "Topic", "Ok");
	last_msg_ = ros::Time::now();
	boost::unique_lock<boost::mutex> lock(mutex_);
	for (const auto &contact : msg->contacts) {
		processMessage(contact);
	}
}

void TactileContactDisplay::update(float wall_dt, float ros_dt)
{
	static const ros::Time ZERO_STAMP;
	if (!this->isEnabled())
		return;

	Display::update(wall_dt, ros_dt);

	ros::Time now = ros::Time::now();
	ros::Duration timeout(timeout_property_->getFloat());
	boost::unique_lock<boost::mutex> lock(mutex_);

	last_update_ = now;
	if (!last_msg_.isZero() && last_msg_ + timeout < now)
		setStatus(StatusProperty::Warn, "Topic", "No recent msg");

	for (auto &contact : contacts_) {
		const tactile_msgs::TactileContact &msg = contact.second.first;
		WrenchVisualPtr &visual = contact.second.second;
		bool new_visual = !visual;

		// hide visuals if they are outdated
		if (msg.header.stamp != ZERO_STAMP && msg.header.stamp + timeout < now) {
			if (visual)
				visual->setVisible(false);
			continue;  // skip further processing for this message
		}

		// Update pose of visual
		Ogre::Vector3 position;
		Ogre::Quaternion orientation;

		const std::string &tf_prefix = tf_prefix_property_->getStdString();
		const std::string &frame = tf_prefix.empty() ? msg.header.frame_id : tf::resolve(tf_prefix, msg.header.frame_id);
		// use zeroStamp to fetch most recent frame (tf is lacking behind our timestamps which caused issues)
		if (!context_->getFrameManager()->getTransform(frame, ZERO_STAMP, position, orientation)) {
			std::string error;
			context_->getFrameManager()->transformHasProblems(frame, msg.header.stamp, error);
			setStatusStd(StatusProperty::Error, frame, error);
			if (visual)
				visual->setVisible(false);
			continue;
		} else {
			deleteStatusStd(frame);
		}

		// create visual if not yet done
		if (new_visual)
			visual.reset(new WrenchVisual(context_->getSceneManager(), scene_node_));

		if (this->full_update_ || new_visual) {
			Ogre::ColourValue force_color = force_color_property_->getOgreColor();
			Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();
			float alpha = alpha_property_->getFloat();

			float scale = scale_property_->getFloat();
			float force_scale = scale * force_scale_property_->getFloat();
			float torque_scale = scale * torque_scale_property_->getFloat();
			float width = scale * width_property_->getFloat();

			visual->setForceColor(force_color.r, force_color.g, force_color.b, alpha);
			visual->setTorqueColor(torque_color.r, torque_color.g, torque_color.b, alpha);
			visual->setForceScale(force_scale);
			visual->setTorqueScale(torque_scale);
			visual->setHideSmallValues(hide_small_values_property_->getBool());
			visual->setWidth(width);
		}

		Ogre::Vector3 force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
		Ogre::Vector3 torque(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);

		if (at_contact_point_property_->getBool()) {
			Ogre::Vector3 contact_pos(msg.position.x, msg.position.y, msg.position.z);
			force = -force;
			torque += contact_pos.crossProduct(force);
			position += orientation * contact_pos;
		}
		visual->setVisible(true);
		visual->setFramePosition(position);
		visual->setFrameOrientation(orientation);
		visual->setWrench(force, torque);
	}
	this->full_update_ = false;
}

}  // end namespace tactile
}  // end namespace rviz
