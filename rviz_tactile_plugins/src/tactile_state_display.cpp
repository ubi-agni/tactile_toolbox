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

#include "tactile_state_display.h"
#include "tactile_taxels_visual.h"
#include "tactile_array_visual.h"
#include "group_property.h"

#include <urdf/sensor.h>
#include <urdf_tactile/tactile.h>
#include <urdf_tactile/cast.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/validate_floats.h>

#include <boost/foreach.hpp>
static const QString ROBOT_DESC = "robot description";

using namespace urdf::tactile;

namespace rviz {
namespace tactile {

TactileStateDisplay::TactileStateDisplay() : mode_(::tactile::TactileValue::rawCurrent)
{
	topic_property_ = new rviz::RosTopicProperty("Topic", "/tactile_states", "tactile_msgs/TactileState", "", this,
	                                             SLOT(onTopicChanged()));

	robot_description_property_ =
	    new rviz::StringProperty(ROBOT_DESC, "robot_description", ROBOT_DESC + " defining tactile sensors", this,
	                             SLOT(onRobotDescriptionChanged()));

	tf_prefix_property_ =
	    new StringProperty("TF Prefix", "",
	                       "Usually the robot link names are the same as the tf frame names. "
	                       "This option allows you to set a prefix. Mainly useful for multi-robot situations.",
	                       this, SLOT(onTFPrefixChanged()));

	mode_property_ =
	    new rviz::EnumProperty("display mode", QString::fromStdString(::tactile::TactileValue::getModeName(mode_)), "",
	                           this, SLOT(onModeChanged()));

	mean_lambda_property_ = new rviz::FloatProperty("mean lambda", 0.7, "sliding average discount for mean computation",
	                                                mode_property_, SLOT(onModeParamsChanged()), this);
	mean_lambda_property_->setMin(0.0);
	mean_lambda_property_->setMax(1.0);

	range_lambda_property_ =
	    new rviz::FloatProperty("range lambda", 0.9995, "sliding average discount for range computation", mode_property_,
	                            SLOT(onModeParamsChanged()), this);
	range_lambda_property_->setMin(0.0);
	range_lambda_property_->setMax(1.0);

	release_decay_property_ = new rviz::FloatProperty("release decay", 0.05, "linear decay for release", mode_property_,
	                                                  SLOT(onModeParamsChanged()), this);
	release_decay_property_->setMin(0.0);
	release_decay_property_->setMax(1.0);

	timeout_property_ = new rviz::FloatProperty("display timeout", 1, "", this);

	sensors_property_ = new GroupProperty("sensors", true, "", this, SLOT(onAllVisibleChanged()));
	sensors_property_->collapse();

	// init mode_property_
	for (unsigned int m = ::tactile::TactileValue::rawCurrent, end = ::tactile::TactileValue::lastMode; m != end; ++m) {
		mode_property_->addOptionStd(::tactile::TactileValue::getModeName(::tactile::TactileValue::Mode(m)), m);
	}

	// init color maps
	QStringList colorNames;
	abs_color_map_.init(0, 1);
	colorNames << "black"
	           << "lime"
	           << "yellow"
	           << "red";
	abs_color_map_.append(colorNames);

	rel_color_map_.init(-1, 1);
	colorNames.clear();
	colorNames << "red"
	           << "black"
	           << "lime";
	rel_color_map_.append(colorNames);
}

TactileStateDisplay::~TactileStateDisplay()
{
	unsubscribe();
}

void TactileStateDisplay::subscribe()
{
	if (!isEnabled() || topic_property_->getTopicStd().empty() || sensors_.empty())
		return;

	try {
		last_msg_ = ros::Time();
		setStatus(StatusProperty::Warn, "Topic", "No message received yet.");
		sub_ = nh_.subscribe(topic_property_->getTopicStd(), 10, &TactileStateDisplay::processMessage, this);
	} catch (const ros::Exception &e) {
		setStatus(StatusProperty::Error, "Topic", e.what());
	}
}

void TactileStateDisplay::unsubscribe()
{
	sub_.shutdown();
}

void TactileStateDisplay::setTopic(const QString &topic, const QString & /*datatype*/)
{
	topic_property_->setString(topic);
}

void TactileStateDisplay::onInitialize() {}

void TactileStateDisplay::reset()
{
	// amongst others, this method is called when time was reset
	ros::Time now = ros::Time::now();
	if (now < last_update_) {
		ROS_WARN_STREAM("Detected jump back in time of " << (last_update_ - now).toSec() << "s. Clearing taxels.");
		for (auto &sensor : sensors_)
			sensor.second->resetTime();  // expire the sensor data
	} else
		// If time was reset, don't clear display statuses via Display::reset()
		Display::reset();
}

void TactileStateDisplay::resetTactile()
{
	// reset tactile_filters
	for (auto &sensor : sensors_)
		sensor.second->reset();
}

void TactileStateDisplay::onEnable()
{
	onRobotDescriptionChanged();
}

void TactileStateDisplay::onDisable()
{
	unsubscribe();
	reset();
	resetTactile();
}

void TactileStateDisplay::onTopicChanged()
{
	unsubscribe();
	subscribe();
	context_->queueRender();
}

GroupProperty *TactileStateDisplay::getGroupProperty(const QString &path, GroupProperty *parent)
{
	assert(parent);
	QStringList names = path.split("/", QString::SkipEmptyParts);
	Q_FOREACH (const QString &name, names) {
		GroupProperty *child = nullptr;
		for (int i = 0, end = parent->numChildren(); i < end && !child; ++i) {
			rviz::Property *prop = parent->childAtUnchecked(i);
			if (prop->getName() != name)
				continue;
			child = dynamic_cast<GroupProperty *>(prop);
		}
		if (child)
			parent = child;
		else
			parent = new GroupProperty(name, parent->getBool(), "", parent, SLOT(onAllVisibleChanged()), this);
	}
	return parent;
}

void TactileStateDisplay::onRobotDescriptionChanged()
{
	// save settings of old sensors to restore them later
	std::map<QString, rviz::Config> configs;
	for (auto &sensor : sensors_) {
		rviz::Config config;
		sensor.second->save(config);
		configs[sensor.second->getName()] = config;
		delete sensor.second;
	}

	sensors_.clear();
	urdf::SensorMap sensors;
	const std::string &tf_prefix = tf_prefix_property_->getStdString();

	try {
		sensors =
		    urdf::parseSensorsFromParam(robot_description_property_->getStdString(), urdf::getSensorParser("tactile"));

		// create a TactileVisual for each tactile sensor listed in the URDF model
		for (const auto &sensor : sensors) {
			urdf::tactile::TactileSensorConstSharedPtr tactile = urdf::tactile::tactile_sensor_cast(sensor.second);
			if (!tactile)
				continue;  // some other sensor than tactile

			TactileVisualBase *visual = nullptr;
			if (tactile->array_) {
				visual = new TactileArrayVisual(sensor.first, sensor.second->parent_link_, sensor.second->origin_,
				                                tactile->array_, this, context_, scene_node_);
			} else if (!tactile->taxels_.empty()) {
				visual = new TactileTaxelsVisual(sensor.first, sensor.second->parent_link_, sensor.second->origin_,
				                                 tactile->taxels_, this, context_, scene_node_);
			}
			if (visual) {
				GroupProperty *group_property =
				    getGroupProperty(QString::fromStdString(sensor.second->group_), sensors_property_);
				group_property->addChild(visual);
				visual->setGroup(QString::fromStdString(sensor.second->group_));
				visual->setTFPrefix(tf_prefix);
				sensors_.insert(std::make_pair(tactile->channel_, visual));

				// restore sensor settings (if available)
				auto config = configs.find(visual->getName());
				if (config != configs.end())
					visual->load(config->second);
			}
		}
		if (!sensors_.empty())
			setStatus(rviz::StatusProperty::Ok, ROBOT_DESC, QString("Found %1 tactile sensors").arg(sensors_.size()));
		else
			setStatus(rviz::StatusProperty::Warn, ROBOT_DESC, "No tactile sensors found");
	} catch (const std::exception &e) {
		setStatus(rviz::StatusProperty::Error, ROBOT_DESC, e.what());
	}

	sensors_property_->removeEmptyChildren();

	onModeChanged();
	onModeParamsChanged();
	subscribe();
	context_->queueRender();
}

void TactileStateDisplay::onTFPrefixChanged()
{
	const std::string &tf_prefix = tf_prefix_property_->getStdString();
	for (auto &sensor : sensors_) {
		sensor.second->setTFPrefix(tf_prefix);
	}
	clearStatuses();
	context_->queueRender();
}

void TactileStateDisplay::onModeChanged()
{
	mode_ = ::tactile::TactileValue::getMode(mode_property_->getStdString());
	ColorMap *color_map = nullptr;

	// choose color map based on mode
	switch (mode_) {
		case ::tactile::TactileValue::dynCurrentRelease:
		case ::tactile::TactileValue::dynMeanRelease:
			color_map = &rel_color_map_;
			break;
		default:
			color_map = &abs_color_map_;
			break;
	}

	for (auto &sensor : sensors_) {
		sensor.second->setMode(mode_);
		sensor.second->setColorMap(color_map);
	}
}

void TactileStateDisplay::onModeParamsChanged()
{
	for (auto &sensor : sensors_) {
		sensor.second->setMeanLambda(mean_lambda_property_->getFloat());
		sensor.second->setRangeLambda(range_lambda_property_->getFloat());
		sensor.second->setReleaseDecay(release_decay_property_->getFloat());
	}
}

void TactileStateDisplay::onAllVisibleChanged()
{
	GroupProperty *parent = dynamic_cast<GroupProperty *>(sender());
	parent->setBoolRecursively(parent->getBool());

	// once hide/show the actual sensors in the end
	for (auto &sensor : sensors_) {
		sensor.second->onVisibleChanged();
	}
}

// This is our callback to handle an incoming message.
void TactileStateDisplay::processMessage(const tactile_msgs::TactileState::ConstPtr &msg)
{
	last_msg_ = ros::Time::now();
	if (msg->header.stamp + ros::Duration(timeout_property_->getFloat()) < last_msg_)
		setStatus(StatusProperty::Error, "Topic", "Received an outdated msg");
	else
		setStatus(StatusProperty::Ok, "Topic", "Ok");

	for (auto sensor = msg->sensors.begin(), end = msg->sensors.end(); sensor != end; ++sensor) {
		const std::string &channel = sensor->name;
		auto range = sensors_.equal_range(channel);
		for (auto s = range.first, range_end = range.second; s != range_end; ++s) {
			s->second->updateValues(msg->header.stamp, sensor->values);
		}
	}
}

void TactileStateDisplay::update(float wall_dt, float ros_dt)
{
	if (!this->isEnabled())
		return;

	Display::update(wall_dt, ros_dt);

	ros::Time now = ros::Time::now();
	ros::Duration timeout(timeout_property_->getFloat());
	last_update_ = now;
	if (!last_msg_.isZero() && last_msg_ + timeout < now)
		setStatus(StatusProperty::Warn, "Topic", "No recent msg");

	for (auto &it : sensors_) {
		TactileVisualBase &sensor = *it.second;
		sensor.updateRangeProperty();
		if (!sensor.isVisible())
			continue;

		bool enabled = !sensor.expired(now, timeout) && sensor.updatePose();
		sensor.setEnabled(enabled);
		if (!enabled)
			continue;

		sensor.updateVisual();
	}
}

}  // end namespace tactile
}  // end namespace rviz
