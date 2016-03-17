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

#include <urdf/sensor.h>
#include <urdf_tactile/tactile.h>
#include <urdf_tactile/parser.h>

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
const QString ROBOT_DESC = "robot description";

using namespace urdf::tactile;

namespace rviz {
namespace tactile {

TactileStateDisplay::TactileStateDisplay()
  : mode_(::tactile::TactileValue::absCurrent)
{
  topic_property_ = new rviz::RosTopicProperty
      ("topic", "/tactile_state", "tactile_msgs/TactileState", "",
       this, SLOT(onTopicChanged()));

  robot_description_property_ = new rviz::StringProperty
      (ROBOT_DESC, "robot_description",
       ROBOT_DESC + " defining tactile sensors",
       this, SLOT(onRobotDescriptionChanged()));

  mode_property_ = new rviz::EnumProperty
      ("display mode", QString::fromStdString(::tactile::TactileValue::getModeName(mode_)),
       "", this, SLOT(onModeChanged()));

  mean_lambda_property_ = new rviz::FloatProperty
      ("mean lambda", 0.7, "sliding average discount for mean computation",
       mode_property_, SLOT(onModeParamsChanged()), this);
  mean_lambda_property_->setMin(0.0); mean_lambda_property_->setMax(1.0);

  range_lambda_property_ = new rviz::FloatProperty
      ("range lambda", 0.9995, "sliding average discount for range computation",
       mode_property_, SLOT(onModeParamsChanged()), this);
  range_lambda_property_->setMin(0.0); range_lambda_property_->setMax(1.0);

  release_decay_property_ = new rviz::FloatProperty
      ("release decay", 0.05, "linear decay for release",
       mode_property_, SLOT(onModeParamsChanged()), this);
  release_decay_property_->setMin(0.0); release_decay_property_->setMax(1.0);

  timeout_property_ = new rviz::FloatProperty
      ("display timeout", 1, "", this);

  sensors_property_ = new rviz::BoolProperty("sensors", true, "", this,
                                             SLOT(onAllVisibleChanged()));
  sensors_property_->collapse();

  // init mode_property_
  for (unsigned int m = ::tactile::TactileValue::rawCurrent, end = ::tactile::TactileValue::lastMode; m != end; ++m) {
    mode_property_->addOptionStd(::tactile::TactileValue::getModeName(::tactile::TactileValue::Mode(m)), m);
  }

  // init color maps
  QStringList colorNames;
  abs_color_map_.init(0,1);
  colorNames << "black" << "lime" << "yellow" << "red";
  abs_color_map_.append(colorNames);

  rel_color_map_.init(-1,1);
  colorNames.clear(); colorNames << "red" << "black" << "lime";
  rel_color_map_.append(colorNames);
}

TactileStateDisplay::~TactileStateDisplay()
{
  unsubscribe();
}

void TactileStateDisplay::subscribe()
{
  if (!isEnabled() ||
      topic_property_->getTopicStd().empty() ||
      sensors_.empty())
    return;

  try {
    sub_ = nh_.subscribe(topic_property_->getTopicStd(), 10,
                         &TactileStateDisplay::processMessage, this);
    setStatus(StatusProperty::Ok, "topic", "OK");
  } catch(const ros::Exception& e) {
    setStatus(StatusProperty::Error, "topic", QString("error subscribing: ") + e.what());
  }
}

void TactileStateDisplay::unsubscribe()
{
  sub_.shutdown();
}

void TactileStateDisplay::onInitialize()
{
  onRobotDescriptionChanged();
}

void TactileStateDisplay::reset()
{
  Display::reset();
}

void TactileStateDisplay::onEnable()
{
  subscribe();
}

void TactileStateDisplay::onDisable()
{
  unsubscribe();
  reset();
}

void TactileStateDisplay::onTopicChanged()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void TactileStateDisplay::onRobotDescriptionChanged()
{
  std::string xml_string;

  sensors_.clear();
  urdf::SensorMap sensors;
  try {
    urdf::SensorParserMap parsers;
    parsers.insert(std::make_pair("tactile", boost::shared_ptr<TactileSensorParser>(new TactileSensorParser())));
    sensors = urdf::parseSensorsFromParam(robot_description_property_->getStdString(), parsers);
  } catch (const std::exception &e) {
    setStatus(rviz::StatusProperty::Error, ROBOT_DESC, e.what());
    return;
  }

  // create a TactileVisual for each tactile sensor listed in the URDF model
  for (auto it = sensors.begin(), end = sensors.end(); it != end; ++it)
  {
    boost::shared_ptr<TactileSensor> sensor
        = boost::dynamic_pointer_cast<TactileSensor>(it->second->sensor_);
    if (!sensor) continue;  // some other sensor than tactile

    TactileVisualBasePtr visual;
    if (sensor->array_) {
      visual.reset(new TactileArrayVisual(it->first, it->second->parent_link_, it->second->origin_, sensor->array_,
                                          this, context_, scene_node_, sensors_property_));
    } else if (sensor->taxels_.size()) {
      visual.reset(new TactileTaxelsVisual(it->first, it->second->parent_link_, it->second->origin_, sensor->taxels_,
                                           this, context_, scene_node_, sensors_property_));
    }
    if (visual) sensors_[it->first] = visual;
  }
  onModeChanged();
  subscribe();
  context_->queueRender();
}

void TactileStateDisplay::onModeChanged()
{
  mode_ = ::tactile::TactileValue::getMode(mode_property_->getStdString());
  ColorMap *color_map = 0;

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

  for (auto it = sensors_.begin(), end = sensors_.end(); it != end; ++it) {
    it->second->setMode(mode_);
    it->second->setColorMap(color_map);
  }
}

void TactileStateDisplay::onModeParamsChanged()
{
  for (auto it = sensors_.begin(), end = sensors_.end(); it != end; ++it) {
    it->second->setMeanLambda(mean_lambda_property_->getFloat());
    it->second->setRangeLambda(range_lambda_property_->getFloat());
    it->second->setReleaseDecay(release_decay_property_->getFloat());
  }
}

void TactileStateDisplay::onAllVisibleChanged()
{
  bool show = sensors_property_->getBool();
  for (auto it = sensors_.begin(), end = sensors_.end(); it != end; ++it)
    it->second->setVisible(show);
}

// This is our callback to handle an incoming message.
void TactileStateDisplay::processMessage(const tactile_msgs::TactileState::ConstPtr& msg)
{
  for (auto sensor = msg->sensors.begin(), end = msg->sensors.end(); sensor != end; ++sensor)
  {
    auto it = sensors_.find(sensor->name);
    if (it == sensors_.end()) continue;
    it->second->update(msg->header.stamp, sensor->values);
  }
}

void TactileStateDisplay::update(float wall_dt, float ros_dt)
{
  if (!this->isEnabled()) return;

  Display::update(wall_dt, ros_dt);

  ros::Time timeout = ros::Time::now();
  try {
    timeout -= ros::Duration(timeout_property_->getFloat());
  } catch (const std::runtime_error &e) {
    // ros::Time::now was smaller than ros::Duration
  }

  for (auto it = sensors_.begin(), end = sensors_.end(); it != end; ++it) {
    TactileVisualBase &sensor = *it->second;
    sensor.updateRangeProperties();
    if (!sensor.isVisible()) continue;

    bool enabled = !sensor.expired(timeout) && sensor.updatePose();
    sensor.setEnabled(enabled);
    if (!enabled) continue;

    sensor.update();
  }
}

} // end namespace tactile
} // end namespace rviz
