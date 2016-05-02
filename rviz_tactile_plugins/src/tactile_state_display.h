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

#pragma once

#include <rviz/display.h>
#include <tactile_msgs/TactileState.h>
#include <tactile_filters/TactileValue.h>
#include "color_map.h"

namespace rviz
{
class Property;
class StringProperty;
class BoolProperty;
class ColorProperty;
class FloatProperty;
class RosTopicProperty;
class EnumProperty;

namespace tactile
{

class TactileVisualBase;
class GroupProperty;

class TactileStateDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TactileStateDisplay();
  ~TactileStateDisplay();

  void resetTactile();

protected:
  void subscribe();
  void unsubscribe();

  void setTopic(const QString &topic, const QString &datatype);
  void onInitialize();
  void reset();
  void onEnable();
  void onDisable();
  void update(float wall_dt, float ros_dt);

  void processMessage(const tactile_msgs::TactileState::ConstPtr& msg);
  GroupProperty *getGroupProperty(const QString &path, GroupProperty *parent);

protected Q_SLOTS:
  void onTopicChanged();
  void onRobotDescriptionChanged();
  void onTFPrefixChanged();
  void onModeChanged();
  void onModeParamsChanged();
  void onAllVisibleChanged();

private:
  rviz::RosTopicProperty* topic_property_;
  rviz::StringProperty* robot_description_property_;
  rviz::StringProperty* tf_prefix_property_;

  rviz::EnumProperty* mode_property_;
  rviz::FloatProperty* mean_lambda_property_;
  rviz::FloatProperty* range_lambda_property_;
  rviz::FloatProperty* release_decay_property_;

  rviz::FloatProperty* timeout_property_;
  GroupProperty* sensors_property_;

  ros::NodeHandle  nh_;
  ros::Subscriber  sub_;
  /// list of all sensors, accessible by sensor name
  std::map<std::string, TactileVisualBase*> sensors_;
  /// map from channel name to sensors monitoring this channel
  std::map<std::string, std::vector<TactileVisualBase*> > channels_;

  ::tactile::TactileValue::Mode mode_;
  ColorMap abs_color_map_;
  ColorMap rel_color_map_;
};

} // namespace tactile
} // namespace rviz
