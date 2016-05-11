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
#include <rviz/properties/ros_topic_property.h>
#include <tactile_msgs/TactileContacts.h>
#include <boost/thread/mutex.hpp>

namespace rviz
{
class FloatProperty;
class ColorProperty;
class WrenchVisual;

namespace tactile
{

class TactileContactTopicProperty : public rviz::RosTopicProperty {
Q_OBJECT
public:
  TactileContactTopicProperty(const QString& name = QString(),
                            const QString& default_value = QString(),
                            const QString& description = QString(),
                            rviz::Property* parent = 0,
                            const char *changed_slot = 0,
                            QObject* receiver = 0);
protected Q_SLOTS:
  virtual void fillTopicList();
};

typedef boost::shared_ptr<WrenchVisual> WrenchVisualPtr;

class TactileContactDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TactileContactDisplay();
  ~TactileContactDisplay();

protected:
  void subscribe();
  void unsubscribe();

  void setTopic(const QString &topic, const QString &datatype);
  void onInitialize();
  void reset();
  void onEnable();
  void onDisable();
  void update(float wall_dt, float ros_dt);

  void processMessage(const tactile_msgs::TactileContact &msg);
  void processMessage(const tactile_msgs::TactileContact::ConstPtr& msg);
  void processMessages(const tactile_msgs::TactileContacts::ConstPtr& msg);

protected Q_SLOTS:
  void onTopicChanged();
  void onTFPrefixChanged();
  void triggerFullUpdate();

private:
  TactileContactTopicProperty* topic_property_;
  rviz::StringProperty* tf_prefix_property_;
  rviz::BoolProperty* at_contact_point_property_;
  rviz::FloatProperty* timeout_property_;
  rviz::ColorProperty *force_color_property_, *torque_color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::FloatProperty *scale_property_, *force_scale_property_, *torque_scale_property_, *width_property_;
  bool full_update_; // update all visual properties?

  ros::NodeHandle  nh_;
  ros::Subscriber  sub_;
  std::map<std::string, std::pair<tactile_msgs::TactileContact, WrenchVisualPtr> > contacts_;
  boost::mutex mutex_;
};

} // namespace tactile
} // namespace rviz
