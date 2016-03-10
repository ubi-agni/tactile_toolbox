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
 */

#pragma once

#include <urdf_sensor/tactile.h>
#include <tactile_msgs/TactileState.h>
#include <geometry_msgs/Pose.h>
#include <tactile_filters/TactileValueArray.h>
#include <tactile_filters/TactileValue.h>
#include <ros/time.h>
#include <rviz/properties/bool_property.h>
#include <QColor>

namespace Ogre
{
class SceneNode;
}

namespace rviz {

class Display;
class DisplayContext;
class Property;
class BoolProperty;
class FloatProperty;

namespace tactile {

class ColorMap;

/* Each TactileVisual represents the visualization of a single tactile sensor,
 * let it be a vector of taxels or an array.
 */
class TactileVisualBase : public rviz::BoolProperty
{
Q_OBJECT
public:
  TactileVisualBase(const std::string &name, const std::string &frame,
                    rviz::Display *owner, rviz::DisplayContext *context,
                    Ogre::SceneNode* parent_node, rviz::Property *parent_property);
  virtual ~TactileVisualBase();

  Qt::ItemFlags getViewFlags(int column) const;

  /// update data buffer from new readings
  virtual void update(const ros::Time &stamp, const sensor_msgs::ChannelFloat32::_values_type &values) = 0;
  /// update sensor's scene_node_
  bool updatePose();
  /// update min/max properties from raw_range_
  void updateRangeProperties();
  /// update taxels
  virtual void update() = 0;

  /// most recent update older than timeout?
  bool expired(const ros::Time &timeout);

  /// isVisible() simply returns status of this' BoolProperty
  bool isVisible() {return this->getBool();}
  /// enabled status of Property
  bool isEnabled() {return enabled_;}

  void setColorMap(const ColorMap* color_map);
  void setMode(::tactile::TactileValue::Mode mode);
  void setAccumulationMode(::tactile::TactileValueArray::AccMode mode, bool mean);
  void setMeanLambda (float fLambda) {values_.setMeanLambda(fLambda);}
  void setRangeLambda (float fLambda) {values_.setRangeLambda(fLambda);}
  void setReleaseDecay (float fDecay) {values_.setReleaseDecay(fDecay);}

public Q_SLOTS:
  void setVisible(bool visible);
  void setEnabled(bool enabled);

protected:
  QColor mapValue(const::tactile::TactileValue &value);
  void update(const ros::Time &stamp);

protected Q_SLOTS:
  void onVisibleChanged();
  void onRangeChanged();

protected:
  rviz::Display *owner_;
  rviz::DisplayContext *context_;
  Ogre::SceneNode *scene_node_;

  std::string name_;   // sensor name
  std::string frame_;  // frame this sensor is attached to
  geometry_msgs::Pose pose_; // pose relative to this frame_

  ::tactile::TactileValueArray values_;  /// tactile values
  ros::Time last_update_time_;

  const ColorMap *color_map_;
  ::tactile::TactileValue::Mode mode_;
  ::tactile::TactileValueArray::AccMode acc_mode_;
  bool acc_mean_;

  ::tactile::Range raw_range_;
  rviz::FloatProperty *range_min_property_;
  rviz::FloatProperty *range_max_property_;
  rviz::FloatProperty *acc_value_property_;

  bool enabled_;
};

}
}
