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

#include "tactile_visual_base.h"
#include "range_property.h"
#include "color_map.h"

#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <ros/time.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace rviz {
namespace tactile {

TactileVisualBase::TactileVisualBase(const std::string &name,
                                     const std::string &frame, const urdf::Pose &origin,
                                     rviz::Display *owner, rviz::DisplayContext *context,
                                     Ogre::SceneNode *parent_node, rviz::Property *parent_property)
  : rviz::BoolProperty(QString::fromStdString(name), true, "", parent_property)
  , owner_(owner), context_(context), scene_node_(parent_node->createChildSceneNode())
  , name_(name), frame_(frame)
  , color_map_(0)
  , mode_(::tactile::TactileValue::rawCurrent)
  , acc_mode_(::tactile::TactileValueArray::Sum), acc_mean_(true)
  , enabled_(false)
{
  pose_.position.x = origin.position.x;
  pose_.position.y = origin.position.y;
  pose_.position.z = origin.position.z;
  pose_.orientation.w = origin.rotation.w;
  pose_.orientation.x = origin.rotation.x;
  pose_.orientation.y = origin.rotation.y;
  pose_.orientation.z = origin.rotation.z;

  this->connect(this, SIGNAL(changed()), SLOT(onVisibleChanged()));
  range_property_ = new RangeProperty("data range", "", this);
  connect(range_property_, SIGNAL(edited()), this, SLOT(setRawRangeFromProperty()));
  acc_value_property_ = new rviz::FloatProperty
      ("current value", 0, "current value across sensor according to accumulation mode", this);
  acc_value_property_->setReadOnly(true);
}

TactileVisualBase::~TactileVisualBase()
{
  context_->getSceneManager()->destroySceneNode(scene_node_);
}

void TactileVisualBase::setColorMap(const ColorMap *color_map)
{
  color_map_ = color_map;
}

void TactileVisualBase::setMode(::tactile::TactileValue::Mode mode)
{
  mode_ = mode;
}

void TactileVisualBase::setAccumulationMode(::tactile::TactileValueArray::AccMode mode, bool mean)
{
  acc_mode_ = mode;
  acc_mean_ = mean;
}

QColor TactileVisualBase::mapValue(const ::tactile::TactileValue &value)
{
  static QColor errColor("magenta");
  float v = value.value(mode_);
  // normalize to range 0..1
  if (mode_ == ::tactile::TactileValue::rawCurrent ||
      mode_ == ::tactile::TactileValue::rawMean) {
    v = (v - raw_range_.min()) / raw_range_.range();
  }
  if (isnan(v)) return errColor;

  QColor color = color_map_->map(v);
  return color;
}

void TactileVisualBase::update(const ros::Time &stamp)
{
  last_update_time_ = stamp;
  for (auto it = values_.begin(), end = values_.end(); it != end; ++it)
    raw_range_.update(it->absRange());
}

bool TactileVisualBase::expired(const ros::Time &timeout)
{
  return last_update_time_ <= timeout;
}

bool TactileVisualBase::updatePose()
{
  Ogre::Vector3 pos;
  Ogre::Quaternion quat;
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), pose_, pos, quat))
  {
    std::string error;
    context_->getFrameManager()->transformHasProblems(frame_, ros::Time(), error);
    owner_->setStatusStd(rviz::StatusProperty::Error, name_, error);
    return false;
  }
  owner_->setStatusStd(rviz::StatusProperty::Ok, name_, "");
  scene_node_->setPosition(pos);
  scene_node_->setOrientation(quat);
  return true;
}

void TactileVisualBase::setRawRangeFromProperty()
{
  raw_range_.init(range_property_->min(), range_property_->max());
}

void TactileVisualBase::updateRangeProperty()
{
  range_property_->update(raw_range_);
  float value = values_.accumulate(mode_, acc_mode_, acc_mean_);
  if (value > -FLT_MAX && value < FLT_MAX)
    acc_value_property_->setFloat(value);
  else
    acc_value_property_->setValue("");
}

void TactileVisualBase::reset()
{
  values_.reset();
  range_property_->reset();
  setRawRangeFromProperty();
}

void TactileVisualBase::onVisibleChanged()
{
  scene_node_->setVisible(isVisible() && isEnabled());
}

void TactileVisualBase::setVisible(bool visible)
{
  this->setBool(visible);
}

void TactileVisualBase::setEnabled(bool enabled)
{
  enabled_ = enabled;
  onVisibleChanged();
}

Qt::ItemFlags TactileVisualBase::getViewFlags(int column) const {
  Qt::ItemFlags flags = BoolProperty::getViewFlags(column);
  if (!enabled_) flags &= ~Qt::ItemIsEnabled;
  return flags;
}

}
}
