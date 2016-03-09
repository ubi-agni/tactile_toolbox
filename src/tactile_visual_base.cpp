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

#include <rviz/properties/property.h>
#include <rviz/frame_manager.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <ros/time.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace rviz {
namespace tactile {

TactileVisualBase::TactileVisualBase(const std::string &name, const std::string &frame,
                                     rviz::Display *owner, rviz::DisplayContext *context,
                                     Ogre::SceneNode *parent_node, rviz::Property *parent_property)
  : rviz::BoolProperty(QString::fromStdString(name), true, "", parent_property)
  , owner_(owner), context_(context), scene_node_(parent_node->createChildSceneNode())
  , name_(name), frame_(frame)
  , color_map_(0), mode_(::tactile::TactileValue::absMean)
  , enabled_(false)

{
  this->connect(this, SIGNAL(changed()), SLOT(onVisibleChanged()));
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
