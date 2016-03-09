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

#include "tactile_array_visual.h"
#include "color_map.h"

#include <rviz/config.h>
#include <ros/console.h>

namespace rviz {
namespace tactile {

TactileArrayVisual::TactileArrayVisual(const std::string &name, const std::string &frame,
                                       const urdf::TactileArraySharedPtr &array,
                                       rviz::Display *owner, DisplayContext *context,
                                       Ogre::SceneNode *parent_node, rviz::Property *parent_property)
   : TactileVisualBase(name, frame, owner, context, parent_node, parent_property)
{
  cloud_ = new rviz::PointCloud();
  scene_node_->attachObject(cloud_);
  cloud_->setRenderMode(rviz::PointCloud::RM_BOXES);
  cloud_->setDimensions(array->size.x, array->size.y, 0.0f);

  points_.resize(array->rows * array->cols);
  values_.init(points_.size());

  size_t idx = 0;
  for (auto it = points_.begin(), end = points_.end(); it != end; ++it, ++idx) {
    size_t row, col;
    if (array->order == urdf::TactileArray::ROWMAJOR) {
      row = idx / array->cols;
      col = idx % array->cols;
    } else {
      row = idx % array->rows;
      col = idx / array->rows;
    }
    it->position.x = row * array->spacing.x - array->offset.x;
    it->position.y = col * array->spacing.y - array->offset.y;
    it->position.z = 0;
  }
}

void TactileArrayVisual::update(const ros::Time &stamp,
                                const sensor_msgs::ChannelFloat32::_values_type &values)
{
  if (values.size() == values_.size()) {
    values_.updateValues(values);
    last_update_time_ = stamp;
  } else {
    ROS_ERROR_STREAM("invalid number of taxels for " << name_);
  }
}

void TactileArrayVisual::update()
{
  updatePose();

  auto p = points_.begin();
  for (auto it = values_.begin(), end = values_.end(); it != end; ++it, ++p) {
    const QColor &c = color_map_->map(it->value(mode_));
    p->color.r = c.redF();
    p->color.g = c.greenF();
    p->color.b = c.blueF();
    p->color.a = c.alphaF();
  }

  cloud_->clear();
  cloud_->addPoints(&points_.front(), points_.size());
}

}
}
