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

#include "tactile_visual_base.h"
#include <urdf_tactile/tactile.h>

namespace rviz {

namespace tactile {

class TaxelEntity;
typedef boost::shared_ptr<TaxelEntity> TaxelEntityPtr;


class TactileTaxelsVisual : public TactileVisualBase
{
public:
  TactileTaxelsVisual(const std::string &name, const std::string &frame, const urdf::Pose &origin,
                      const std::vector<urdf::tactile::TactileTaxelSharedPtr> &taxels,
                      rviz::Display *owner, rviz::DisplayContext *context,
                      Ogre::SceneNode* parent_node, Property *parent_property=0);

protected:
  void update(const ros::Time &stamp, const sensor_msgs::ChannelFloat32::_values_type &values);
  void update();

protected:

  std::vector<unsigned int> mapping_;  /// mapping raw data indeces to values_
  std::vector<TaxelEntityPtr> taxels_;
};

}
}
