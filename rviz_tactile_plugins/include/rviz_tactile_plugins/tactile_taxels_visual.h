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

#include "rviz_tactile_plugins/tactile_visual_base.h"
#include <urdf_tactile/tactile.h>

#define ENABLE_ARROWS 0

namespace rviz {

#if ENABLE_ARROWS
class Arrow;
typedef boost::shared_ptr<rviz::Arrow> ArrowPtr;
#endif

namespace tactile {

class TaxelEntity;
typedef boost::shared_ptr<TaxelEntity> TaxelEntityPtr;

class TactileTaxelsVisual : public TactileVisualBase
{
	Q_OBJECT
public:
	TactileTaxelsVisual(const std::string &name, const std::string &frame, const urdf::Pose &origin,
	                    const std::vector<urdf::tactile::TactileTaxelSharedPtr> &taxels, rviz::Display *owner,
	                    rviz::DisplayContext *context, Ogre::SceneNode *parent_node,
	                    Property *parent_property = nullptr);

protected:
	void updateValues(const ros::Time &stamp, const sensor_msgs::ChannelFloat32::_values_type &values) override;
	void updateVisual() override;

#if ENABLE_ARROWS
protected Q_SLOTS:
	void onVisibleChanged();
	void onArrowsEnabled();
#endif

protected:
	std::vector<unsigned int> mapping_;  /// mapping raw data indeces to taxels_
	std::vector<TaxelEntityPtr> taxels_;

#if ENABLE_ARROWS
	rviz::BoolProperty *arrows_property_;
	rviz::FloatProperty *arrows_scale_property_;
	Ogre::SceneNode *arrows_node_;
	std::vector<rviz::ArrowPtr> arrows_;
#endif
};

}  // namespace tactile
}  // namespace rviz
