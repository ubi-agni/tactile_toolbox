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

#include "rviz_tactile_plugins/tactile_taxels_visual.h"

#include <rviz/mesh_loader.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/float_property.h>
#include <ros/console.h>

#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

using namespace urdf::tactile;

namespace rviz {
namespace tactile {

class TaxelEntity
{
public:
	TaxelEntity(const urdf::Geometry &geometry, const urdf::Pose &origin, rviz::DisplayContext *context,
	            Ogre::SceneNode *parent_node);
	~TaxelEntity();
	void setColor(float r, float g, float b, float a);

protected:
	Ogre::Entity *createEntityFromGeometry(const urdf::Geometry &geom, const urdf::Pose &origin);

	Ogre::SceneManager *scene_manager_;
	Ogre::SceneNode *taxel_node_;  // scene node all taxels are attached to

	Ogre::Entity *entity_;
	Ogre::MaterialPtr material_;
};

TaxelEntity::TaxelEntity(const urdf::Geometry &geometry, const urdf::Pose &origin, rviz::DisplayContext *context,
                         Ogre::SceneNode *parent_node)
  : scene_manager_(context->getSceneManager()), taxel_node_(parent_node->createChildSceneNode())
{
	// create material for coloring links
	std::stringstream ss;
	static int count = 0;
	ss << "taxel color material " << count;
	material_ = Ogre::MaterialPtr(new Ogre::Material(nullptr, Ogre::String(), 0, Ogre::String()));
	material_->setReceiveShadows(false);

	entity_ = createEntityFromGeometry(geometry, origin);
	setColor(0, 0, 0, 0);
}

TaxelEntity::~TaxelEntity()
{
	scene_manager_->destroyEntity(entity_);
	scene_manager_->destroySceneNode(taxel_node_);
}

Ogre::Entity *TaxelEntity::createEntityFromGeometry(const urdf::Geometry &geom, const urdf::Pose &origin)
{
	Ogre::Entity *entity = nullptr;  // default in case nothing works.
	Ogre::SceneNode *offset_node = taxel_node_->createChildSceneNode();

	static int count = 0;
	std::stringstream ss;
	ss << "taxel" << count++;
	std::string entity_name = ss.str();

	Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);
	Ogre::Vector3 position(origin.position.x, origin.position.y, origin.position.z);
	Ogre::Quaternion orientation(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);

	switch (geom.type) {
		case urdf::Geometry::SPHERE: {
			const urdf::Sphere &sphere = static_cast<const urdf::Sphere &>(geom);
			entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Sphere, scene_manager_);

			scale = Ogre::Vector3(sphere.radius * 2, sphere.radius * 2, sphere.radius * 2);
			break;
		}
		case urdf::Geometry::BOX: {
			const urdf::Box &box = static_cast<const urdf::Box &>(geom);
			entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cube, scene_manager_);

			scale = Ogre::Vector3(box.dim.x, box.dim.y, box.dim.z);
			break;
		}
		case urdf::Geometry::CYLINDER: {
			const urdf::Cylinder &cylinder = static_cast<const urdf::Cylinder &>(geom);

			Ogre::Quaternion rot_x;
			rot_x.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
			orientation = orientation * rot_x;

			entity = rviz::Shape::createEntity(entity_name, rviz::Shape::Cylinder, scene_manager_);
			scale = Ogre::Vector3(cylinder.radius * 2, cylinder.length, cylinder.radius * 2);
			break;
		}
		case urdf::Geometry::MESH: {
			const urdf::Mesh &mesh = static_cast<const urdf::Mesh &>(geom);
			if (mesh.filename.empty())
				return entity;

			scale = Ogre::Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z);
			const std::string &model_name = mesh.filename;

			loadMeshFromResource(model_name);
			entity = scene_manager_->createEntity(entity_name, model_name);
			break;
		}
		default:
			ROS_WARN("Unsupported geometry type for element: %d", geom.type);
			break;
	}

	if (entity) {
		offset_node->attachObject(entity);
		offset_node->setScale(scale);
		offset_node->setPosition(position);
		offset_node->setOrientation(orientation);
		entity->setMaterial(material_);
	}
	return entity;
}

void TaxelEntity::setColor(float r, float g, float b, float a)
{
	Ogre::SceneBlendType blending;
	bool depth_write;

	if (a < 0.9998) {
		blending = Ogre::SBT_TRANSPARENT_ALPHA;
		depth_write = false;
	} else {
		blending = Ogre::SBT_REPLACE;
		depth_write = true;
	}

	Ogre::Technique *technique = material_->getTechnique(0);
	technique->setAmbient(r * 0.5, g * 0.5, b * 0.5);
	technique->setDiffuse(r, g, b, a);
	technique->setSceneBlending(blending);
	technique->setDepthWriteEnabled(depth_write);
	technique->setLightingEnabled(true);
}

TactileTaxelsVisual::TactileTaxelsVisual(const std::string &name, const std::string &frame, const urdf::Pose &origin,
                                         const std::vector<TactileTaxelSharedPtr> &taxels, rviz::Display *owner,
                                         rviz::DisplayContext *context, Ogre::SceneNode *parent_node,
                                         rviz::Property *parent_property)
  : TactileVisualBase(name, frame, origin, owner, context, parent_node, parent_property)
{
#if ENABLE_ARROWS
	arrows_property_ = new rviz::BoolProperty("Show arrows", false, "", this, SLOT(onArrowsEnabled()));
	arrows_scale_property_ = new rviz::FloatProperty("Arrow scale", 0.01, "", arrows_property_);
	arrows_node_ = scene_node_->createChildSceneNode();
#endif

	for (const auto &taxel : taxels) {
		urdf::GeometryConstSharedPtr geometry = taxel->geometry;
		TaxelEntityPtr t(new TaxelEntity(*geometry, urdf::Pose(), context, scene_node_));
		taxels_.push_back(t);
		mapping_.push_back(taxel->idx);

#if ENABLE_ARROWS
		rviz::ArrowPtr arrow(new rviz::Arrow(context->getSceneManager(), arrows_node_));
		const urdf::Vector3 &pos = taxel->origin.position;
		const urdf::Rotation &rot = taxel->origin.rotation;
		arrow->setColor(1, 0, 0, 1);
		arrow->setPosition(Ogre::Vector3(pos.x, pos.y, pos.z));
		arrow->setDirection(Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z).zAxis());
		arrows_.push_back(arrow);
#endif
	}
	values_.init(taxels_.size());
}

void TactileTaxelsVisual::updateValues(const ros::Time &stamp, const sensor_msgs::ChannelFloat32::_values_type &values)
{
	size_t N = values.size();  // NOLINT(readability-identifier-naming)
	auto vit = values_.begin();
	for (auto it = mapping_.begin(), end = mapping_.end(); it != end; ++it, ++vit) {
		if (*it < N)
			vit->update(values[*it]);
		else
			ROS_ERROR_STREAM("too short taxel msg for " << qPrintable(getName()));
	}
	TactileVisualBase::updateRange(stamp);
}

void TactileTaxelsVisual::updateVisual()
{
	auto val_it = values_.begin();
	for (auto it = taxels_.begin(), end = taxels_.end(); it != end; ++it, ++val_it) {
		const QColor &c = mapColor(mapValue(*val_it));
		(*it)->setColor(c.redF(), c.greenF(), c.blueF(), c.alphaF());
	}

#if ENABLE_ARROWS
	float scale = arrows_scale_property_->getFloat();
	val_it = values_.begin();
	for (auto it = arrows_.begin(), end = arrows_.end(); it != end; ++it, ++val_it) {
		float value = mapValue(*val_it);
		value = std::isfinite(value) ? value * scale : 0.0;
		(*it)->setScale(Ogre::Vector3(value, value, value));
	}
#endif
}

#if ENABLE_ARROWS
void TactileTaxelsVisual::onVisibleChanged()
{
	TactileVisualBase::onVisibleChanged();
	onArrowsEnabled();
}

void TactileTaxelsVisual::onArrowsEnabled()
{
	arrows_node_->setVisible(arrows_property_->getBool());
}
#endif

}  // namespace tactile
}  // namespace rviz
