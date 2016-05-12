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
#include "taxel_group.h"

#include <urdf/sensor.h>
#include <ros/console.h>
#include <urdf_tactile/taxel_info_iterator.h>
#include <urdf_tactile/cast.h>

namespace tactile {

TaxelGroup::TaxelGroup(const std::string &frame)
   : frame_(frame)
{
}

void TaxelGroup::addTaxel(const Taxel &taxel)
{
	taxels_.push_back(taxel);
}

static TaxelGroupPtr&
getGroup(TaxelGroupMap &groups, const std::string &frame) {
	TaxelGroupMap::iterator it = groups.find(frame);
	if (it != groups.end()) return it->second;
	auto res = groups.insert(std::make_pair(frame, TaxelGroupPtr(new TaxelGroup(frame))));
	return res.first->second;
}

void TaxelGroup::addTaxels(const urdf::SensorConstSharedPtr &sensor) {
	TaxelGroup::TaxelMapping mapping;
	const urdf::tactile::TactileSensor& tactile = urdf::tactile::tactile_sensor_cast(*sensor);

	for (auto taxel = urdf::tactile::TaxelInfoIterator::begin(sensor),
	     end = urdf::tactile::TaxelInfoIterator::end(sensor); taxel != end; ++taxel) {
		mapping[taxel->idx] = size();
		addTaxel(Taxel(taxel->position, taxel->normal));
	}
	mappings_.insert(std::make_pair(tactile.channel_, mapping));
}

/** Create a TaxelGroup for each link for which we have a tactile sensor
 *  A TaxelGroup is identified by link name.
 *  A TaxelGroup can hold several tactile sensors if they are attached to the same link.
 */
TaxelGroupMap TaxelGroup::load (const std::string &desc_param) {
	TaxelGroupMap result;

	urdf::SensorMap sensors = urdf::parseSensorsFromParam(desc_param, urdf::getSensorParser("tactile"));
	// create a TaxelGroup for each tactile sensor
	for (auto it = sensors.begin(), end = sensors.end(); it != end; ++it) {
		urdf::tactile::TactileSensorConstSharedPtr sensor = urdf::tactile::tactile_sensor_cast(it->second);
		if (!sensor) continue;  // some other sensor than tactile

		TaxelGroupPtr &group = getGroup(result, it->second->parent_link_);
		group->addTaxels(it->second);
	}
	return result;
}

template <typename Iterator>
void TaxelGroup::update(const TaxelMapping &mapping, Iterator input_begin, Iterator input_end)
{
	for (auto it = mapping.begin(), end = mapping.end(); it != end; ++it) {
		assert(input_begin + it->first < input_end);
		taxels_[it->second].weight = *(input_begin + it->first);
	}
}
template void TaxelGroup::update<std::vector<float>::const_iterator>
(const TaxelMapping &mapping,
std::vector<float>::const_iterator begin, std::vector<float>::const_iterator end);


bool TaxelGroup::average(tactile_msgs::TactileContact &contact)
{
	double sum = 0;
	Eigen::Vector3d pos, normal, force, torque;
	pos = normal = force = torque = Eigen::Vector3d::Zero();
	for (auto it = taxels_.begin(), end = taxels_.end(); it != end; ++it) {
		double w = it->weight;
		sum += w;
		pos += w * it->position;
		normal += w * it->normal;
	}
	if (sum > Eigen::NumTraits<float>::dummy_precision()) {
		pos /= sum;
		normal.normalize();
	} else
		return false;

	contact.position.x = pos.x();
	contact.position.y = pos.y();
	contact.position.z = pos.z();

	contact.normal.x = normal.x();
	contact.normal.y = normal.y();
	contact.normal.z = normal.z();

	force = (-sum) * normal; // force acts opposite to normal
	contact.wrench.force.x = force.x();
	contact.wrench.force.y = force.y();
	contact.wrench.force.z = force.z();

	torque = pos.cross(force);
	contact.wrench.torque.x = torque.x();
	contact.wrench.torque.y = torque.y();
	contact.wrench.torque.z = torque.z();
	return true;
}

} // namespace tactile
