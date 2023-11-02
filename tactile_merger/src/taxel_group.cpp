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
#include <tactile_merger/taxel_group.h>

#include <ros/console.h>
#include <urdf_tactile/taxel_info_iterator.h>
#include <urdf_tactile/parser.h>

using namespace urdf::tactile;

namespace tactile {

TaxelGroup::TaxelGroup(const std::string &frame) : frame_(frame) {}

void TaxelGroup::addTaxel(const Taxel &taxel)
{
	taxels_.push_back(taxel);
}

static TaxelGroupPtr &getGroup(TaxelGroupMap &groups, const std::string &frame)
{
	TaxelGroupMap::iterator it = groups.find(frame);
	if (it != groups.end())
		return it->second;
	auto res = groups.insert(std::make_pair(frame, TaxelGroupPtr(new TaxelGroup(frame))));
	return res.first->second;
}

void TaxelGroup::addTaxels(const TactileSensorConstSharedPtr &sensor)
{
	TaxelGroup::TaxelMapping mapping;

	auto taxels = TaxelInfoIterable(sensor);
	for (const auto &taxel : taxels) {
		mapping[taxel.idx] = size();
		addTaxel(Taxel(taxel.position, taxel.normal));
	}
	mappings_.insert(std::make_pair(sensor->channel_, mapping));
}

/** Create a TaxelGroup for each link for which we have a tactile sensor
 *  A TaxelGroup is identified by link name.
 *  A TaxelGroup can hold several tactile sensors if they are attached to the same link.
 */
TaxelGroupMap TaxelGroup::load(const std::string &desc_param)
{
	TaxelGroupMap result;

	// create a TaxelGroup for each tactile sensor
	for (auto &sensor : parseSensorsFromParam(desc_param)) {
		TaxelGroupPtr &group = getGroup(result, sensor.second->parent_link_);
		group->addTaxels(sensor.second);
	}
	return result;
}

template <typename Iterator>
void TaxelGroup::update(const TaxelMapping &mapping, Iterator input_begin, Iterator input_end)
{
	for (const auto &pair : mapping) {
		assert(input_begin + pair.first < input_end);
		(void)(input_end);
		taxels_[pair.second].weight = *(input_begin + pair.first);
	}
}
template void TaxelGroup::update<std::vector<float>::const_iterator>(const TaxelMapping &mapping,
                                                                     std::vector<float>::const_iterator begin,
                                                                     std::vector<float>::const_iterator end);

inline void toContact(tactile_msgs::TactileContact &contact, const Eigen::Vector3d &pos, const Eigen::Vector3d &normal,
                      const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
{
	contact.position.x = pos.x();
	contact.position.y = pos.y();
	contact.position.z = pos.z();

	contact.normal.x = normal.x();
	contact.normal.y = normal.y();
	contact.normal.z = normal.z();

	contact.wrench.force.x = force.x();
	contact.wrench.force.y = force.y();
	contact.wrench.force.z = force.z();

	contact.wrench.torque.x = torque.x();
	contact.wrench.torque.y = torque.y();
	contact.wrench.torque.z = torque.z();
}

bool TaxelGroup::all(std::vector<tactile_msgs::TactileContact> &contacts,
                     const tactile_msgs::TactileContact &contact_template)
{
	unsigned int i = 0;
	Eigen::Vector3d force;
	for (auto it = taxels_.begin(), end = taxels_.end(); it != end; ++it, ++i) {
		tactile_msgs::TactileContact contact = contact_template;  // copy header and name
		contact.name.append("_");
		contact.name.append(std::to_string(i));

		force = (-it->weight) * it->normal;
		toContact(contact, it->position, it->normal, force, it->position.cross(force));
		contacts.push_back(contact);
	}
	return true;
}

bool TaxelGroup::average(tactile_msgs::TactileContact &contact)
{
	double sum = 0;
	Eigen::Vector3d pos, normal, force, torque;
	pos = normal = force = torque = Eigen::Vector3d::Zero();
	for (const auto &taxel : taxels_) {
		double w = std::abs(taxel.weight);
		sum += w;
		pos += w * taxel.position;
		normal += w * taxel.normal;
		Eigen::Vector3d f = (-taxel.weight) * taxel.normal;  // force is pointing opposite to normal
		force += f;
		torque += taxel.position.cross(f);
	}
	// Is overall force magnitude large enough to compute location?
	if (sum > Eigen::NumTraits<float>::dummy_precision())
		pos /= sum;
	else
		return false;

	normal.normalize();

	toContact(contact, pos, normal, force, torque);
	return true;
}

}  // namespace tactile
