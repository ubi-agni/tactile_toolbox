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
#include "merger.h"
#include <tactile_msgs/TactileContacts.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <ros/console.h>
#include <deque>

namespace tactile {

struct Merger::GroupData {
	GroupData(const TaxelGroupPtr &group);

	mutable boost::mutex mutex;
	TaxelGroupPtr group;
	ros::Time timestamp;
};

Merger::GroupData::GroupData(const TaxelGroupPtr &group)
   : group(group)
{
}


Merger::Merger()
{
}

void Merger::init(const std::string &param)
{
	sensors_.clear();
	groups_.clear();

	const TaxelGroupMap &groups = TaxelGroup::load(param);
	for (auto it = groups.begin(), end = groups.end(); it != end; ++it) {
		TaxelGroupPtr group = it->second;
		GroupDataPtr data(new GroupData(group));
		groups_.insert(std::make_pair(it->first, data));
		const TaxelGroup::SensorToTaxelMapping& m = group->mappings();
		for (auto sit = m.begin(), send = m.end(); sit != send; ++sit) {
			// create mapping from sensor name to (data, m)
			auto pair = std::make_pair(data, &sit->second);
			sensors_.insert(std::make_pair(sit->first, pair));
		}
	}
}

template <typename Iterator>
void Merger::update(const ros::Time &stamp, const std::string &sensor_name,
                    Iterator begin, Iterator end) {
	auto s = sensors_.find(sensor_name);
	if (s == sensors_.end()) {
		ROS_ERROR_STREAM("unknown sensor: " << sensor_name);
		return;
	}

	GroupDataPtr &data = s->second.first;
	TaxelGroupPtr &group = data->group;
	const TaxelGroup::TaxelMapping &mapping = *s->second.second;
	{
		boost::unique_lock<boost::mutex> lock(data->mutex);
		data->timestamp = stamp;
		group->update(mapping, begin, end);
	}
}
template void Merger::update<std::vector<float>::const_iterator>
(const ros::Time &stamp, const std::string &sensor_name,
std::vector<float>::const_iterator begin, std::vector<float>::const_iterator end);

tactile_msgs::TactileContacts Merger::getContacts() {
	tactile_msgs::TactileContacts contacts;
	for (auto it = groups_.begin(), end = groups_.end(); it != end; ++it) {
		const GroupDataPtr &data = it->second;
		boost::unique_lock<boost::mutex> lock(data->mutex);

		// TODO: How should we handle a stalled topic?
		tactile_msgs::TactileContact contact;
		contact.name = it->first; // group name
		contact.header.frame_id = data->group->frame();
		contact.header.stamp = data->timestamp;
		data->group->average(contact);

		// insert all contacts of the group into result array
		contacts.contacts.push_back(contact);
	}
	return contacts;
}

} // namespace tactile
