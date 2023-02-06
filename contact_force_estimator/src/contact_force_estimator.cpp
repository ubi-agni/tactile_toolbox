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
#include <contact_force_estimator/contact_force_estimator.h>
#include <tactile_msgs/TactileContacts.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <ros/console.h>
#include <deque>

namespace tactile {

struct ContactForceEstimator::GroupData
{
	GroupData(const TaxelGroupPtr &group);

	mutable boost::mutex mutex;
	TaxelGroupPtr group;
	ros::Time timestamp;
};

ContactForceEstimator::GroupData::GroupData(const TaxelGroupPtr &group) : group(group) {}

ContactForceEstimator::ContactForceEstimator() {}
ContactForceEstimator::~ContactForceEstimator()
{
	sensors_.clear();
	groups_.clear();
	parsers_.clear();
}

void ContactForceEstimator::init(const std::string &param)
{
	sensors_.clear();
	groups_.clear();
	parsers_ = std::move(urdf::getSensorParser("tactile"));

	const TaxelGroupMap &groups = TaxelGroup::load(param, parsers_);
	for (const auto &name_group : groups) {
		TaxelGroupPtr group = name_group.second;
		GroupDataPtr data(new GroupData(group));
		groups_.insert(std::make_pair(name_group.first, data));
		for (const auto &sensor_taxel : group->mappings()) {
			// create mapping from channel name to (data, m)
			auto pair = std::make_pair(data, &sensor_taxel.second);
			sensors_.insert(std::make_pair(sensor_taxel.first, pair));
		}
	}
}

void ContactForceEstimator::reset()
{
	for (auto &group : groups_)
		group.second->timestamp = ros::Time();
}

template <typename Iterator>
void ContactForceEstimator::update(const ros::Time &stamp, const std::string &channel, Iterator begin, Iterator end)
{
	auto range = sensors_.equal_range(channel);
	if (range.first == range.second) {
		ROS_ERROR_STREAM_ONCE("unknown channel: " << channel);
		return;
	}

	for (auto ch = range.first, range_end = range.second; ch != range_end; ++ch) {
		GroupDataPtr &data = ch->second.first;
		TaxelGroupPtr &group = data->group;
		const TaxelGroup::TaxelMapping &mapping = *ch->second.second;
		{
			boost::unique_lock<boost::mutex> lock(data->mutex);
			data->timestamp = stamp;
			group->update(mapping, begin, end);
		}
	}
}
template void ContactForceEstimator::update<std::vector<float>::const_iterator>(
    const ros::Time &stamp, const std::string &sensor_name, std::vector<float>::const_iterator begin,
    std::vector<float>::const_iterator end);

tactile_msgs::TactileContacts ContactForceEstimator::getAllTaxelContacts()
{
	static ros::Duration timeout(1);
	ros::Time now = ros::Time::now();

	tactile_msgs::TactileContacts contacts;
	for (auto &name_data : groups_) {
		const GroupDataPtr &data = name_data.second;
		boost::unique_lock<boost::mutex> lock(data->mutex);
		if (data->timestamp + timeout < now)
			continue;  // ignore stalled groups

		tactile_msgs::TactileContact contact;
		contact.name = name_data.first;  // group name
		contact.header.frame_id = data->group->frame();
		contact.header.stamp = data->timestamp;
		// insert all contacts of the group into result array
		data->group->all(contacts.contacts, contact);
	}
	return contacts;
}

tactile_msgs::TactileContacts ContactForceEstimator::getGroupAveragedContacts()
{
	static ros::Duration timeout(1);
	ros::Time now = ros::Time::now();

	tactile_msgs::TactileContacts contacts;
	for (auto &name_data : groups_) {
		const GroupDataPtr &data = name_data.second;
		boost::unique_lock<boost::mutex> lock(data->mutex);
		if (data->timestamp + timeout < now) {
			ROS_DEBUG_STREAM_NAMED("timeouts",
			                       "getGroupAveragedContacts timed out " << data->timestamp + timeout << " < " << now);
			continue;  // ignore stalled groups
		}
		tactile_msgs::TactileContact contact;
		contact.name = name_data.first;  // group name
		contact.header.frame_id = data->group->frame();
		contact.header.stamp = data->timestamp;
		if (!data->group->average(contact)) {
			ROS_DEBUG_STREAM_NAMED("contacts", "getGroupAveragedContacts no contact for group of frame_id "
			                                       << contact.header.frame_id);
			continue;  // ignore not contacted groups
		}

		// insert all contacts of the group into result array
		contacts.contacts.push_back(contact);
	}
	return contacts;
}

}  // namespace tactile
