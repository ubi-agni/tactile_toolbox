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

/* Author: Robert Haschke */

#include <urdf_tactile/sensor.h>
#include <urdf_tactile/sort.h>
#include <urdf_tactile/cast.h>
#include <memory>

#include <urdf_tactile/taxel_info_iterator.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace urdf {
namespace tactile {

enum SortKey
{
	BY_GROUP,
	BY_CHANNEL
};
template <SortKey key>
const std::string &get(const urdf::SensorConstSharedPtr &sensor);
template <>
const std::string &get<BY_GROUP>(const urdf::SensorConstSharedPtr &sensor)
{
	return tactile_sensor_cast(*sensor).group_;
}
template <>
const std::string &get<BY_CHANNEL>(const urdf::SensorConstSharedPtr &sensor)
{
	return tactile_sensor_cast(*sensor).channel_;
}

template <typename Result>
Sensors &getOrInsertEntry(Result &result, const std::string &name);
template <>
Sensors &getOrInsertEntry<SensorsMap>(SensorsMap &result, const std::string &name)
{
	return result.insert(std::make_pair(name, Sensors())).first->second;
}
template <>
Sensors &getOrInsertEntry<SensorsTree>(SensorsTree &parent, const std::string &name)
{
	std::vector<std::string> names;
	SensorsTree &node = parent;
	boost::algorithm::split(names, name, boost::algorithm::is_any_of("/"), boost::token_compress_on);
	for (const auto &name : names) {
		node = node.children.insert(std::make_pair(name, SensorsTree())).first->second;
	}
	return node;
}

template <typename Result, SortKey key>
Result sort(const SensorMap &sensors)
{
	Result result;

	for (auto it = sensors.begin(), end = sensors.end(); it != end; ++it) {
		TactileSensorSharedPtr tactile = tactile_sensor_cast(it->second);
		if (!tactile)
			continue;  // some other sensor than tactile

		Sensors &g = getOrInsertEntry<Result>(result, get<key>(it->second));
		if (tactile->array_)
			g.arrays.push_back(it);
		else if (!tactile->taxels_.empty())
			g.taxels.push_back(it);
	}
	return result;
}

SensorsMap sortByGroups(const SensorMap &sensors)
{
	return sort<SensorsMap, BY_GROUP>(sensors);
}
SensorsTree sortByGroupsHierarchical(const SensorMap &sensors)
{
	return sort<SensorsTree, BY_GROUP>(sensors);
}
SensorsMap sortByChannels(const SensorMap &sensors)
{
	return sort<SensorsMap, BY_CHANNEL>(sensors);
}

/******************************************************************************
 * retrieving taxels
 ******************************************************************************/
taxel_list &getTaxels(const iterator_list &sensors, taxel_list &target)
{
	for (auto sensor : sensors) {
		auto taxels = TaxelInfoIterable(sensor->second);
		for (auto taxel = taxels.begin(), end = taxels.end(); taxel != end; ++taxel) {
			target.push_back(taxel);
		}
	}
	return target;
}

taxel_list &getTaxels(const Sensors &sensors, taxel_list &target)
{
	getTaxels(sensors.taxels, target);
	getTaxels(sensors.arrays, target);
	return target;
}

TaxelsMap &getTaxels(const SensorsMap &sensors, TaxelsMap &target)
{
	for (const auto &sensor : sensors) {
		auto tgt = target.insert(std::make_pair(sensor.first, TaxelsMap::mapped_type())).first->second;
		getTaxels(sensor.second, tgt);
	}
	return target;
}

// common template for all type combinations
template <typename TargetType, typename SourceType>
TargetType getTaxels(const SourceType &source)
{
	TargetType target;
	getTaxels(source, target);
	return target;
}
taxel_list getTaxels(const iterator_list &sensors)
{
	return getTaxels<taxel_list, iterator_list>(sensors);
}
taxel_list getTaxels(const Sensors &sensors)
{
	return getTaxels<taxel_list, Sensors>(sensors);
}
TaxelsMap getTaxels(const SensorsMap &sensors)
{
	return getTaxels<TaxelsMap, SensorsMap>(sensors);
}

size_t maxIndex(const taxel_list &taxels)
{
	size_t m = 0;
	for (const auto &taxel : taxels) {
		m = std::max(m, index(taxel));
	}
	return m;
}

}  // end namespace tactile
}  // end namespace urdf
