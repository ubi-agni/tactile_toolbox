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
#include "taxel.h"
#include <urdf_tactile/tactile.h>
#include <tactile_msgs/TactileContact.h>
#include <Eigen/Dense>
#include <map>
#include <vector>

namespace tactile {

class TaxelGroup;
typedef boost::shared_ptr<TaxelGroup> TaxelGroupPtr;

/// map from group names to group pointers (group name = parent link name)
typedef std::map<std::string, TaxelGroupPtr> TaxelGroupMap;

class TaxelGroup
{
public:
	typedef size_t index_t;

	/// mapping an index of a TactileState.msg's channel onto the index in taxels_
	typedef std::map<index_t, index_t> TaxelMapping;
	/// mapping from a channel name onto its TaxelMapping
	typedef std::map<std::string, TaxelMapping> SensorToTaxelMapping;

	/// load TaxelGroups from robot_description, identified by group name
	static TaxelGroupMap load(const std::string &desc_param);

	TaxelGroup(const std::string &frame);
	void addTaxels(const urdf::SensorConstSharedPtr &sensor);

	const std::string& frame() const {return frame_;}
	const std::vector<Taxel>& taxels() const {return taxels_;}
	size_t size() const {return taxels_.size();}
	const SensorToTaxelMapping& mappings() const {return mappings_;}

	template <typename Iterator>
	void update(const TaxelMapping &mapping, Iterator begin, Iterator end);
	bool all(std::vector<tactile_msgs::TactileContact> &contacts,
	         const tactile_msgs::TactileContact &contact_template);
	bool average(tactile_msgs::TactileContact &contact);

private:
	void addTaxel(const Taxel &taxel);

private:
	/// (link) frame this sensor group is attached to
	std::string frame_;
	/// taxels within this group
	std::vector<Taxel> taxels_;
	/// mapping from channel name onto its TaxelMapping
	SensorToTaxelMapping mappings_;
};

} // namespace tactile
