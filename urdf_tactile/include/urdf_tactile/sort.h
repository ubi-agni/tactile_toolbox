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

#include <urdf_parser/sensor_parser.h>

namespace urdf {
namespace tactile {

/******************************************************************************
 * sort sensors by group or channel name
 ******************************************************************************/
typedef urdf::SensorMap::const_iterator iterator;
typedef std::deque<iterator> iterator_list;
struct Sensors {
  // sensors that define a vector of taxels
  iterator_list taxels;
  // sensors that define a 2D taxel array
  iterator_list arrays;
};
typedef std::map<std::string, Sensors> SensorsMap;

struct SensorsTree : Sensors {
  typedef std::map<std::string, SensorsTree> map_type;
  map_type children;
};

SensorsMap  sortByGroups(const urdf::SensorMap &sensors);
SensorsTree sortByGroupsHierarchical(const urdf::SensorMap &sensors);
SensorsMap  sortByChannels(const urdf::SensorMap &sensors);


/******************************************************************************
 * retrieving taxels
 ******************************************************************************/
class TaxelInfoIteratorI;
typedef boost::shared_ptr<TaxelInfoIteratorI> TaxelInfoIteratorIPtr;
typedef std::deque<TaxelInfoIteratorIPtr> taxel_list;
typedef std::map<std::string, taxel_list> TaxelsMap;
/// retrieve all taxels
taxel_list& getTaxels(const iterator_list &sensors, taxel_list &target);
/// retrieve all taxels
taxel_list& getTaxels(const Sensors &sensors, taxel_list &target);
/// retrieve all taxels
TaxelsMap& getTaxels(const SensorsMap &sensors, TaxelsMap &target);

/// versions without providing the target
taxel_list getTaxels(const iterator_list &sensors);
taxel_list getTaxels(const Sensors &sensors);
TaxelsMap getTaxels(const SensorsMap &sensors);

size_t maxIndex(const taxel_list &taxels);

} // end namespace tactile
} // end namespace urdf
