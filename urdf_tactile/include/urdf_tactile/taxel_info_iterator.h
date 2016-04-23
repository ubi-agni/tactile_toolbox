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

#include <urdf_tactile/tactile.h>
#include "taxel_info.h"
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/bind.hpp>

namespace tactile {

/// class to iterator over all taxels in a TactileSensor providing the corresponding TaxelInfo
template<typename Iterator>
class TaxelInfoIterator : public boost::transform_iterator<
    boost::function<TaxelInfo(const typename std::iterator_traits<Iterator>::reference)>,
    Iterator
>
{
public:
  typedef boost::function<TaxelInfo(const typename std::iterator_traits<Iterator>::reference)> Function;

  static TaxelInfoIterator begin(const urdf::SensorConstSharedPtr &sensor);
  static TaxelInfoIterator end(const urdf::SensorConstSharedPtr &sensor);

protected:
  explicit TaxelInfoIterator(const urdf::SensorConstSharedPtr &sensor, const Iterator &it);
  const TaxelInfo& transform(typename std::iterator_traits<Iterator>::reference value);

private:
  urdf::SensorConstSharedPtr sensor;
  TaxelInfo info;
};

// convenience typedefs
typedef TaxelInfoIterator<std::vector<urdf::tactile::TactileTaxelSharedPtr>::const_iterator> TaxelVectorIterator;
typedef TaxelInfoIterator<boost::counting_iterator<size_t> > TaxelArrayIterator;


const urdf::tactile::TactileSensor&
tactile_sensor_cast(const urdf::Sensor &sensor) {
  return dynamic_cast<const urdf::tactile::TactileSensor&>(*sensor.sensor_);
}

}
