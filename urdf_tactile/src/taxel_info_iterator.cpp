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
#include <urdf_tactile/taxel_info_iterator.h>

using namespace urdf::tactile;

namespace tactile {

namespace {

void finish(TaxelInfo &info) {
  static const urdf::Vector3 zAxis(0,0,1);
  info.position = info.taxel_origin.position;
  info.normal = info.taxel_origin.rotation * zAxis;
}

urdf::Pose compose(const urdf::Pose &a, const urdf::Pose &b) {
  urdf::Pose r;
  r.rotation = a.rotation * b.rotation;
  urdf::Vector3 ab = a.rotation * b.position;
  r.position.x = a.position.x + ab.x;
  r.position.y = a.position.x + ab.y;
  r.position.z = a.position.x + ab.z;
  return r;
}

} // anonymous namespace

// Define how to retrieve begin() and end() iterators with a template trait
template<typename Iterator>
struct iterator_traits {
  /// retrieve begin iterator
  static Iterator begin(const urdf::Sensor &sensor);
  /// retrieve end iterator
  static Iterator end(const urdf::Sensor &sensor);
};

/******************************************************************************
 * generic TaxelInfoIterator implementation
 ******************************************************************************/
template <typename Iterator>
TaxelInfoIterator<Iterator> TaxelInfoIterator<Iterator>::begin(const urdf::SensorConstSharedPtr &sensor) {
  return TaxelInfoIterator(sensor, ::tactile::iterator_traits<Iterator>::begin(*sensor));
}

template <typename Iterator>
TaxelInfoIterator<Iterator> TaxelInfoIterator<Iterator>::end(const urdf::SensorConstSharedPtr &sensor) {
  return TaxelInfoIterator(sensor, ::tactile::iterator_traits<Iterator>::end(*sensor));
}

template <typename Iterator>
TaxelInfoIterator<Iterator>::TaxelInfoIterator(const urdf::SensorConstSharedPtr &sensor,
                                               const Iterator &it)
  : boost::transform_iterator<Function, Iterator>
    (it, boost::bind(&TaxelInfoIterator<Iterator>::transform, this, _1))
  , sensor(sensor)
{
  // initialize common properties of all taxels
  info.link = sensor->parent_link_;
  info.group = sensor->group_;
  info.channel = tactile_sensor_cast(*sensor).channel_;
  info.geometry_origin = sensor->origin_;
  // specific constructor part
  // TODO init();
}

/******************************************************************************
 * specializations for std::vector<TactileTaxelSharedPtr>::const_iterator
 ******************************************************************************/
typedef std::vector<TactileTaxelSharedPtr>::const_iterator VectorBaseIterator;
template <>
struct iterator_traits<VectorBaseIterator> {
  /// retrieve first iterator
  static std::vector<TactileTaxelSharedPtr>::const_iterator
  begin(const urdf::Sensor &sensor) {
    return tactile_sensor_cast(sensor).taxels_.begin();
  }

  /// retrieve last iterator
  static std::vector<TactileTaxelSharedPtr>::const_iterator
  end(const urdf::Sensor &sensor) {
    return tactile_sensor_cast(sensor).taxels_.end();
  }
};

// TODO: compute TaxelInfo only once during iterator update
template <>
const TaxelInfo& TaxelInfoIterator<VectorBaseIterator>::transform(const TactileTaxelSharedPtr &taxel) {
  // taxel-specific properties
  info.geometry = taxel->geometry;
  info.idx = taxel->idx;
  info.taxel_origin = compose(sensor->origin_, taxel->origin);

  finish(info);
  return info;
}

/******************************************************************************
 * specializations for boost::counting_iterator<size_t>
 ******************************************************************************/
typedef boost::counting_iterator<size_t> ArrayBaseIterator;
template <>
struct iterator_traits<ArrayBaseIterator> {
  /// retrieve first iterator
  static ArrayBaseIterator begin(const urdf::Sensor &sensor) {
    return ArrayBaseIterator(0);
  }

  /// retrieve last iterator
  static ArrayBaseIterator end(const urdf::Sensor &sensor) {
    const TactileSensor &tactile = tactile_sensor_cast(sensor);
    return ArrayBaseIterator(tactile.array_->rows * tactile.array_->cols);
  }
};

template <>
const TaxelInfo& TaxelInfoIterator<ArrayBaseIterator>::transform(const size_t& idx) {
  const TactileSensor &tactile = tactile_sensor_cast(*sensor);
  const TactileArray &array = *tactile.array_;

  // TODO: init()
  urdf::Box *b = new urdf::Box();
  b->dim = urdf::Vector3(array.size.x, array.size.y, 0);
  info.geometry.reset(b);

  // taxel-specific properties
  info.idx = idx;
  info.taxel_origin.rotation = info.geometry_origin.rotation;

  size_t row, col;
  if (array.order == TactileArray::ROWMAJOR) {
    row = idx / array.cols;
    col = idx % array.cols;
  } else {
    row = idx % array.rows;
    col = idx / array.rows;
  }
  info.taxel_origin.position.x = row * array.spacing.x - array.offset.x;
  info.taxel_origin.position.y = col * array.spacing.y - array.offset.y;
  info.taxel_origin.position.z = 0;

  finish(info);
  return info;
}

/******************************************************************************
 * explicit template instantiations
 ******************************************************************************/
template class TaxelInfoIterator<std::vector<TactileTaxelSharedPtr>::const_iterator>;
template class TaxelInfoIterator<boost::counting_iterator<size_t> >;

} // end namespace tactile
