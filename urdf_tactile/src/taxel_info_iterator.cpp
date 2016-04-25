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

#include <urdf_tactile/taxel_info_iterator.h>
#include <urdf_tactile/cast.h>

namespace urdf {
namespace tactile {

namespace {

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

/******************************************************************************
 * common interface class TaxelInfoIteratorI for implementations
 ******************************************************************************/
class TaxelInfoIteratorI : public std::iterator<std::random_access_iterator_tag, TaxelInfo>
{
public:
  TaxelInfoIteratorI(const urdf::SensorConstSharedPtr &sensor) : sensor(sensor) {}
  virtual TaxelInfoIteratorIPtr clone() const = 0;
  virtual ~TaxelInfoIteratorI() {}

  virtual TaxelInfoIteratorI& operator++() = 0;
  virtual TaxelInfoIteratorI& operator--() = 0;
  virtual bool operator==(const TaxelInfoIteratorI& other) const = 0;

  virtual size_t index() const = 0;

  virtual void initInfo(TaxelInfo &info);
  virtual void updateInfo(TaxelInfo &info) = 0;
  void finishInfo(TaxelInfo &info);

  urdf::SensorConstSharedPtr sensor;
};

void TaxelInfoIteratorI::initInfo(TaxelInfo &info)
{
  // initialize common properties of all taxels
  info.link = sensor->parent_link_;
  info.group = sensor->group_;
  info.channel = tactile_sensor_cast(*sensor).channel_;
}

void TaxelInfoIteratorI::finishInfo(TaxelInfo& info) {
  static const urdf::Vector3 zAxis(0,0,1);
  info.position = info.taxel_origin.position;
  info.normal = info.taxel_origin.rotation * zAxis;
}

/******************************************************************************
 * generic TaxelInfoIterator:
 * - implements interface TaxelInfoIteratorI
 * - use template to implement common stuff
 ******************************************************************************/
template<typename Iterator>
class TaxelInfoIteratorBase : public TaxelInfoIteratorI
{
protected:
  Iterator it;

public:
  typedef Iterator iterator;

  explicit TaxelInfoIteratorBase(const urdf::SensorConstSharedPtr &sensor, Iterator it)
    : TaxelInfoIteratorI(sensor), it(it) {}

  TaxelInfoIteratorIPtr clone() const {return TaxelInfoIteratorIPtr(new TaxelInfoIteratorBase(*this));}
  TaxelInfoIteratorI& operator++() { ++it; return *this; }
  TaxelInfoIteratorI& operator--() { --it; return *this; }
  bool operator==(const TaxelInfoIteratorI& other) const {
    // if sensors are equal, iterator types are actually identical and we can safely static_cast
    return (sensor == other.sensor) && (it == static_cast<const TaxelInfoIteratorBase&>(other).it);
  }

  size_t index() const;
  void initInfo(TaxelInfo& info);
  void updateInfo(TaxelInfo& info);
};

/******************************************************************************
 * TaxelInfoIterator for vector of taxels
 ******************************************************************************/
typedef std::vector<TactileTaxelSharedPtr>::const_iterator VectorBaseIterator;
template <>
size_t TaxelInfoIteratorBase<VectorBaseIterator>::index() const {
  const TactileTaxel &taxel = **it;
  return taxel.idx;
}

template <>
void TaxelInfoIteratorBase<VectorBaseIterator>::initInfo(TaxelInfo& info) {
  TaxelInfoIteratorI::initInfo(info);
  info.geometry_origin = sensor->origin_;
}

template <>
void TaxelInfoIteratorBase<VectorBaseIterator>::updateInfo(TaxelInfo& info) {
  const TactileTaxel &taxel = **it;
  info.geometry = taxel.geometry;
  info.idx = taxel.idx;
  info.taxel_origin = compose(sensor->origin_, taxel.origin);
  finishInfo(info);
}

/******************************************************************************
 * TaxelInfoIterator for 2D taxel array
 ******************************************************************************/
typedef size_t ArrayBaseIterator;
template <>
size_t TaxelInfoIteratorBase<ArrayBaseIterator>::index() const {
  return it;
}

template <>
void TaxelInfoIteratorBase<ArrayBaseIterator>::initInfo(TaxelInfo& info) {
  TaxelInfoIteratorI::initInfo(info);
  info.geometry_origin = sensor->origin_;
  info.taxel_origin.rotation = info.geometry_origin.rotation;

  const TactileArray &array = *tactile_sensor_cast(*sensor).array_;
  urdf::Box *box = new urdf::Box();
  box->dim = urdf::Vector3(array.size.x, array.size.y, 0);
  info.geometry.reset(box);
}

template <>
void TaxelInfoIteratorBase<ArrayBaseIterator>::updateInfo(TaxelInfo& info) {
  const TactileArray &array = *tactile_sensor_cast(*sensor).array_;

  info.idx = it;

  size_t row, col;
  if (array.order == TactileArray::ROWMAJOR) {
    row = it / array.cols;
    col = it % array.cols;
  } else {
    row = it % array.rows;
    col = it / array.rows;
  }
  info.taxel_origin.position.x = row * array.spacing.x - array.offset.x;
  info.taxel_origin.position.y = col * array.spacing.y - array.offset.y;
  info.taxel_origin.position.z = 0;

  info.geometry_origin.position = const_cast<urdf::Vector3&>(sensor->origin_.position)
      + sensor->origin_.rotation * info.taxel_origin.position;
  finishInfo(info);
}


/******************************************************************************
 * implementation of wrapper class TaxelInfoIterator
 * We need to wrap a TaxelInfoIteratorI* to allow for cloning of derived classes.
 * http://www.ocoudert.com/blog/2010/07/07/how-to-write-abstract-iterators-in-c
 ******************************************************************************/
TaxelInfoIterator::TaxelInfoIterator(const TaxelInfoIterator &other)
  : impl_(other.impl_ ? other.impl_->clone() : NULL)
  , valid_(false)
{
  info_ = other.info_;
}

TaxelInfoIterator::~TaxelInfoIterator() {}

TaxelInfoIterator& TaxelInfoIterator::operator=(const TaxelInfoIterator& other) {
  if (impl_ != other.impl_) {
    impl_ = other.impl_->clone();
  }
  info_ = other.info_;
  valid_ = other.valid_;
  return *this;
}

TaxelInfoIterator &TaxelInfoIterator::operator++() {
  if (impl_) { ++(*impl_); valid_ = false; }
  return *this;
}

TaxelInfoIterator TaxelInfoIterator::operator++(int)
{
  TaxelInfoIterator tmp(*this);
  ++(*this);
  return tmp;
}

TaxelInfoIterator &TaxelInfoIterator::operator--() {
  if (impl_) { --(*impl_); valid_ = false; }
  return *this;
}

TaxelInfoIterator TaxelInfoIterator::operator--(int)
{
  TaxelInfoIterator tmp(*this);
  --(*this);
  return tmp;
}

bool TaxelInfoIterator::operator==(const TaxelInfoIterator &other) const {
  return (impl_ == other.impl_) || (impl_ && other.impl_ && *impl_ == *other.impl_);
}

TaxelInfoIterator::operator TaxelInfoIteratorIPtr() {
  // this is like a operator=() and thus should clone a new copy
  return TaxelInfoIteratorIPtr(impl_->clone());
}

TaxelInfoIterator::TaxelInfoIterator(TaxelInfoIteratorIPtr impl, bool valid)
  : impl_(impl), valid_(valid)
{
  // THIS SHOULD BE THE ONE AND ONLY PLACE WHERE initInfo() IS CALLED
  impl->initInfo(info_);
  if (valid) impl->updateInfo(info_);
}

void TaxelInfoIterator::validate() const
{
  if (!valid_) {
    assert(impl_);
    impl_->updateInfo(const_cast<TaxelInfo&>(info_));
    valid_ = true;
  }
}

TaxelInfoIterator TaxelInfoIterator::begin(const urdf::SensorConstSharedPtr &sensor)
{
  const TactileSensor &tactile = tactile_sensor_cast(*sensor);
  TaxelInfoIteratorIPtr impl;

  if (tactile.taxels_.size())
    impl.reset(new TaxelInfoIteratorBase<VectorBaseIterator>(sensor, tactile.taxels_.begin()));
  else
    impl.reset(new TaxelInfoIteratorBase<ArrayBaseIterator>(sensor, 0));
  return TaxelInfoIterator(impl, true);
}

TaxelInfoIterator TaxelInfoIterator::end(const urdf::SensorConstSharedPtr &sensor)
{
  const TactileSensor &tactile = tactile_sensor_cast(*sensor);
  TaxelInfoIteratorIPtr impl;

  if (tactile.taxels_.size())
    impl.reset(new TaxelInfoIteratorBase<VectorBaseIterator>(sensor, tactile.taxels_.end()));
  else
    impl.reset(new TaxelInfoIteratorBase<ArrayBaseIterator>(sensor, tactile.array_->rows * tactile.array_->cols));
  return TaxelInfoIterator(impl, false);
}

/******************************************************************************
 * direct access from TaxelInfoIteratorIPtr
 ******************************************************************************/
size_t index(const TaxelInfoIteratorIPtr &impl)
{
  return impl->index();
}

TaxelInfo::TaxelInfo(const TaxelInfoIteratorIPtr& it) {
  it->initInfo(*this);
  it->updateInfo(*this);
}

} // end namespace tactile
} // end namespace urdf
