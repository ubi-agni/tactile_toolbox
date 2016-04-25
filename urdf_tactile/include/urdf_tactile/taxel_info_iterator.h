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

namespace urdf {
namespace tactile {

/// common interface class for TaxelInfoIterator for vector of taxels or array
class TaxelInfoIteratorI;
typedef boost::shared_ptr<TaxelInfoIteratorI> TaxelInfoIteratorIPtr;

/// common iterator for vector of taxels or taxel array
class TaxelInfoIterator
{
  TaxelInfoIteratorIPtr impl_;  //! iterator
  TaxelInfo info_;              //! lazily evaluated
  mutable bool valid_;          //! is info valid?

public:
  static TaxelInfoIterator begin(const urdf::SensorConstSharedPtr &sensor);
  static TaxelInfoIterator end(const urdf::SensorConstSharedPtr &sensor);

  TaxelInfoIterator() : impl_(0), valid_(false) {}
  TaxelInfoIterator(const TaxelInfoIterator &other);
  ~TaxelInfoIterator();
  TaxelInfoIterator &operator=(const TaxelInfoIterator &other);

  TaxelInfoIterator& operator++();
  TaxelInfoIterator operator++(int);

  TaxelInfoIterator& operator--();
  TaxelInfoIterator operator--(int);

  bool operator==(const TaxelInfoIterator& other) const;
  bool operator!=(const TaxelInfoIterator& other) const {return !(*this == other);}

  const TaxelInfo& operator*() const { validate(); return info_; }
  const TaxelInfo* operator->() const { validate(); return &info_; }

  /// allow implicit type conversion to TaxelInfoIteratorIPtr
  operator TaxelInfoIteratorIPtr();

private:
  explicit TaxelInfoIterator(TaxelInfoIteratorIPtr impl_, bool valid);
  void validate() const;
};

size_t index(const TaxelInfoIteratorIPtr &it);

} // end namespace tactile
} // end namespace urdf
