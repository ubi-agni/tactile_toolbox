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

#include <urdf_sensor/types.h>
#include <urdf_tactile/tactile.h>
#include <memory>

namespace urdf {
namespace tactile {

class TaxelInfoIteratorI;
using TaxelInfoIteratorIPtr = std::shared_ptr<TaxelInfoIteratorI>;

struct TaxelInfo
{
	TaxelInfo() {}
	TaxelInfo(const TaxelInfoIteratorIPtr& it);

	std::string group;
	std::string link;

	std::string channel;  /// name of channel
	size_t idx;  /// index into channel of TactileState

	urdf::GeometrySharedPtr geometry;  /// geometry of taxel
	urdf::Pose geometry_origin;  /// geometry origin w.r.t. link frame

	urdf::Pose taxel_origin;  /// taxel origin w.r.t. link frame
	urdf::Vector3 position;  /// position of taxel_origin
	urdf::Vector3 normal;  /// z-axis of taxel_origin
};

}  // end namespace tactile
}  // end namespace urdf
