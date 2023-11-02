/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CITEC, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Guillaume Walck */

#pragma once

#include <urdf_model/pose.h>
#include <urdf_model/link.h>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace urdf {
namespace tactile {

template <typename T>
class Vector2
{
public:
	Vector2(T _x, T _y)
	{
		this->x = _x;
		this->y = _y;
	}
	Vector2() { this->clear(); }
	T x;
	T y;

	void clear() { this->x = this->y = 0; }

	Vector2<T> operator+(const Vector2<T> &vec) { return Vector2(this->x + vec.x, this->y + vec.y); }
};

/** A taxel describes the location (origin), geometry,
    and vector index in the TactileState message.
*/
class TactileTaxel
{
public:
	TactileTaxel() { this->clear(); }

	unsigned int idx;  /// index into TactileState
	Pose origin;  /// location of taxel w.r.t. sensor frame
	GeometrySharedPtr geometry;  /// geometry of taxel

	void clear()
	{
		this->idx = 0;
		this->origin.clear();
		this->geometry.reset();
	}
};

/** A tactile array is a rectangular layout of taxels of size row x col.
    The order specifies the ordering of taxels in the TactileState msg.
*/
class TactileArray
{
public:
	TactileArray() { this->clear(); }

	enum DataOrder
	{
		ROWMAJOR,
		COLUMNMAJOR
	};

	unsigned int rows;
	unsigned int cols;
	DataOrder order;
	Vector2<double> size;
	Vector2<double> spacing;
	Vector2<double> offset;

	void clear()
	{
		this->rows = 0;
		this->cols = 0;
		this->order = ROWMAJOR;
		this->size.clear();
		this->spacing.clear();
		this->offset.clear();
	}
};

using TactileTaxelSharedPtr = std::shared_ptr<TactileTaxel>;
using TactileArraySharedPtr = std::shared_ptr<TactileArray>;

class TactileSensor
{
public:
	TactileSensor() { TactileSensor::clear(); }

	std::string name_;  //< sensor name must be unique
	double update_rate_;  //< update rate in Hz
	std::string parent_link_;  //< name of parent link this sensor is attached to
	Pose origin_;  //< transform from parent frame sensor frame

	std::vector<TactileTaxelSharedPtr> taxels_;
	TactileArraySharedPtr array_;
	std::string channel_;
	std::string group_;

	void clear()
	{
		name_.clear();
		parent_link_.clear();
		origin_.clear();
		taxels_.clear();
		array_.reset();
		channel_.clear();
		group_.clear();
	}
};
URDF_TYPEDEF_CLASS_POINTER(TactileSensor);

using SensorMap = std::map<std::string, TactileSensorSharedPtr>;

}  // namespace tactile
}  // namespace urdf
