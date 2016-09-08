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

namespace urdf {
namespace tactile {

inline const urdf::tactile::TactileSensor&
tactile_sensor_cast(const urdf::Sensor &sensor) {
  return dynamic_cast<const urdf::tactile::TactileSensor&>(*sensor.sensor_);
}
inline urdf::tactile::TactileSensor&
tactile_sensor_cast(urdf::Sensor &sensor) {
  return dynamic_cast<urdf::tactile::TactileSensor&>(*sensor.sensor_);
}

inline TactileSensorSharedPtr
tactile_sensor_cast(const urdf::SensorBaseSharedPtr& sensor) {
  return boost::dynamic_pointer_cast<urdf::tactile::TactileSensor>(sensor);
}
inline TactileSensorConstSharedPtr
tactile_sensor_cast(const urdf::SensorBaseConstSharedPtr& sensor) {
  return boost::dynamic_pointer_cast<const urdf::tactile::TactileSensor>(sensor);
}

inline TactileSensorSharedPtr
tactile_sensor_cast(const urdf::SensorSharedPtr& sensor) {
  return tactile_sensor_cast(sensor->sensor_);
}
inline TactileSensorConstSharedPtr
tactile_sensor_cast(const urdf::SensorConstSharedPtr& sensor) {
  return tactile_sensor_cast(sensor->sensor_);
}

} // end namespace tactile
} // end namespace urdf
