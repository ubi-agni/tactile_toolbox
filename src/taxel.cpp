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
#include "taxel.h"
#include <Eigen/Geometry>

namespace tactile {

Taxel::Taxel(const urdf::Pose &sensor_frame, const urdf::Pose &taxel_frame)
   : weight(0)
{
	Eigen::Vector3d sensor_translation(sensor_frame.position.x,
	                                   sensor_frame.position.y,
	                                   sensor_frame.position.z);
	Eigen::Quaterniond sensor_rotation(sensor_frame.rotation.w,
	                                   sensor_frame.rotation.x,
	                                   sensor_frame.rotation.y,
	                                   sensor_frame.rotation.z);

	Eigen::Vector3d taxel_translation(taxel_frame.position.x,
	                                  taxel_frame.position.y,
	                                  taxel_frame.position.z);
	Eigen::Quaterniond taxel_rotation(taxel_frame.rotation.w,
	                                  taxel_frame.rotation.x,
	                                  taxel_frame.rotation.y,
	                                  taxel_frame.rotation.z);

	Eigen::Transform<double, 3, Eigen::AffineCompact> origin
	      = Eigen::Translation3d(sensor_translation) * sensor_rotation *
	        Eigen::Translation3d(taxel_translation) * taxel_rotation;
	position = origin.translation();
	normal = origin.affine().col(2); // normal is along z-axis
}

} // namespace tactile
