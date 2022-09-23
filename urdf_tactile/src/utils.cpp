/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Wim Meeussen, functions copied from urdfdom/urdf_parser/link.cpp */

#include "utils.h"
#include <urdf_model/link.h>
#include <console_bridge/console.h>

namespace {
using namespace urdf;

bool parseSphere(Sphere &s, TiXmlElement *c)
{
	s.clear();

	s.type = Geometry::SPHERE;
	if (!c->Attribute("radius")) {
		CONSOLE_BRIDGE_logError("Sphere shape must have a radius attribute");
		return false;
	}

	try {
		s.radius = strToDouble(c->Attribute("radius"));
	} catch (std::runtime_error &) {
		std::stringstream stm;
		stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
		CONSOLE_BRIDGE_logError(stm.str().c_str());
		return false;
	}

	return true;
}

bool parseBox(Box &b, TiXmlElement *c)
{
	b.clear();

	b.type = Geometry::BOX;
	if (!c->Attribute("size")) {
		CONSOLE_BRIDGE_logError("Box shape has no size attribute");
		return false;
	}
	try {
		b.dim.init(c->Attribute("size"));
	} catch (ParseError &e) {
		b.dim.clear();
		CONSOLE_BRIDGE_logError(e.what());
		return false;
	}
	return true;
}

bool parseCylinder(Cylinder &y, TiXmlElement *c)
{
	y.clear();

	y.type = Geometry::CYLINDER;
	if (!c->Attribute("length") || !c->Attribute("radius")) {
		CONSOLE_BRIDGE_logError("Cylinder shape must have both length and radius attributes");
		return false;
	}

	try {
		y.length = strToDouble(c->Attribute("length"));
	} catch (std::runtime_error &) {
		std::stringstream stm;
		stm << "length [" << c->Attribute("length") << "] is not a valid float";
		CONSOLE_BRIDGE_logError(stm.str().c_str());
		return false;
	}

	try {
		y.radius = strToDouble(c->Attribute("radius"));
	} catch (std::runtime_error &) {
		std::stringstream stm;
		stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
		CONSOLE_BRIDGE_logError(stm.str().c_str());
		return false;
	}

	return true;
}

bool parseMesh(Mesh &m, TiXmlElement *c)
{
	m.clear();

	m.type = Geometry::MESH;
	if (!c->Attribute("filename")) {
		CONSOLE_BRIDGE_logError("Mesh must contain a filename attribute");
		return false;
	}

	m.filename = c->Attribute("filename");

	if (c->Attribute("scale")) {
		try {
			m.scale.init(c->Attribute("scale"));
		} catch (ParseError &e) {
			m.scale.clear();
			CONSOLE_BRIDGE_logError("Mesh scale was specified, but could not be parsed: %s", e.what());
			return false;
		}
	} else {
		m.scale.x = m.scale.y = m.scale.z = 1;
	}
	return true;
}
}  // namespace

namespace urdf {
namespace tactile {

GeometrySharedPtr parseGeometry(TiXmlElement *g)
{
	GeometrySharedPtr geom;
	if (!g)
		return geom;

	TiXmlElement *shape = g->FirstChildElement();
	if (!shape) {
		CONSOLE_BRIDGE_logError("Geometry tag contains no child element.");
		return geom;
	}

	std::string type_name = shape->ValueStr();
	if (type_name == "sphere") {
		Sphere *s = new Sphere();
		geom.reset(s);
		if (parseSphere(*s, shape))
			return geom;
	} else if (type_name == "box") {
		Box *b = new Box();
		geom.reset(b);
		if (parseBox(*b, shape))
			return geom;
	} else if (type_name == "cylinder") {
		Cylinder *c = new Cylinder();
		geom.reset(c);
		if (parseCylinder(*c, shape))
			return geom;
	} else if (type_name == "mesh") {
		Mesh *m = new Mesh();
		geom.reset(m);
		if (parseMesh(*m, shape))
			return geom;
	} else {
		CONSOLE_BRIDGE_logError("Unknown geometry type '%s'", type_name.c_str());
		return geom;
	}

	return GeometrySharedPtr();
}

}  // namespace tactile
}  // namespace urdf
