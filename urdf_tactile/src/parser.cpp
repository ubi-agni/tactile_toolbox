/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CITEC / Bielefeld University
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
 *   * Neither the name of the author nor the names of its
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

/* Author: Robert Haschke */

#include "urdf_tactile/parser.h"
#include "utils.h"
#include <urdf_parser/urdf_parser.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <console_bridge/console.h>
#include <ros/ros.h>
#include <fstream>

namespace urdf {
namespace tactile {

/* specialization of parseAttribute(const char* value) for TactileArray::DataOrder */
template <>
tactile::TactileArray::DataOrder parseAttribute<tactile::TactileArray::DataOrder>(const char *value)
{
	if (strcmp(value, "col-major") == 0)
		return tactile::TactileArray::COLUMNMAJOR;
	else if (strcmp(value, "row-major") == 0)
		return tactile::TactileArray::ROWMAJOR;
	else
		throw ParseError("invalid order, expecting 'col-major'' or 'row-major'");
}

/* specialization of parseAttribute(const char* value) for Vector2 */
template <>
tactile::Vector2<double> parseAttribute<tactile::Vector2<double> >(const char *value)
{
	std::vector<std::string> pieces;
	std::vector<double> xy;
	boost::split(pieces, value, boost::is_any_of(" "));
	for (const auto &piece : pieces) {
		if (!piece.empty()) {
			xy.push_back(boost::lexical_cast<double>(piece.c_str()));
		}
	}

	if (xy.size() != 2)
		throw ParseError("expecting 2, but found " + boost::lexical_cast<std::string>(xy.size()) + " elements");

	return tactile::Vector2<double>(xy[0], xy[1]);
}

bool parseTactileTaxel(TactileTaxel &taxel, TiXmlElement *config)
{
	taxel.clear();

	// taxel frame
	if (!parsePose(taxel.origin, config))
		return false;

	// Geometry
	taxel.geometry = parseGeometry(config->FirstChildElement("geometry"));
	if (!taxel.geometry)
		return false;

	// Idx
	try {
		taxel.idx = parseAttribute<unsigned int>(*config, "idx");
	} catch (const ParseError &e) {
		CONSOLE_BRIDGE_logError(e.what());
		return false;
	}

	return true;
}

bool parseTactileArray(TactileArray &array, TiXmlElement *config)
{
	array.clear();
	try {
		array.rows = parseAttribute<unsigned int>(*config, "rows");
		array.cols = parseAttribute<unsigned int>(*config, "cols");

		array.size = parseAttribute<Vector2<double> >(*config, "size");
		array.spacing = parseAttribute<Vector2<double> >(*config, "spacing", &array.size);
		Vector2<double> origin;
		array.offset = parseAttribute<Vector2<double> >(*config, "offset", &origin);

		TactileArray::DataOrder order = TactileArray::ROWMAJOR;
		array.order = parseAttribute<TactileArray::DataOrder>(*config, "order", &order);
	} catch (const ParseError &e) {
		CONSOLE_BRIDGE_logError(e.what());
		return false;
	}
	return true;
}

TactileSensor *TactileSensorParser::parse(TiXmlElement &config) const
{
	TiXmlElement *parent = config.Parent()->ToElement()->FirstChildElement("parent");
	if (!parent) {
		CONSOLE_BRIDGE_logError("No <parent> tag given for the sensor.");
		return nullptr;
	}

	auto tactile = std::make_unique<TactileSensor>();
	tactile->name_ = parseAttribute<std::string>(*config.Parent()->ToElement(), "name");
	tactile->parent_link_ = parseAttribute<std::string>(*parent, "link");
	if (TiXmlElement *o = config.Parent()->ToElement()->FirstChildElement("origin")) {
		if (!parsePose(tactile->origin_, o))
			return nullptr;
	}

	tactile->channel_ = parseAttribute<std::string>(config, "channel");
	tactile->group_ = parseAttribute<std::string>(*config.Parent()->ToElement(), "group");

	// multiple Taxels (optional)
	for (TiXmlElement *taxel_xml = config.FirstChildElement("taxel"); taxel_xml;
	     taxel_xml = taxel_xml->NextSiblingElement("taxel")) {
		TactileTaxelSharedPtr taxel;
		taxel.reset(new TactileTaxel());
		if (parseTactileTaxel(*taxel, taxel_xml)) {
			tactile->taxels_.push_back(taxel);
		} else {
			CONSOLE_BRIDGE_logError("Could not parse taxel element for tactile sensor");
			return nullptr;
		}
	}

	// a single array (optional)
	for (TiXmlElement *array_xml = config.FirstChildElement("array"); array_xml;
	     array_xml = array_xml->NextSiblingElement("array")) {
		if (tactile->array_) {
			CONSOLE_BRIDGE_logWarn("Only a single array element is allowed for a tactile sensor");
			break;  // only warn once
		}
		tactile->array_.reset(new TactileArray());
		if (!parseTactileArray(*tactile->array_, array_xml)) {
			CONSOLE_BRIDGE_logError("Could not parse array element for tactile sensor");
			return nullptr;
		}
	}

	if (tactile->array_ && !tactile->taxels_.empty()) {
		CONSOLE_BRIDGE_logWarn("Either an array or multiple taxel elements are allowed for a tactile sensor");
		tactile->array_.reset();
	}
	return tactile.release();
}

SensorMap parseSensorsFromFile(const std::string &filename)
{
	SensorMap result;
	std::ifstream stream(filename.c_str());
	if (!stream.is_open()) {
		throw std::runtime_error("Could not open file [" + filename + "] for parsing.");
	}

	std::string xml_string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	return parseSensors(xml_string);
}

SensorMap parseSensorsFromParam(const std::string &param)
{
	ros::NodeHandle nh;
	std::string xml_string;

	// gets the location of the robot description on the parameter server
	std::string full_param;
	if (!nh.searchParam(param, full_param)) {
		throw std::runtime_error("Could not find parameter " + param + " on parameter server");
	}

	// read the robot description from the parameter server
	if (!nh.getParam(full_param, xml_string)) {
		throw std::runtime_error("Could not read parameter " + param + " on parameter server");
	}
	return parseSensors(xml_string);
}

SensorMap parseSensors(const std::string &xml_string)
{
	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
		throw std::runtime_error(std::string("Could not parse the xml document: ") + xml_doc.ErrorDesc());
	return parseSensors(xml_doc);
}

SensorMap parseSensors(TiXmlDocument &urdf_xml)
{
	TiXmlElement *robot_xml = urdf_xml.FirstChildElement("robot");
	if (!robot_xml) {
		CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the URDF");
		return SensorMap();
	}

	TactileSensorParser parser;
	SensorMap results;
	// Get all sensor elements
	for (TiXmlElement *sensor_xml = robot_xml->FirstChildElement("sensor"); sensor_xml;
	     sensor_xml = sensor_xml->NextSiblingElement("sensor")) {
		if (TiXmlElement *tactile_xml = sensor_xml->FirstChildElement("tactile")) {
			if (TactileSensor *sensor = parser.parse(*tactile_xml)) {
				auto res = results.insert(make_pair(sensor->name_, sensor));
				if (!res.second)
					CONSOLE_BRIDGE_logWarn("Sensor '%s' is not unique. Ignoring consecutive ones.", sensor->name_.c_str());
				else
					CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new sensor '%s'", sensor->name_.c_str());
			} else {
				CONSOLE_BRIDGE_logError("failed to parse sensor element");
			}
		}
	}
	return results;
}

}  // namespace tactile
}  // namespace urdf
