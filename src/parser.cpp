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
#include "urdf_tactile/tactile.h"
#include <urdf_parser/utils.h>
#include <urdf_parser/pose.h>
#include <urdf_parser/link.h>
#include <console_bridge/console.h>

namespace urdf {

/* specialization of parseAttribute(const char* value) for TactileArray::DataOrder */
template <>
tactile::TactileArray::DataOrder
parseAttribute<tactile::TactileArray::DataOrder>(const char* value)
{
  if (strcmp(value, "col-major") == 0) return tactile::TactileArray::COLUMNMAJOR;
  else if (strcmp(value, "row-major") == 0) return tactile::TactileArray::ROWMAJOR;
  else throw ParseError("invalid order, expecting 'col-major'' or 'row-major'");
}

/* specialization of parseAttribute(const char* value) for Vector2 */
template <>
tactile::Vector2<double> parseAttribute<tactile::Vector2<double> >(const char* value)
{
  std::vector<std::string> pieces;
  std::vector<double> xy;
  boost::split(pieces, value, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i){
    if (pieces[i] != ""){
      xy.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
    }
  }

  if (xy.size() != 2)
    throw ParseError("expecting 2, but found " + boost::lexical_cast<std::string>(xy.size()) + " elements");

  return tactile::Vector2<double>(xy[0], xy[1]);
}


namespace tactile {

bool parseTactileTaxel(TactileTaxel &taxel, TiXmlElement *config)
{
  taxel.clear();

  // taxel frame
  if (!parsePose(taxel.origin, config))
    return false;

  // Geometry
  taxel.geometry = urdf::parseGeometry(config->FirstChildElement("geometry"));
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
  }
  catch (const ParseError &e) {
    CONSOLE_BRIDGE_logError(e.what());
    return false;
  }
  return true;
}


SensorBaseSharedPtr TactileSensorParser::parse(TiXmlElement &config)
{
  TactileSensorSharedPtr tactile(new TactileSensor());
  // multiple Taxels (optional)
  for (TiXmlElement* taxel_xml = config.FirstChildElement("taxel"); taxel_xml; taxel_xml = taxel_xml->NextSiblingElement("taxel"))
  {
    TactileTaxelSharedPtr taxel;
    taxel.reset(new TactileTaxel());
    if (parseTactileTaxel(*taxel, taxel_xml))
    {
      tactile->taxels_.push_back(taxel);
    }
    else
    {
      taxel.reset();
      CONSOLE_BRIDGE_logError("Could not parse taxel element for tactile sensor");
      return TactileSensorSharedPtr();
    }
  }

  // a single array (optional)
  for (TiXmlElement* array_xml = config.FirstChildElement("array"); array_xml; array_xml = array_xml->NextSiblingElement("array"))
  {
    if (tactile->array_)
    {
      CONSOLE_BRIDGE_logWarn("Only a single array element is allowed for a tactile sensor");
      break; // only warn once
    }
    tactile->array_.reset(new TactileArray());
    if (!parseTactileArray(*tactile->array_, array_xml))
    {
      tactile->array_.reset();
      CONSOLE_BRIDGE_logError("Could not parse array element for tactile sensor");
      return TactileSensorSharedPtr();
    }
  }

  if (tactile->array_ && tactile->taxels_.size())
  {
    CONSOLE_BRIDGE_logWarn("Either an array or multiple taxel elements are allowed for a tactile sensor");
    tactile->array_.reset();
  }
  return tactile;
}

}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(urdf::tactile::TactileSensorParser, urdf::SensorParser)
