#include "urdf_tactile/parser.h"
#include <fstream>

using namespace urdf::tactile;

// the name of our test module
#define BOOST_TEST_MODULE URDF_UNIT_TEST
// needed for automatic generation of the main()
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

static std::shared_ptr<tinyxml2::XMLDocument> loadFromFile(const std::string &path)
{
	std::ifstream stream(path.c_str());
	BOOST_REQUIRE_MESSAGE(stream, "failed to open file: " + path);

	std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());

	std::shared_ptr<tinyxml2::XMLDocument> xml_doc(new tinyxml2::XMLDocument());
	BOOST_REQUIRE(xml_doc->Parse(xml_str.c_str()) == tinyxml2::XML_SUCCESS);

	return xml_doc;
}

void check_sensor_map(const SensorMap &sensors)
{
	TactileSensorSharedPtr tactile = sensors.at("tactile_taxel_sensor");
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->taxels_.size() == 2);

	tactile = sensors.at("tactile_array_sensor");
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_);
	BOOST_CHECK(tactile->array_->order == TactileArray::ROWMAJOR);
	BOOST_CHECK(tactile->array_->spacing.x == tactile->array_->size.x);
	BOOST_CHECK(tactile->array_->spacing.y == tactile->array_->size.y);
}

BOOST_AUTO_TEST_CASE(test_pluginlib_parsing)
{
	check_sensor_map(parseSensorsFromFile("tactile.urdf"));
}

void change_and_check_order_mode(const TactileSensorParser &parser, tinyxml2::XMLElement &tactile_xml,
                                 const char *pcMode, TactileArray::DataOrder mode)
{
	tinyxml2::XMLElement *array_xml = tactile_xml.FirstChildElement("array");
	array_xml->SetAttribute("order", pcMode);
	TactileSensorSharedPtr tactile(parser.parse(tactile_xml));
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_->order == mode);
}

BOOST_AUTO_TEST_CASE(test_tactile_array)
{
	TactileSensorParser parser;
	std::shared_ptr<tinyxml2::XMLDocument> root = loadFromFile("tactile.urdf");
	BOOST_REQUIRE(root);

	tinyxml2::XMLElement *tactile_xml = root->RootElement()->FirstChildElement("sensor")->FirstChildElement("tactile");
	BOOST_REQUIRE(tactile_xml);

	TactileSensorSharedPtr tactile(parser.parse(*tactile_xml));
	BOOST_REQUIRE(tactile);

	BOOST_CHECK(tactile->taxels_.empty());
	BOOST_REQUIRE(tactile->array_);

	BOOST_CHECK(tactile->array_->order == TactileArray::ROWMAJOR);
	change_and_check_order_mode(parser, *tactile_xml, "row-major", TactileArray::ROWMAJOR);
	change_and_check_order_mode(parser, *tactile_xml, "col-major", TactileArray::COLUMNMAJOR);

	tinyxml2::XMLElement *array_xml = tactile_xml->FirstChildElement("array");

	// missing spacing attribute defaults to size
	array_xml->DeleteAttribute("spacing");
	tactile.reset(parser.parse(*tactile_xml));
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_->spacing.x == tactile->array_->size.x);
	BOOST_CHECK(tactile->array_->spacing.y == tactile->array_->size.y);

	// missing offset attribute defaults to zero
	array_xml->DeleteAttribute("offset");
	tactile.reset(parser.parse(*tactile_xml));
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_->offset.x == 0);
	BOOST_CHECK(tactile->array_->offset.y == 0);
}
