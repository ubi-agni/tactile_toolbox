#include "urdf_tactile/sensor.h"
#include "urdf_tactile/cast.h"
#include <urdf/sensor.h>
#include <fstream>

using namespace urdf::tactile;

// the name of our test module
#define BOOST_TEST_MODULE URDF_UNIT_TEST
// needed for automatic generation of the main()
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

static std::shared_ptr<TiXmlDocument> loadFromFile(const std::string &path)
{
	std::ifstream stream(path.c_str());
	BOOST_REQUIRE_MESSAGE(stream, "failed to open file: " + path);

	std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());

	std::shared_ptr<TiXmlDocument> xml_doc(new TiXmlDocument());
	xml_doc->Parse(xml_str.c_str());
	BOOST_REQUIRE(!xml_doc->Error());

	return xml_doc;
}

void check_sensor_map(const urdf::SensorMap &sensors)
{
	TactileSensorSharedPtr tactile = urdf::getSensor<TactileSensor>("tactile_taxel_sensor", sensors);
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->taxels_.size() == 2);

	tactile = urdf::getSensor<TactileSensor>("tactile_array_sensor", sensors);
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_);
	BOOST_CHECK(tactile->array_->order == TactileArray::ROWMAJOR);
	BOOST_CHECK(tactile->array_->spacing.x == tactile->array_->size.x);
	BOOST_CHECK(tactile->array_->spacing.y == tactile->array_->size.y);
}

BOOST_AUTO_TEST_CASE(test_pluginlib_parsing)
{
	auto parsers = urdf::getSensorParser("tactile");
	check_sensor_map(urdf::parseSensorsFromFile("tactile.urdf", parsers));
}

void change_and_check_order_mode(const urdf::SensorParserSharedPtr &parser, TiXmlElement &tactile_xml,
                                 const char *pcMode, TactileArray::DataOrder mode)
{
	TiXmlElement *array_xml = tactile_xml.FirstChildElement("array");
	array_xml->SetAttribute("order", pcMode);
	TactileSensorSharedPtr tactile(tactile_sensor_cast(parser->parse(tactile_xml)));
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_->order == mode);
}

BOOST_AUTO_TEST_CASE(test_tactile_array)
{
	auto parsers = urdf::getSensorParser("tactile");
	std::shared_ptr<TiXmlDocument> root = loadFromFile("tactile.urdf");
	BOOST_REQUIRE(root);

	TiXmlElement *tactile_xml = root->RootElement()->FirstChildElement("sensor")->FirstChildElement("tactile");
	BOOST_REQUIRE(tactile_xml);

	urdf::SensorParserSharedPtr parser = parsers["tactile"];
	TactileSensorSharedPtr tactile(tactile_sensor_cast(parser->parse(*tactile_xml)));
	BOOST_REQUIRE(tactile);

	BOOST_CHECK(tactile->taxels_.empty());
	BOOST_REQUIRE(tactile->array_);

	BOOST_CHECK(tactile->array_->order == TactileArray::ROWMAJOR);
	change_and_check_order_mode(parser, *tactile_xml, "row-major", TactileArray::ROWMAJOR);
	change_and_check_order_mode(parser, *tactile_xml, "col-major", TactileArray::COLUMNMAJOR);

	TiXmlElement *array_xml = tactile_xml->FirstChildElement("array");

	// missing spacing attribute defaults to size
	array_xml->RemoveAttribute("spacing");
	tactile.reset(tactile_sensor_cast(parser->parse(*tactile_xml)));
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_->spacing.x == tactile->array_->size.x);
	BOOST_CHECK(tactile->array_->spacing.y == tactile->array_->size.y);

	// missing offset attribute defaults to zero
	array_xml->RemoveAttribute("offset");
	tactile.reset(tactile_sensor_cast(parser->parse(*tactile_xml)));
	BOOST_REQUIRE(tactile);
	BOOST_CHECK(tactile->array_->offset.x == 0);
	BOOST_CHECK(tactile->array_->offset.y == 0);
}
