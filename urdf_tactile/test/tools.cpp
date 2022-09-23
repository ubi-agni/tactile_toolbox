#include "urdf_tactile/sensor.h"
#include "urdf_tactile/taxel_info_iterator.h"
#include "urdf_tactile/sort.h"

using namespace urdf::tactile;

// the name of our test module
#define BOOST_TEST_MODULE URDF_TACTILE_TOOLS_TEST
// needed for automatic generation of the main()
#define BOOST_TEST_DYN_LINK
// enable BOOST_ASSERT
#undef NDEBUG

#include <boost/test/unit_test.hpp>

urdf::SensorSharedPtr create_array(unsigned int rows, unsigned int cols,
                                   TactileArray::DataOrder order = TactileArray::ROWMAJOR)
{
	urdf::SensorSharedPtr s(new urdf::Sensor);
	s->name_ = "array";
	s->parent_link_ = "link";

	TactileSensorSharedPtr tactile(new TactileSensor);
	s->sensor_ = tactile;
	tactile->channel_ = "channel";
	tactile->group_ = "array";

	TactileArraySharedPtr array(new TactileArray);
	array->order = order;
	array->rows = rows;
	array->cols = cols;
	array->size = urdf::tactile::Vector2<double>(0.5, 0.5);
	array->spacing = urdf::tactile::Vector2<double>(1, 1);
	tactile->array_ = array;
	return s;
}

void test_array_iterator(TactileArray::DataOrder order)
{
	auto taxels = TaxelInfoIterable(create_array(5, 3, order));
	auto it = taxels.begin();
	auto end = taxels.end();
	urdf::GeometryConstSharedPtr geom = it->geometry;
	BOOST_ASSERT(geom);
	BOOST_ASSERT(it->idx == 0);
	BOOST_ASSERT(it != end);

	auto begin = it;
	BOOST_ASSERT(begin->geometry == geom);
	BOOST_ASSERT(begin->idx == 0);

	for (size_t r = 0; r < 5; ++r) {
		for (size_t c = 0; c < 3; ++c, ++it) {
			BOOST_ASSERT(it != end);
			TaxelInfo i = *it;
			size_t idx = (order == TactileArray::ROWMAJOR) ? 3 * r + c : 5 * c + r;
			BOOST_ASSERT(it->idx == idx);
			BOOST_ASSERT(index(it) == idx);
			BOOST_ASSERT(it->taxel_origin.position.x == r);
			BOOST_ASSERT(it->taxel_origin.position.y == c);
			// all taxel should share the same geometry
			BOOST_ASSERT(it->geometry == geom);
		}
	}
	BOOST_ASSERT(it == end);
	--end;
	BOOST_ASSERT(end->idx == 5 * 3 - 1);
	it = end;
	BOOST_ASSERT(end->idx == it->idx);
}
BOOST_AUTO_TEST_CASE(test_array_iterator_rowmajor)
{
	test_array_iterator(TactileArray::ROWMAJOR);
}
BOOST_AUTO_TEST_CASE(test_array_iterator_colmajor)
{
	test_array_iterator(TactileArray::ROWMAJOR);
}

urdf::SensorSharedPtr create_taxels(unsigned int num = 10)
{
	urdf::SensorSharedPtr s(new urdf::Sensor);
	s->name_ = "taxels";
	s->parent_link_ = "link";

	TactileSensorSharedPtr tactile(new TactileSensor);
	s->sensor_ = tactile;
	tactile->channel_ = "channel";
	tactile->group_ = "taxels";

	TactileTaxelSharedPtr taxel;
	for (unsigned int i = 0; i < num; ++i) {
		taxel.reset(new TactileTaxel);
		taxel->idx = i;
		tactile->taxels_.push_back(taxel);
	}
	return s;
}
BOOST_AUTO_TEST_CASE(test_vector_iterator)
{
	auto taxels = TaxelInfoIterable(create_taxels());
	auto it = taxels.begin();
	auto end = taxels.end();
	BOOST_ASSERT(it != end);
	for (size_t idx = 0; it != end; ++it, ++idx) {
		BOOST_ASSERT(it != end);
		BOOST_ASSERT(it->idx == idx);
	}
	BOOST_ASSERT(it == end);
}

void test_grouping(unsigned int taxels, unsigned int rows, unsigned int cols)
{
	urdf::SensorMap sensors;
	sensors["taxels"] = create_taxels(taxels);
	sensors["array"] = create_array(rows, cols);
	auto groups = sortByGroups(sensors);
	BOOST_ASSERT(groups.size() == 2);
	auto t = getTaxels(groups["taxels"]);
	BOOST_ASSERT(index(t[0]) == 0);
	BOOST_ASSERT(getTaxels(groups["taxels"]).size() == taxels);
	BOOST_ASSERT(getTaxels(groups["array"]).size() == rows * cols);
	BOOST_ASSERT(maxIndex(getTaxels(sortByChannels(sensors)["channel"])) == std::max(taxels, rows * cols) - 1);
}

BOOST_AUTO_TEST_CASE(test_groupings)
{
	test_grouping(10, 5, 3);
	test_grouping(10, 2, 5);
	test_grouping(15, 2, 5);
}
