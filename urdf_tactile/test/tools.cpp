#include "urdf_tactile/taxel_info_iterator.h"
#include "urdf_tactile/tactile.h"

using namespace urdf::tactile;

// the name of our test module
#define BOOST_TEST_MODULE URDF_TACTILE_TOOLS_TEST
// needed for automatic generation of the main()
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

urdf::SensorSharedPtr create_array(unsigned int rows, unsigned int cols,
                                   TactileArray::DataOrder order = TactileArray::ROWMAJOR)
{
  urdf::SensorSharedPtr s(new urdf::Sensor);
  s->name_ = "array";
  s->parent_link_ = "link";
  s->group_ = "group";

  TactileSensorSharedPtr tactile(new TactileSensor);
  s->sensor_ = tactile;
  tactile->channel_ = "channel";

  TactileArraySharedPtr array(new TactileArray);
  array->order = order;
  array->rows = rows;
  array->cols = cols;
  array->size = urdf::tactile::Vector2<double>(0.5,0.5);
  array->spacing = urdf::tactile::Vector2<double>(1,1);
  tactile->array_ = array;
  return s;
}

BOOST_AUTO_TEST_CASE(test_array_iterator)
{
  urdf::SensorSharedPtr sensor = create_array(5, 3);
  auto it = TaxelInfoIterator::begin(sensor);
  auto end = TaxelInfoIterator::end(sensor);
  urdf::GeometryConstSharedPtr geom = it->geometry;
  BOOST_ASSERT(geom);
  BOOST_ASSERT(it->idx == 0);

  auto begin = it;
  BOOST_ASSERT(begin->geometry == geom);
  BOOST_ASSERT(begin->idx == 0);

  for (size_t r=0; r < 5; ++r) {
    for (size_t c=0; c < 3; ++c, ++it) {
      TaxelInfo i = *it;
      BOOST_ASSERT(it->idx == 3*r + c);
      BOOST_ASSERT(it->taxel_origin.position.x == r);
      BOOST_ASSERT(it->taxel_origin.position.y == c);
      // all taxel should share the same geometry
      BOOST_ASSERT(it->geometry == geom);
    }
  }
  --end;
  BOOST_ASSERT(end->idx == 5*3-1);
  it = end;
  BOOST_ASSERT(end->idx == it->idx);
}

urdf::SensorSharedPtr create_taxels()
{
  urdf::SensorSharedPtr s(new urdf::Sensor);
  s->name_ = "taxels";
  s->parent_link_ = "link";
  s->group_ = "group";

  TactileSensorSharedPtr tactile(new TactileSensor);
  s->sensor_ = tactile;
  tactile->channel_ = "channel";

  TactileTaxelSharedPtr taxel;
  for (unsigned int i=0; i < 10; ++i) {
    taxel.reset(new TactileTaxel);
    taxel->idx = i;
    tactile->taxels_.push_back(taxel);
  }
  return s;
}
BOOST_AUTO_TEST_CASE(test_vector_iterator)
{
  urdf::SensorSharedPtr sensor = create_taxels();
  auto it = TaxelInfoIterator::begin(sensor);
  auto end = TaxelInfoIterator::end(sensor);
  for (size_t idx = 0; it != end; ++it, ++idx) {
    BOOST_ASSERT(it->idx == idx);
  }
}
