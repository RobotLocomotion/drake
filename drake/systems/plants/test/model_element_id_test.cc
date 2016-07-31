#include <gtest/gtest.h>

#include "drake/systems/plants/model_element_id.h"

namespace drake {
namespace systems {
namespace plants {

// Tests the basic functionality of ModelElementId.
GTEST_TEST(ModelElementIdTest, BasicTest) {
  ModelElementId element_id_1("Foo_Instance", "Bar_Model", "Baz_Element", 1);
  EXPECT_EQ(element_id_1.get_instance_name(), "Foo_Instance");
  EXPECT_EQ(element_id_1.get_model_name(), "Bar_Model");
  EXPECT_EQ(element_id_1.get_element_name(), "Baz_Element");
  EXPECT_EQ(element_id_1.get_model_instance_id(), 1);
}

// Tests ability to compare two ModelElementId objects.
GTEST_TEST(ModelElementIdTest, CompareTest) {
  ModelElementId element_id_1("Foo_Instance", "Bar_Model", "Baz_Element", 1);
  ModelElementId element_id_2("Foo_Instance", "Bar_Model", "Baz_Element", 1);
  ModelElementId element_id_3("AS@#$AB", "Bar_Model", "Baz_Element", 2);
  ModelElementId element_id_4("Foo_Instance", "Q@#$@#!!@#", "Baz_Element", 3);
  ModelElementId element_id_5("Foo_Instance", "Bar_Model", "345234614", 4);

  EXPECT_EQ(element_id_1, element_id_1);
  EXPECT_EQ(element_id_1, element_id_2);
  EXPECT_NE(element_id_1, element_id_3);
  EXPECT_NE(element_id_1, element_id_4);
  EXPECT_NE(element_id_1, element_id_5);
}

}  // namespace plants
}  // namespace systems
}  // namespace drake
