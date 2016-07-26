#include <gtest/gtest.h>

#include "drake/systems/plants/element_id.h"

namespace drake {
namespace systems {
namespace plants {

// Tests the basic functionality of ElementId.
GTEST_TEST(ElementIdTest, BasicTest) {
  ElementId element_id_1("Foo_Instance", "Bar_Model", "Baz_Element", 1);
  EXPECT_EQ(element_id_1.get_instance_name(), "Foo_Instance");
  EXPECT_EQ(element_id_1.get_model_name(), "Bar_Model");
  EXPECT_EQ(element_id_1.get_element_name(), "Baz_Element");
  EXPECT_EQ(element_id_1.get_model_id(), 1);
}

// Tests ability to compare two ElementId objects.
GTEST_TEST(ElementIdTest, CompareTest) {
  ElementId element_id_1("Foo_Instance", "Bar_Model", "Baz_Element", 1);
  ElementId element_id_2("Foo_Instance", "Bar_Model", "Baz_Element", 1);
  ElementId element_id_3("AS@#$AB", "Bar_Model", "Baz_Element", 2);
  ElementId element_id_4("Foo_Instance", "Q@#$@#!!@#", "Baz_Element", 3);
  ElementId element_id_5("Foo_Instance", "Bar_Model", "345234614", 4);

  EXPECT_EQ(element_id_1, element_id_1);
  EXPECT_EQ(element_id_1, element_id_2);
  EXPECT_NE(element_id_1, element_id_3);
  EXPECT_NE(element_id_1, element_id_4);
  EXPECT_NE(element_id_1, element_id_5);
}

}  // namespace plants
}  // namespace systems
}  // namespace drake
