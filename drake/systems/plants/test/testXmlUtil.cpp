#include <list>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

using Eigen::Vector3d;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

GTEST_TEST(testXmlUtil, testPopulatePackageMap) {
  PackageMap package_map;
  populatePackageMap(package_map);

  std::list<std::string> expected_packages = {"drake", "test"};
  for (auto package : expected_packages) {
    EXPECT_TRUE(package_map.count(package))
        << std::string("Expected package not found in list: ") << package;
  }
}

// Tests the ability to load a three vector from a string. Evalautes both
// the case where the string describes a three vector and the case where
// the string describes a single scalar value.
GTEST_TEST(test_parse_three_vector, from_string) {
  const char* three_value_string = "1.1 2.2 3.3";
  const char* one_value_string = "4.4";

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 1.1, 2.2, 3.3;
  EXPECT_TRUE(ParseThreeVectorValue(three_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);

  expected_vector3d << 4.4, 4.4, 4.4;
  EXPECT_TRUE(ParseThreeVectorValue(one_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML whose first child
// contains a three vector.
GTEST_TEST(test_parse_three_vector, from_xml_value_child_vector) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>5.5 6.6 7.7</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 5.5, 6.6, 7.7;
  EXPECT_TRUE(ParseThreeVectorValue(node, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML whose first child
// contains a scalar value.
GTEST_TEST(test_parse_three_vector, from_xml_value_child_scalar) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>10.1</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 10.1, 10.1, 10.1;
  EXPECT_TRUE(ParseThreeVectorValue(node, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML whose grandchild
// contains a three vector.
GTEST_TEST(test_parse_three_vector, from_xml_value_grandchild_vector) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>\n"
      "  <bar>5.5 6.6 7.7</bar>\n"
      "</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 5.5, 6.6, 7.7;
  EXPECT_TRUE(ParseThreeVectorValue(node, "bar", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML whose grandchild
// contains a scalar value.
GTEST_TEST(test_parse_three_vector, from_xml_value_grandchild_scalar) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>\n"
      "  <bar>93.5</bar>\n"
      "</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 93.5, 93.5, 93.5;
  EXPECT_TRUE(ParseThreeVectorValue(node, "bar", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML attribute containing
// a three vector.
GTEST_TEST(test_parse_three_vector, from_xml_attribute_vector) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"1.2 3.4 5.6\" />";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("mesh");

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 1.2, 3.4, 5.6;
  EXPECT_TRUE(ParseThreeVectorAttribute(node, "scale", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML attribute containing
// a three vector.
GTEST_TEST(test_parse_three_vector, from_xml_attribute_scalar) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"9.9\" />";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("mesh");

  Vector3d parsed_vector3d;
  Vector3d expected_vector3d;
  expected_vector3d << 9.9, 9.9, 9.9;
  EXPECT_TRUE(ParseThreeVectorAttribute(node, "scale", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}
}
