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

// Tests the ability to load a three vector from a string. Evaluates both
// the case where the string describes a three vector and the case where
// the string describes a single scalar value.
GTEST_TEST(test_parse_three_vector, from_string) {
  const char* three_value_string = "1.1 2.2 3.3";
  const char* one_value_string = "4.4";

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 1.1, 2.2, 3.3;

  EXPECT_TRUE(ParseThreeVectorValue(three_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);

  expected_vector3d << 4.4, 4.4, 4.4;
  EXPECT_TRUE(ParseThreeVectorValue(one_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to gracefully handle the failure mode where the string
// containing the vector or scalar value is nullptr.
GTEST_TEST(test_parse_three_vector, string_input_null_failure_mode) {
  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  const char* null_string = nullptr;

  EXPECT_FALSE(ParseThreeVectorValue(null_string, &parsed_vector3d));
  EXPECT_THROW(ParseThreeVectorValue("0.0, 1.1, 2.2", nullptr),
               std::runtime_error);
  EXPECT_THROW(ParseThreeVectorValue(null_string, nullptr), std::runtime_error);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides an empty string as the vector.
GTEST_TEST(test_parse_three_vector, string_input_empty_string_failure_mode) {
  const char* empty_string = "";

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  EXPECT_THROW(ParseThreeVectorValue(empty_string, &parsed_vector3d),
               std::runtime_error);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides a two-vector.
GTEST_TEST(test_parse_three_vector, string_input_two_vector_failure_mode) {
  const char* two_value_string = "2.1 0.3";

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  EXPECT_THROW(ParseThreeVectorValue(two_value_string, &parsed_vector3d),
               std::runtime_error);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides a vector with more than three elements.
GTEST_TEST(test_parse_three_vector,
           string_input_more_than_three_vector_failure_mode) {
  const char* two_value_string = "2.1 0.3 4.9 10.2 11.3";

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  EXPECT_THROW(ParseThreeVectorValue(two_value_string, &parsed_vector3d),
               std::runtime_error);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides a vector with non-double-type values.
GTEST_TEST(test_parse_three_vector,
           string_input_non_double_vector_failure_mode) {
  const int num_value_strings = 6;
  const char* non_double_value_string[] = {
    "foo bar baz",
    "1.0 bar baz",
    "2.0 3.0 baz",
    "foo 4.0 4.1",
    "foo bar 4.1",
    "9.2 bar 4.1"
  };

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  for (int ii = 0; ii < num_value_strings; ii++) {
    EXPECT_THROW(
        ParseThreeVectorValue(non_double_value_string[ii], &parsed_vector3d),
        std::invalid_argument);
  }
}

// Tests the ability to load a three vector from an XML whose first child
// contains a three vector.
GTEST_TEST(test_parse_three_vector, node_input_vector) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>5.5 6.6 7.7</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 5.5, 6.6, 7.7;

  EXPECT_TRUE(ParseThreeVectorValue(node, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML whose first child
// contains a scalar value.
GTEST_TEST(test_parse_three_vector, node_input_scalar) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>10.1</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 10.1, 10.1, 10.1;

  EXPECT_TRUE(ParseThreeVectorValue(node, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to handle the failure mode where the XML child node that's
// supposed to have the three vector is nullptr.
GTEST_TEST(test_parse_three_vector, node_input_nullptr) {
  XMLElement* node = nullptr;

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  EXPECT_FALSE(ParseThreeVectorValue(node, &parsed_vector3d));
}

// Tests the ability to load a three vector from an XML whose grandchild
// contains a three vector.
GTEST_TEST(test_parse_three_vector, node_child_input_vector) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>\n"
      "  <bar>5.5 6.6 7.7</bar>\n"
      "</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 5.5, 6.6, 7.7;

  EXPECT_TRUE(ParseThreeVectorValue(node, "bar", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML whose grandchild
// contains a scalar value.
GTEST_TEST(test_parse_three_vector, node_child_input_scalar) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>\n"
      "  <bar>93.5</bar>\n"
      "</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 93.5, 93.5, 93.5;

  EXPECT_TRUE(ParseThreeVectorValue(node, "bar", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to handle failure mode where XML grandchild or grandchild
// name are nullptr.
GTEST_TEST(test_parse_three_vector, node_child_input_nullptr_failure_mode) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>\n"
      "  <bar>93.5</bar>\n"
      "</foo>";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("foo");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  EXPECT_FALSE(ParseThreeVectorValue(nullptr, "bar", &parsed_vector3d));
  EXPECT_FALSE(ParseThreeVectorValue(node, nullptr, &parsed_vector3d));
  EXPECT_FALSE(ParseThreeVectorValue(nullptr, nullptr, &parsed_vector3d));
}

// Tests the ability to load a three vector from an XML attribute containing
// a three vector.
GTEST_TEST(test_parse_three_vector, node_attribute_input_vector) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"1.2 3.4 5.6\" />";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("mesh");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 1.2, 3.4, 5.6;

  EXPECT_TRUE(ParseThreeVectorAttribute(node, "scale", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML attribute containing
// a three vector.
GTEST_TEST(test_parse_three_vector, node_attribute_input_scalar) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"9.9\" />";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("mesh");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  Vector3d expected_vector3d;
  expected_vector3d << 9.9, 9.9, 9.9;

  EXPECT_TRUE(ParseThreeVectorAttribute(node, "scale", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to load a three vector from an XML attribute containing
// a three vector.
GTEST_TEST(test_parse_three_vector, node_attribute_input_nullptr) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"9.9\" />";

  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);

  XMLElement* node = xml_doc.FirstChildElement("mesh");

  Vector3d parsed_vector3d;
  parsed_vector3d << 0, 0, 0;

  EXPECT_FALSE(ParseThreeVectorAttribute(nullptr, "scale", &parsed_vector3d));
  EXPECT_FALSE(ParseThreeVectorAttribute(node, nullptr, &parsed_vector3d));
  EXPECT_FALSE(ParseThreeVectorAttribute(nullptr, nullptr, &parsed_vector3d));
}
}
