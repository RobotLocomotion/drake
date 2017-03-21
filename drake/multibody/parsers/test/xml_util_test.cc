#include "drake/multibody/parsers/xml_util.h"

#include <list>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

using Eigen::Vector3d;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

// Tests the ability to load a three vector from a string. Evaluates both
// the case where the string describes a three vector and the case where
// the string describes a single scalar value.
GTEST_TEST(test_parse_three_vector, from_string) {
  const char* three_value_string = "1.1 2.2 3.3";
  const char* one_value_string = "4.4";
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(1.1, 2.2, 3.3);
  EXPECT_NO_THROW(ParseThreeVectorValue(three_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
  expected_vector3d << 4.4, 4.4, 4.4;
  EXPECT_NO_THROW(ParseThreeVectorValue(one_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to gracefully handle the failure mode where the string
// containing the vector or scalar value is nullptr.
GTEST_TEST(test_parse_three_vector, string_input_null_failure_mode) {
  Vector3d parsed_vector3d = Vector3d::Zero();
  const char* null_string = nullptr;
  EXPECT_THROW(ParseThreeVectorValue(null_string, &parsed_vector3d),
               std::invalid_argument);
  EXPECT_THROW(ParseThreeVectorValue("0.0, 1.1, 2.2", nullptr),
               std::invalid_argument);
  EXPECT_THROW(ParseThreeVectorValue(null_string, nullptr),
               std::invalid_argument);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides an empty string as the vector.
GTEST_TEST(test_parse_three_vector, string_input_empty_string_failure_mode) {
  const char* empty_string = "";
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorValue(empty_string, &parsed_vector3d),
               std::invalid_argument);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides a two-vector.
GTEST_TEST(test_parse_three_vector, string_input_two_vector_failure_mode) {
  const char* two_value_string = "2.1 0.3";
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorValue(two_value_string, &parsed_vector3d),
               std::invalid_argument);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides a vector with more than three elements.
GTEST_TEST(test_parse_three_vector,
           string_input_more_than_three_vector_failure_mode) {
  const char* many_value_string = "2.1 0.3 4.9 10.2 11.3";
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorValue(many_value_string, &parsed_vector3d),
               std::invalid_argument);
}

// Tests the ability to gracefully handle the failure mode where the user
// provides a vector with non-double-type values.
GTEST_TEST(test_parse_three_vector,
           string_input_non_double_vector_failure_mode) {
  std::vector<std::string> invalid_strings;
  invalid_strings.push_back("foo bar baz");
  invalid_strings.push_back("1.0 bar baz");
  invalid_strings.push_back("2.0 3.0 baz");
  invalid_strings.push_back("foo 4.0 4.1");
  invalid_strings.push_back("foo bar 4.1");
  invalid_strings.push_back("9.2 bar 4.1");
  invalid_strings.push_back("1.1foo 2.2bar 3.3baz");
  Vector3d parsed_vector3d = Vector3d::Zero();
  for (const std::string& bad_value : invalid_strings) {
    EXPECT_THROW(ParseThreeVectorValue(bad_value.c_str(), &parsed_vector3d),
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(5.5, 6.6, 7.7);
  EXPECT_NO_THROW(ParseThreeVectorValue(node, &parsed_vector3d));
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(10.1, 10.1, 10.1);
  EXPECT_NO_THROW(ParseThreeVectorValue(node, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the ability to handle the failure mode where the XML child node that's
// supposed to have the three vector is nullptr.
GTEST_TEST(test_parse_three_vector, node_input_nullptr) {
  XMLElement* node = nullptr;
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorValue(node, &parsed_vector3d),
               std::invalid_argument);
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(5.5, 6.6, 7.7);
  EXPECT_NO_THROW(ParseThreeVectorValue(node, "bar", &parsed_vector3d));
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(93.5, 93.5, 93.5);
  EXPECT_NO_THROW(ParseThreeVectorValue(node, "bar", &parsed_vector3d));
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorValue(nullptr, "bar", &parsed_vector3d),
               std::invalid_argument);
  EXPECT_THROW(ParseThreeVectorValue(node, nullptr, &parsed_vector3d),
               std::invalid_argument);
  EXPECT_THROW(ParseThreeVectorValue(nullptr, nullptr, &parsed_vector3d),
               std::invalid_argument);
}

// Tests failure mode where the specified element does not exist.
GTEST_TEST(test_parse_three_vector, node_child_input_no_element_failure_mode) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<foo>\n"
      "  <bar>93.5</bar>\n"
      "</foo>";
  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);
  XMLElement* node = xml_doc.FirstChildElement("foo");
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorValue(node, "baz", &parsed_vector3d),
               std::invalid_argument);
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(1.2, 3.4, 5.6);
  EXPECT_NO_THROW(ParseThreeVectorAttribute(node, "scale", &parsed_vector3d));
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
  Vector3d parsed_vector3d = Vector3d::Zero();
  Vector3d expected_vector3d(9.9, 9.9, 9.9);
  EXPECT_NO_THROW(ParseThreeVectorAttribute(node, "scale", &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

// Tests the failure mode where the parameters to ParseThreeVectorAttribute are
// null.
GTEST_TEST(test_parse_three_vector, node_attribute_input_nullptr) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"9.9\" />";
  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);
  XMLElement* node = xml_doc.FirstChildElement("mesh");
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorAttribute(nullptr, "scale", &parsed_vector3d),
               std::invalid_argument);
  EXPECT_THROW(ParseThreeVectorAttribute(node, nullptr, &parsed_vector3d),
               std::invalid_argument);
  EXPECT_THROW(ParseThreeVectorAttribute(nullptr, nullptr, &parsed_vector3d),
               std::invalid_argument);
}

// Tests the failure mode where the the attribute name does not exist.
GTEST_TEST(test_parse_three_vector, node_attribute_does_not_exist) {
  const char* xml_string =
      "<?xml version=\"1.0\" ?>\n"
      "<mesh scale=\"9.9\" />";
  XMLDocument xml_doc;
  xml_doc.Parse(xml_string);
  XMLElement* node = xml_doc.FirstChildElement("mesh");
  Vector3d parsed_vector3d = Vector3d::Zero();
  EXPECT_THROW(ParseThreeVectorAttribute(node, "foo", &parsed_vector3d),
               std::invalid_argument);
}

}  // namespace
