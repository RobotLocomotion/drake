#include "drake/multibody/parsing/detail_tinyxml.h"

#include <locale>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using drake::math::RollPitchYawd;
using drake::math::RigidTransformd;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace drake {
namespace multibody {
namespace internal {
namespace {

GTEST_TEST(TinyxmlUtilTest, ParseAttributeTest) {
  const std::string test_xml =
      "<element scalar=\"1\" empty=\"\" bad=\"one\""
      "vec3=\"2.1 3.2 4.3\" vec4=\"5 6 7 8\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  std::string str;
  EXPECT_TRUE(ParseStringAttribute(element, "bad", &str));
  EXPECT_EQ(str, "one");
  EXPECT_FALSE(ParseStringAttribute(element, "missing", &str));

  double scalar{};
  EXPECT_TRUE(ParseScalarAttribute(element, "scalar", &scalar));
  EXPECT_EQ(scalar, 1.);
  EXPECT_FALSE(ParseScalarAttribute(element, "missing", &scalar));
  EXPECT_THROW(ParseScalarAttribute(element, "vec3", &scalar),
               std::invalid_argument);
  EXPECT_THROW(ParseScalarAttribute(element, "empty", &scalar),
               std::invalid_argument);
  EXPECT_THROW(ParseScalarAttribute(element, "bad", &scalar),
               std::invalid_argument);


  Vector3d vec3 = Vector3d::Zero();
  EXPECT_TRUE(ParseVectorAttribute(element, "vec3", &vec3));
  EXPECT_TRUE(CompareMatrices(vec3, Vector3d(2.1, 3.2, 4.3)));
  EXPECT_FALSE(ParseVectorAttribute(element, "vec3_foo", &vec3));
  EXPECT_THROW(ParseVectorAttribute(element, "vec4", &vec3),
               std::invalid_argument);
  EXPECT_THROW(ParseVectorAttribute(element, "scalar", &vec3),
               std::invalid_argument);

  Vector4d vec4 = Vector4d::Zero();
  EXPECT_TRUE(ParseVectorAttribute(element, "vec4", &vec4));
  EXPECT_TRUE(CompareMatrices(vec4, Vector4d(5, 6, 7, 8)));
  EXPECT_FALSE(ParseVectorAttribute(element, "vec4_foo", &vec4));
  EXPECT_THROW(ParseVectorAttribute(element, "vec3", &vec4),
               std::invalid_argument);
  EXPECT_THROW(ParseVectorAttribute(element, "scalar", &vec4),
               std::invalid_argument);
}

GTEST_TEST(TinyxmlUtilTest, OriginAttributesTest) {
  const std::string test_xml =
      "<element rpy=\"1 2 3\" xyz=\"4 5 6\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  const RigidTransformd actual = internal::OriginAttributesToTransform(element);
  const RigidTransformd expected(RollPitchYawd(1, 2, 3), Vector3d(4, 5, 6));

  EXPECT_TRUE(CompareMatrices(actual.GetAsMatrix4(), expected.GetAsMatrix4()));
}

GTEST_TEST(TinyxmlUtilTest, MalformedRpyOriginAttributesTest) {
  const std::string test_xml =
      "<element rpy=\"1 2\" xyz=\"4 5 6\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  EXPECT_THROW(internal::OriginAttributesToTransform(element),
               std::invalid_argument);
}

GTEST_TEST(TinyxmlUtilTest, ThreeVectorAttributeTest) {
  const std::string test_xml =
      "<element one=\"1\" three=\"4 5 6\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  Vector3d out = Vector3d::Zero();
  EXPECT_TRUE(internal::ParseThreeVectorAttribute(element, "one", &out));
  EXPECT_TRUE(CompareMatrices(out, Vector3d(1, 1, 1)));

  EXPECT_TRUE(internal::ParseThreeVectorAttribute(element, "three", &out));
  EXPECT_TRUE(CompareMatrices(out, Vector3d(4, 5, 6)));

  EXPECT_FALSE(internal::ParseThreeVectorAttribute(element, "meh", &out));
}

GTEST_TEST(TinyxmlUtilTest, LocaleAttributeTest) {
  const std::string test_xml = "<element oneAndHalf=\"1.5\"/>";

  struct CommaDecimalPointFacet : std::numpunct<char> {
    char do_decimal_point() const {
      return ',';
    }
  };

  // Set a global locale in which the decimal separator is the comma
  std::locale original_global_locale =
    std::locale::global(std::locale(std::locale::classic(),
                                    new CommaDecimalPointFacet));

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  double scalar = 0.0;
  EXPECT_TRUE(internal::ParseScalarAttribute(element, "oneAndHalf", &scalar));
  EXPECT_EQ(scalar, 1.5);

  // Restore the original global locale
  std::locale::global(original_global_locale);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
