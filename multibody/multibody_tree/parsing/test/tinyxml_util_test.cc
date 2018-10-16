#include "drake/multibody/multibody_tree/parsing/tinyxml_util.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {
namespace {

GTEST_TEST(TinyxmlUtilTest, ParseAttributeTest) {
  const std::string test_xml =
      "<element scalar=\"1\" empty=\"\" bad=\"one\""
      "vec3=\"2.1 3.2 4.3\" vec4=\"5 6 7 8\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

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


  Eigen::Vector3d vec3 = Eigen::Vector3d::Zero();
  EXPECT_TRUE(ParseVectorAttribute(element, "vec3", &vec3));
  EXPECT_TRUE(CompareMatrices(vec3, Eigen::Vector3d(2.1, 3.2, 4.3)));
  EXPECT_FALSE(ParseVectorAttribute(element, "vec3_foo", &vec3));
  EXPECT_THROW(ParseVectorAttribute(element, "vec4", &vec3),
               std::invalid_argument);
  EXPECT_THROW(ParseVectorAttribute(element, "scalar", &vec3),
               std::invalid_argument);

  Eigen::Vector4d vec4 = Eigen::Vector4d::Zero();
  EXPECT_TRUE(ParseVectorAttribute(element, "vec4", &vec4));
  EXPECT_TRUE(CompareMatrices(vec4, Eigen::Vector4d(5, 6, 7, 8)));
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

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T = detail::OriginAttributesToTransform(element);

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  const math::RollPitchYaw<double> rpy(1, 2, 3);
  Eigen::Vector3d xyz(4, 5, 6);
  expected.matrix() << rpy.ToMatrix3ViaRotationMatrix(), xyz, 0, 0, 0, 1;

  EXPECT_TRUE(CompareMatrices(T.matrix(), expected.matrix()));
}

GTEST_TEST(TinyxmlUtilTest, MalformedRpyOriginAttributesTest) {
  const std::string test_xml =
      "<element rpy=\"1 2\" xyz=\"4 5 6\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  EXPECT_THROW(detail::OriginAttributesToTransform(element),
               std::invalid_argument);
}

GTEST_TEST(TinyxmlUtilTest, ThreeVectorAttributeTest) {
  const std::string test_xml =
      "<element one=\"1\" three=\"4 5 6\"/>";

  XMLDocument xml_doc;
  xml_doc.Parse(test_xml.c_str());
  XMLElement* element = xml_doc.FirstChildElement("element");
  ASSERT_TRUE(element != nullptr);

  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  EXPECT_TRUE(detail::ParseThreeVectorAttribute(element, "one", &out));
  EXPECT_TRUE(CompareMatrices(out, Eigen::Vector3d(1, 1, 1)));

  EXPECT_TRUE(detail::ParseThreeVectorAttribute(element, "three", &out));
  EXPECT_TRUE(CompareMatrices(out, Eigen::Vector3d(4, 5, 6)));

  EXPECT_FALSE(detail::ParseThreeVectorAttribute(element, "meh", &out));
}

}  // namespace
}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
