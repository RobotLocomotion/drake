/// @file  Tests the compliant parameter parsing for both sdf and urdf parsers.
#include <memory>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace parsers {
namespace {

using systems::CompliantMaterial;
using std::make_unique;
using std::to_string;
using std::unique_ptr;

class CompliantMaterialParseTest
    : public ::testing::TestWithParam<const char*> {
 protected:
  void SetUp() override { tree_ = make_unique<RigidBodyTreed>(); }

  // Creates the specification of a box (embedding the given compliance string).
  // Attempts to parse the box and tests the parsed compliant parameters against
  // the given set.
  void ParseBoxAndCheck(const std::string& compliance_string,
                        const CompliantMaterial& expected) {
    const std::string format = GetParam();
    if (format == "urdf") {
      urdf::AddModelInstanceFromUrdfString(
          FormatData(kUrdfHead, compliance_string, kUrdfTail), ".",
          drake::multibody::joints::kFixed, nullptr, tree_.get());
    } else if (format == "sdf") {
      sdf::AddModelInstancesFromSdfString(
          FormatData(kSdfHead, compliance_string, kSdfTail),
          drake::multibody::joints::kFixed, nullptr, tree_.get());
    } else {
      GTEST_FAIL() << "Unrecognized model format: " << format;
    }
    ExpectMaterial(expected);
  }

  // Inserts the given compliance string into the sdf specification.
  std::string FormatData(const std::string& head,
                         const std::string& compliance_string,
                         const std::string& tail) {
    std::stringstream ss;
    ss << head << compliance_string << tail;
    return ss.str();
  }

  // Searches the tree for the box body and extracts its collision element's
  // compliant contact parameters, returning a reference.
  const CompliantMaterial& GetBoxMaterial() const {
    RigidBody<double>* body = tree_->FindBody("box", "box");
    EXPECT_NE(body, nullptr);
    const auto& element_ids = body->get_collision_element_ids();
    EXPECT_EQ(element_ids.size(), 1u);
    return (*body->collision_elements_begin())->compliant_material();
  }

  // Searches the tree for the "box" and compares the box's collision elements'
  // compliant contact parameters with the given expected set of values,
  // returning a reference to the element's parameters.
  void ExpectMaterial(const CompliantMaterial& expected) {
    const CompliantMaterial& parsed = GetBoxMaterial();
    EXPECT_EQ(parsed.dynamic_friction(), expected.dynamic_friction());
    EXPECT_EQ(parsed.static_friction(), expected.static_friction());
    EXPECT_EQ(parsed.dissipation(), expected.dissipation());
    EXPECT_EQ(parsed.youngs_modulus(), expected.youngs_modulus());
  }

  unique_ptr<RigidBodyTree<double>> tree_;

  // The definition of a single-link model broken up so that a drake compliance
  // tag can be injected into the collision specification. One for sdf and one
  // for urdf.
  static const char* kSdfHead;
  static const char* kSdfTail;
  static const char* kUrdfHead;
  static const char* kUrdfTail;
};

const char* CompliantMaterialParseTest::kSdfHead =
    R"_(<?xml version="1.0"?>
<sdf version="1.4">
  <model name="box">
    <pose>0 0 0 0 0 0</pose>
    <link name="box">
      <collision name="box">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
)_";

const char* CompliantMaterialParseTest::kSdfTail =
    R"_(      </collision>\
    </link>
  </model>
</sdf>
)_";

const char* CompliantMaterialParseTest::kUrdfHead =
    R"_(<?xml version="1.0"?>
<robot name="box">
  <link name="box">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
)_";

const char* CompliantMaterialParseTest::kUrdfTail =
    R"_(    </collision>
  </link>
</robot>
)_";

INSTANTIATE_TEST_CASE_P(CompliantMaterialParseTest, CompliantMaterialParseTest,
                        ::testing::Values("urdf", "sdf"));

// Test that parsing a file with *no* specification assigns the default
// compliant parameters to the element.
TEST_P(CompliantMaterialParseTest, DefaultValues) {
  ParseBoxAndCheck("", CompliantMaterial());
}

// Test that parsing a file with bad value tags ignores the bad tags.
TEST_P(CompliantMaterialParseTest, BadComplianceTag) {
  std::string parameters_xml =
      "<drake_compliance><bad>13</bad></drake_compliance>\n";
  ParseBoxAndCheck(parameters_xml, CompliantMaterial());
}

// Test that parsing a file that specifies only a single value (Young's modulus)
// assigns parameter values that are default except for the specified value.
TEST_P(CompliantMaterialParseTest, SingleValueYoungsMOdulus) {
  double value = 15;
  CompliantMaterial defaults;
  EXPECT_NE(value, defaults.youngs_modulus());
  std::string parameters_xml = "<drake_compliance><youngs_modulus>" +
                               to_string(value) +
                               "</youngs_modulus></drake_compliance>\n";
  CompliantMaterial expected;
  expected.set_youngs_modulus(value);
  ParseBoxAndCheck(parameters_xml, expected);

  // Confirm that specifying Young's modulus *only* leaves all other fields tied
  // to the default values.
  const auto& parsed = GetBoxMaterial();
  EXPECT_NE(parsed.youngs_modulus(defaults.youngs_modulus()),
            defaults.youngs_modulus());
  EXPECT_EQ(parsed.dissipation(defaults.dissipation()), defaults.dissipation());
  EXPECT_EQ(parsed.static_friction(defaults.static_friction()),
            defaults.static_friction());
  EXPECT_EQ(parsed.dynamic_friction(defaults.dynamic_friction()),
            defaults.dynamic_friction());
}

// Test that parsing a file that specifies only a single value (dissipation)
// assigns parameter values that are default except for the specified value.
TEST_P(CompliantMaterialParseTest, SingleValueDissipation) {
  double value = 15;
  CompliantMaterial defaults;
  EXPECT_NE(value, defaults.dissipation());
  std::string parameters_xml = "<drake_compliance><dissipation>" +
                               to_string(value) +
                               "</dissipation></drake_compliance>\n";
  CompliantMaterial expected;
  expected.set_dissipation(value);
  ParseBoxAndCheck(parameters_xml, expected);

  // Confirm that specifying dissipation *only* leaves all other fields tied to
  // the default values.
  const auto& parsed = GetBoxMaterial();
  EXPECT_EQ(parsed.youngs_modulus(defaults.youngs_modulus()),
            defaults.youngs_modulus());
  EXPECT_NE(parsed.dissipation(defaults.dissipation()), defaults.dissipation());
  EXPECT_EQ(parsed.static_friction(defaults.static_friction()),
            defaults.static_friction());
  EXPECT_EQ(parsed.dynamic_friction(defaults.dynamic_friction()),
            defaults.dynamic_friction());
}

TEST_P(CompliantMaterialParseTest, ValidFrictionValues) {
  double value = 15;
  CompliantMaterial defaults;
  EXPECT_NE(value, defaults.static_friction());
  EXPECT_NE(value - 1, defaults.dynamic_friction());
  std::string parameters_xml =
      "<drake_compliance><static_friction>" + to_string(value) +
      "</static_friction><dynamic_friction>" + to_string(value - 1) +
      "</dynamic_friction></drake_compliance>\n";
  CompliantMaterial expected;
  expected.set_friction(value, value - 1);
  ParseBoxAndCheck(parameters_xml, expected);

  // Confirm that specifying friction leaves all other fields tied to
  // the default values.
  const auto& parsed = GetBoxMaterial();
  EXPECT_EQ(parsed.youngs_modulus(defaults.youngs_modulus()),
            defaults.youngs_modulus());
  EXPECT_EQ(parsed.dissipation(defaults.dissipation()), defaults.dissipation());
  EXPECT_NE(parsed.static_friction(defaults.static_friction()),
            defaults.static_friction());
  EXPECT_NE(parsed.dynamic_friction(defaults.dynamic_friction()),
            defaults.dynamic_friction());
}

// Test that parsing a file that specifies only a single value is considered a
// parsing error.
TEST_P(CompliantMaterialParseTest, SingleValueFriction) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><static_friction>" +
                               to_string(value) +
                               "</static_friction></drake_compliance>\n";
  CompliantMaterial expected;
  EXPECT_THROW(ParseBoxAndCheck(parameters_xml, expected), std::runtime_error);

  parameters_xml = "<drake_compliance><dynamic_friction>" + to_string(value) +
                   "</dynamic_friction></drake_compliance>\n";
  EXPECT_THROW(ParseBoxAndCheck(parameters_xml, expected), std::runtime_error);
}

// Test that parsing a file that specifies bad friction values is considered a
// parsing error.
TEST_P(CompliantMaterialParseTest, BadFrictionValues) {
  CompliantMaterial expected;

  auto format = [](double static_friction, double dynamic_friction) {
    return "<drake_compliance><dynamic_friction>" +
           to_string(dynamic_friction) +
           "</dynamic_friction>"
           "<static_friction>" +
           to_string(static_friction) +
           "</static_friction></drake_compliance>\n";
  };

  // Case 1: Both values negative
  EXPECT_THROW(ParseBoxAndCheck(format(-1, -1), expected), std::runtime_error);
  // Case 2: static negative
  EXPECT_THROW(ParseBoxAndCheck(format(-1, 1), expected), std::runtime_error);
  // Case 3: dynamic negative
  EXPECT_THROW(ParseBoxAndCheck(format(1, -1), expected), std::runtime_error);
  // Case 4: dynamic > static
  EXPECT_THROW(ParseBoxAndCheck(format(0.5, 1.0), expected),
               std::runtime_error);
}

// Test errors on various malformed XML (e.g., missing numerical value,
// non-numerical value.
TEST_P(CompliantMaterialParseTest, MisformattedValues) {
  CompliantMaterial expected;

  auto format = [](const std::string& bad_value) {
    return "<drake_compliance><youngs_modulus>" + bad_value +
           "</youngs_modulus></drake_compliance>\n";
  };

  // Case 1: Missing value
  EXPECT_THROW(ParseBoxAndCheck(format(""), expected), std::runtime_error);
  // Case 2: Trailing non-numerical characters
  EXPECT_THROW(ParseBoxAndCheck(format("1.3d"), expected),
               std::invalid_argument);
  // Case 3: Leading non-numerical characters
  EXPECT_THROW(ParseBoxAndCheck(format("d1.5"), expected),
               std::invalid_argument);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
