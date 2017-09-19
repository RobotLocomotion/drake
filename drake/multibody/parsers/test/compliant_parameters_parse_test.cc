/// @file  Tests the compliant parameter parsing for both sdf and urdf parsers.
#include <memory>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_parameters.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace parsers {
namespace {

using systems::CompliantContactParameters;
using std::make_unique;
using std::to_string;
using std::unique_ptr;

class CompliantParameterParseTest
    : public ::testing::TestWithParam<const char*> {
 protected:
  void SetUp() override {
    tree_ = make_unique<RigidBodyTreed>();
  }

  // Creates the specification of a box (embedding the given compliance string).
  // Attempts to parse the box and tests the parsed compliant parameters against
  // the given set.
  void ParseBoxAndCheck(
      const std::string& compliance_string,
      const CompliantContactParameters& expected) {
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
    ExpectParameters(expected);
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
  const CompliantContactParameters& GetBoxParameter() const {
    RigidBody<double>* body = tree_->FindBody("box", "box");
    EXPECT_NE(body, nullptr);
    const auto& element_ids = body->get_collision_element_ids();
    EXPECT_EQ(element_ids.size(), 1u);
    return (*body->collision_elements_begin())->compliant_parameters();
  }

  // Searches the tree for the "box" and compares the box's collision elements'
  // compliant contact parameters with the given expected set of values,
  // returning a reference to the element's parameters.
  void ExpectParameters(
      const CompliantContactParameters& expected) {
    const CompliantContactParameters& parsed = GetBoxParameter();
    EXPECT_EQ(parsed.dynamic_friction(), expected.dynamic_friction());
    EXPECT_EQ(parsed.static_friction(), expected.static_friction());
    EXPECT_EQ(parsed.dissipation(), expected.dissipation());
    EXPECT_EQ(parsed.stiffness(), expected.stiffness());
  }

  // Sets the default contact parameters to an arbitrary "crazy" set of values.
  // These *should* be different from any of the values explicitly used in the
  // tests. Returns an baked version of those defaults.
  CompliantContactParameters SetCrazyDefaults() {
    CompliantContactParameters new_defaults;
    new_defaults.set_stiffness(123456);
    new_defaults.set_dissipation(3);
    new_defaults.set_friction(10000, 9000);
    CompliantContactParameters::SetDefaultValues(new_defaults);
    return new_defaults;
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

const char* CompliantParameterParseTest::kSdfHead = \
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

const char* CompliantParameterParseTest::kSdfTail = \
    R"_(      </collision>\
    </link>
  </model>
</sdf>
)_";

const char* CompliantParameterParseTest::kUrdfHead = \
    R"_(<?xml version="1.0"?>
<robot name="box">
  <link name="box">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
)_";

const char* CompliantParameterParseTest::kUrdfTail = \
    R"_(    </collision>
  </link>
</robot>
)_";

INSTANTIATE_TEST_CASE_P(CompliantParameterParseTest,
                        CompliantParameterParseTest,
                        ::testing::Values("urdf", "sdf"));

// Test that parsing a file with *no* specification assigns the default
// compliant parameters to the element.
TEST_P(CompliantParameterParseTest, DefaultValues) {
  ParseBoxAndCheck("", CompliantContactParameters());
}

// Test that parsing a file with bad value tags ignores the bad tags.
TEST_P(CompliantParameterParseTest, BadComplianceTag) {
  std::string parameters_xml =
      "<drake_compliance><bad>13</bad></drake_compliance>\n";
  ParseBoxAndCheck(parameters_xml, CompliantContactParameters());
}

// Test that parsing a file that specifies only a single value (stiffness)
// assigns parameter values that are default except for the specified value.
TEST_P(CompliantParameterParseTest, SingleValueStiffness) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><stiffness>" +
      to_string(value) + "</stiffness></drake_compliance>\n";
  CompliantContactParameters expected;
  expected.set_stiffness(value);
  ParseBoxAndCheck(parameters_xml, expected);

  // Confirm that specifying stiffness *only* leaves all other fields tied to
  // the default values.
  const auto& parsed = GetBoxParameter();
  CompliantContactParameters new_defaults = SetCrazyDefaults();
  EXPECT_NE(parsed.stiffness(), new_defaults.stiffness());
  EXPECT_EQ(parsed.dissipation(), new_defaults.dissipation());
  EXPECT_EQ(parsed.static_friction(), new_defaults.static_friction());
  EXPECT_EQ(parsed.dynamic_friction(), new_defaults.dynamic_friction());
}

// Test that parsing a file that specifies only a single value (dissipation)
// assigns parameter values that are default except for the specified value.
TEST_P(CompliantParameterParseTest, SingleValueDissipation) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><dissipation>" +
      to_string(value) + "</dissipation></drake_compliance>\n";
  CompliantContactParameters expected;
  expected.set_dissipation(value);
  ParseBoxAndCheck(parameters_xml, expected);

  // Confirm that specifying dissipation *only* leaves all other fields tied to
  // the default values.
  const auto& parsed = GetBoxParameter();
  CompliantContactParameters new_defaults = SetCrazyDefaults();
  EXPECT_EQ(parsed.stiffness(), new_defaults.stiffness());
  EXPECT_NE(parsed.dissipation(), new_defaults.dissipation());
  EXPECT_EQ(parsed.static_friction(), new_defaults.static_friction());
  EXPECT_EQ(parsed.dynamic_friction(), new_defaults.dynamic_friction());
}

TEST_P(CompliantParameterParseTest, ValidFrictionValues) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><static_friction>" +
      to_string(value) + "</static_friction><dynamic_friction>" +
      to_string(value - 1) + "</dynamic_friction></drake_compliance>\n";
  CompliantContactParameters expected;
  expected.set_friction(value, value - 1);
  ParseBoxAndCheck(parameters_xml, expected);

  // Confirm that specifying friction leaves all other fields tied to
  // the default values.
  const auto& parsed = GetBoxParameter();
  CompliantContactParameters new_defaults = SetCrazyDefaults();
  EXPECT_EQ(parsed.stiffness(), new_defaults.stiffness());
  EXPECT_EQ(parsed.dissipation(), new_defaults.dissipation());
  EXPECT_NE(parsed.static_friction(), new_defaults.static_friction());
  EXPECT_NE(parsed.dynamic_friction(), new_defaults.dynamic_friction());
}

// Test that parsing a file that specifies only a single value is considered a
// parsing error.
TEST_P(CompliantParameterParseTest, SingleValueFriction) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><static_friction>" +
      to_string(value) + "</static_friction></drake_compliance>\n";
  CompliantContactParameters expected;
  EXPECT_THROW(ParseBoxAndCheck(parameters_xml, expected), std::runtime_error);

  parameters_xml = "<drake_compliance><dynamic_friction>" +
      to_string(value) + "</dynamic_friction></drake_compliance>\n";
  EXPECT_THROW(ParseBoxAndCheck(parameters_xml, expected), std::runtime_error);
}

// Test that parsing a file that specifies bad friction values is considered a
// parsing error.
TEST_P(CompliantParameterParseTest, BadFrictionValues) {
  CompliantContactParameters expected;

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
TEST_P(CompliantParameterParseTest, MisformattedValues) {
  CompliantContactParameters expected;

  auto format = [](const std::string& bad_value) {
    return "<drake_compliance><stiffness>" + bad_value +
           "</stiffness></drake_compliance>\n";
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
