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

  void ParseBox(const std::string& compliance_string,
                const CompliantContactParameters& expected) {
    std::string format = GetParam();
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

  void ExpectParameters(const CompliantContactParameters& expected) {
    RigidBody<double>& body = *tree_->FindBody("box", "box");
    const auto& element_ids = body.get_collision_element_ids();
    EXPECT_EQ(element_ids.size(), 1u);
    for (auto iter = body.collision_elements_begin();
         iter != body.collision_elements_end();
         ++iter) {
      const CompliantContactParameters & parsed =
          (*iter)->compliant_parameters();
      EXPECT_EQ(parsed.dynamic_friction(), expected.dynamic_friction());
      EXPECT_EQ(parsed.static_friction(), expected.static_friction());
      EXPECT_EQ(parsed.dissipation(), expected.dissipation());
      EXPECT_EQ(parsed.stiffness(), expected.stiffness());
    }
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

const char* CompliantParameterParseTest::kSdfHead =
    "<?xml version=\"1.0\"?>\n"
        "<sdf version=\"1.4\">\n"
        "  <model name=\"box\">\n"
        "    <pose>0 0 0 0 0 0</pose>\n"
        "    <link name=\"box\">\n"
        "      <collision name=\"box\">\n"
        "        <geometry>\n"
        "          <box>\n"
        "            <size>0.1 0.1 0.1</size>\n"
        "          </box>\n"
        "        </geometry>\n";

const char* CompliantParameterParseTest::kSdfTail =
        "      </collision>\n"
        "    </link>\n"
        "  </model>\n"
        "</sdf>\n";

const char* CompliantParameterParseTest::kUrdfHead =
    "<?xml version=\"1.0\"?>\n"
        "<robot name=\"box\">\n"
        "  <link name=\"box\">\n"
        "    <collision>\n"
        "      <geometry>\n"
        "        <box size=\"0.1 0.1 0.1\"/>\n"
        "      </geometry>\n";

const char* CompliantParameterParseTest::kUrdfTail =
    "    </collision>\n"
        "  </link>\n"
        "</robot>\n";

INSTANTIATE_TEST_CASE_P(CompliantParameterParseTest,
                        CompliantParameterParseTest,
                        ::testing::Values("urdf", "sdf"));

// Test that parsing a file with *no* specification assigns the default
// compliant parameters to the element.
TEST_P(CompliantParameterParseTest, DefaultValues) {
  ParseBox("", CompliantContactParameters());
}

// Test that parsing a file with specification with invalid values does ignores
// the bad tags.
TEST_P(CompliantParameterParseTest, BadComplianceTag) {
  std::string parameters_xml =
      "<drake_compliance><bad>13</bad></drake_compliance>\n";
  ParseBox(parameters_xml, CompliantContactParameters());
}

// Test that parsing a file that specifies only a single value (stiffness)
// assigns parameter values that are default except for the specified value.
TEST_P(CompliantParameterParseTest, SingleValueStiffness) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><penetration_stiffness>" +
      to_string(value) + "</penetration_stiffness></drake_compliance>\n";
  CompliantContactParameters expected;
  expected.set_stiffness(value);
  ParseBox(parameters_xml, expected);
}

// Test that parsing a file that specifies only a single value (dissipation)
// assigns parameter values that are default except for the specified value.
TEST_P(CompliantParameterParseTest, SingleValueDissipation) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><dissipation>" +
      to_string(value) + "</dissipation></drake_compliance>\n";
  CompliantContactParameters expected;
  expected.set_dissipation(value);
  ParseBox(parameters_xml, expected);
}

// Test that parsing a file that specifies only a single value is considered a
// parsing error.
TEST_P(CompliantParameterParseTest, SingleValueStaticFriction) {
  double value = 15;
  std::string parameters_xml = "<drake_compliance><static_friction>" +
      to_string(value) + "</static_friction></drake_compliance>\n";
  CompliantContactParameters expected;
  EXPECT_THROW(ParseBox(parameters_xml, expected), std::runtime_error);

  parameters_xml = "<drake_compliance><dynamic_friction>" +
      to_string(value) + "</dynamic_friction></drake_compliance>\n";
  EXPECT_THROW(ParseBox(parameters_xml, expected), std::runtime_error);
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
  EXPECT_THROW(ParseBox(format(-1, -1), expected), std::runtime_error);
  // Case 2: static negative
  EXPECT_THROW(ParseBox(format(-1, 1), expected), std::runtime_error);
  // Case 3: dynamic negative
  EXPECT_THROW(ParseBox(format(1, -1), expected), std::runtime_error);
  // Case 4: dynamic > static
  EXPECT_THROW(ParseBox(format(0.5, 1.0), expected), std::runtime_error);
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
  EXPECT_THROW(ParseBox(format(""), expected), std::runtime_error);
  // Case 2: Trailing non-numerical characters
  EXPECT_THROW(ParseBox(format("1.3d"), expected), std::runtime_error);
  // Case 3: Leading non-numerical characters
  EXPECT_THROW(ParseBox(format("d1.5"), expected), std::invalid_argument);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
