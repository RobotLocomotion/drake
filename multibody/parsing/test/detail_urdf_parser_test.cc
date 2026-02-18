#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/curvilinear_joint.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/universal_joint.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::MatchesRegex;

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using drake::trajectories::PiecewiseConstantCurvatureTrajectory;
using drake::trajectories::Trajectory;
using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::SceneGraph;

// Fixture to add test coverage for the URDF parser. Some features such as mimic
// joints and ball constraints are only supported in discrete mode when using
// the SAP solver. For testing such features, we set a model approximation that
// uses the SAP solver. More specifically, we call
// set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
// MultibodyPlant used for testing before parsing.
class UrdfParserTestBase : public test::DiagnosticPolicyTestBase {
 public:
  explicit UrdfParserTestBase(double time_step) : plant_(time_step) {
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  }

  std::optional<ModelInstanceIndex> AddModelFromUrdfFile(
      const std::string& file_name, const std::string& model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       nullptr,  &plant_,      &resolver,
                       NoSelect};
    auto result = AddModelFromUrdf({DataSource::kFilename, &file_name},
                                   model_name, {}, w);
    last_parsed_groups_ = ConvertInstancedNamesToStrings(
        resolver.Resolve(diagnostic_policy_), plant_);
    return result;
  }

  std::optional<ModelInstanceIndex> AddModelFromUrdfString(
      const std::string& file_contents, const std::string& model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       nullptr,  &plant_,      &resolver,
                       NoSelect};
    auto result = AddModelFromUrdf({DataSource::kContents, &file_contents},
                                   model_name, {}, w);
    last_parsed_groups_ = ConvertInstancedNamesToStrings(
        resolver.Resolve(diagnostic_policy_), plant_);
    return result;
  }

  // URDF cannot delegate to any other parsers.
  static ParserInterface& NoSelect(const drake::internal::DiagnosticPolicy&,
                                   const std::string&) {
    DRAKE_UNREACHABLE();
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  // Note: We currently use a discrete plant here to be able to test
  // Sap-specific features like the joint 'mimic' element.
  MultibodyPlant<double> plant_{0.1};
  SceneGraph<double> scene_graph_;
  CollisionFilterGroupsImpl<std::string> last_parsed_groups_;
};

class UrdfParserTest : public UrdfParserTestBase {
 public:
  UrdfParserTest() : UrdfParserTestBase(0.1) {}
};

class UrdfParserTestContinuous : public UrdfParserTestBase {
 public:
  UrdfParserTestContinuous() : UrdfParserTestBase(0.0) {}
};

// Some tests contain deliberate typos to provoke parser errors or warnings. In
// those cases, the sequence `QQQ` will be inserted to stand in for some more
// naturalistic typo.

TEST_F(UrdfParserTest, BadFilename) {
  EXPECT_EQ(AddModelFromUrdfFile("nonexistent.urdf", ""), std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex("/.*/nonexistent.urdf:0: error: "
                   "Failed to parse XML file: XML_ERROR_FILE_NOT_FOUND"));
}

TEST_F(UrdfParserTest, BadXmlString) {
  EXPECT_EQ(AddModelFromUrdfString("not proper xml content", ""), std::nullopt);
  EXPECT_EQ(TakeError(),
            "<literal-string>.urdf:1: error: Failed to parse"
            " XML string: XML_ERROR_PARSING_TEXT");
}

TEST_F(UrdfParserTest, NoRobot) {
  EXPECT_EQ(AddModelFromUrdfString("<empty/>", ""), std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*URDF does not contain a robot tag."));
}

TEST_F(UrdfParserTest, NoName) {
  EXPECT_EQ(AddModelFromUrdfString("<robot/>", ""), std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(
                  ".*Your robot must have a name attribute or a model name must"
                  " be specified."));
}

TEST_F(UrdfParserTest, ModelRenameWithColons) {
  std::optional<ModelInstanceIndex> index =
      AddModelFromUrdfString(R"""(
    <robot name='to-be-overwritten'>
    </robot>)""",
                             "left::robot");
  ASSERT_NE(index, std::nullopt);
  EXPECT_EQ(plant_.GetModelInstanceName(*index), "left::robot");
}

TEST_F(UrdfParserTest, ObsoleteLoopJoint) {
  EXPECT_NE(AddModelFromUrdfString("<robot name='a'><loop_joint/></robot>", ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*loop joints are not supported in MultibodyPlant"));
}

TEST_F(UrdfParserTest, LegacyDrakeIgnoreBody) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='ignore-me' drake_ignore='true'>
        <!-- Empty visual will trigger errors if ignore fails. -->
        <visual/>
      </link>
    </robot>)""",
                                   ""),
            std::nullopt);
}

TEST_F(UrdfParserTest, BodyNameBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link naQQQme='broken'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*link tag is missing name attribute."));
}

TEST_F(UrdfParserTest, LegacyDrakeIgnoreJoint) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <!-- Empty joint will trigger errors if ignore fails. -->
      <joint name='ignore-me' drake_ignore='true'/>
    </robot>)""",
                                   ""),
            std::nullopt);
}

TEST_F(UrdfParserTest, JointNameBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <joint naQQQme='broken'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*joint tag is missing name attribute"));
}

TEST_F(UrdfParserTest, JointTypeBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <joint name='a' tyQQQpe='broken'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*joint 'a' is missing type attribute"));
}

TEST_F(UrdfParserTest, JointNoParent) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <joint name='a' type='revolute'>
        <parQQQent link='parent'/>
        <child link='child'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*joint 'a' doesn't have a parent node!"));
}

TEST_F(UrdfParserTest, JointParentLinkBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent liQQQnk='broken'/>
        <child link='child'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*joint a's parent does not have a link attribute!"));
}

TEST_F(UrdfParserTest, JointNoChild) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <chiQQQld link='child'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*joint 'a' doesn't have a child node!"));
}

TEST_F(UrdfParserTest, JointChildLinkBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child liQQQnk='broken'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*joint a's child does not have a link attribute!"));
}

TEST_F(UrdfParserTest, JointBadDynamicsAttributes) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <dynamics {}/>
      </joint>
    </robot>)""";
  const auto attrs =
      std::array<std::string, 3>{"damping", "friction", "coulomb_window"};
  for (const auto& attr : attrs) {
    EXPECT_NE(AddModelFromUrdfString(fmt::format(base, attr + "='1 2'"), attr),
              std::nullopt);
    EXPECT_THAT(TakeError(),
                MatchesRegex(".*Expected single value.*" + attr + ".*"));
  }
  // Dynamics warnings are tested elsewhere in this file.
  warning_records_.clear();
}

TEST_F(UrdfParserTest, DrakeFrictionWarning) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <dynamics friction='10'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*joint friction.*"));
}

TEST_F(UrdfParserTest, DrakeCoulombWarning) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <dynamics coulomb_window='10'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*coulomb_window.*ignored.*"));
}

TEST_F(UrdfParserTest, JointLinkMissing) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <!-- Parent link is missing. -->
      <link name='child'/>
      <joint name='joint' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Could not find link named 'parent' with model instance"
                   " ID 2 for element 'joint'."));
}

TEST_F(UrdfParserTest, JointZeroAxis) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='joint' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <axis xyz="0 0 0"/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint 'joint' axis is zero.  Don't do that."));
}

TEST_F(UrdfParserTest, JointFloatingWarning) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='joint' type='floating'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*Joint 'joint' specified as type floating which is not"
                   " supported by MultibodyPlant.  Leaving 'child' as a"
                   " free body."));
}

TEST_F(UrdfParserTest, JointTypeUnknown) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='joint' type='who'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint 'joint' has unrecognized type: 'who'"));
}

static constexpr char kMimicModel[] = R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child0'/>
      <link name='child1'/>
      <joint name='joint0' type='revolute'>
        <parent link='parent'/>
        <child link='child0'/>
      </joint>
      <joint name='joint1' type='revolute'>
        <parent link='parent'/>
        <child link='child1'/>
        <mimic joint='joint0'/>
      </joint>
    </robot>)""";

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(UrdfParserTest, MimicNoSap) {
  // Currently the <mimic> tag is only supported by SAP. Setting the solver
  // to TAMSI should be a warning.
  plant_.set_discrete_contact_approximation(
      DiscreteContactApproximation::kTamsi);
  EXPECT_NE(AddModelFromUrdfString(kMimicModel, ""), std::nullopt);
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*Mimic elements are currently only supported by MultibodyPlant "
          "with a discrete time step and using "
          "DiscreteContactSolver::kSap..*or.*continuous.*CENIC.*"));
}

TEST_F(UrdfParserTestContinuous, MimicContinuous) {
  // Feature support in continuous plants depends on integrator selection, so
  // can't be checked at parsing time.
  EXPECT_NE(AddModelFromUrdfString(kMimicModel, ""), std::nullopt);
}
#pragma GCC diagnostic pop

TEST_F(UrdfParserTest, MimicNoJoint) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='joint' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <mimic/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint 'joint' mimic element is missing the "
                           "required 'joint' attribute."));
}

TEST_F(UrdfParserTest, MimicBadJoint) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='joint' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <mimic joint='nonexistent'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint 'joint' mimic element specifies joint "
                           "'nonexistent' which does not exist."));
}

TEST_F(UrdfParserTest, MimicSameJoint) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='joint' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <mimic joint='joint'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint 'joint' mimic element specifies joint "
                           "'joint'. Joints cannot mimic themselves."));
}

TEST_F(UrdfParserTest, MimicMismatchedJoint) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child0'/>
      <link name='child1'/>
      <joint name='joint0' type='fixed'>
        <parent link='parent'/>
        <child link='child0'/>
      </joint>
      <joint name='joint1' type='revolute'>
        <parent link='parent'/>
        <child link='child1'/>
        <mimic joint='joint0'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint 'joint1' which has 1 DOF cannot mimic "
                           "joint 'joint0' which has 0 DOF."));
}

TEST_F(UrdfParserTest, MimicOnlyOneDOFJoint) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child0'/>
      <link name='child1'/>
      <joint name='joint0' type='fixed'>
        <parent link='parent'/>
        <child link='child0'/>
      </joint>
      <joint name='joint1' type='fixed'>
        <parent link='parent'/>
        <child link='child1'/>
        <mimic joint='joint0'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*Drake only supports the mimic element for "
                           "single-dof joints.*"));
}

TEST_F(UrdfParserTest, MimicFloatingJoint) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child0'/>
      <link name='child1'/>
      <joint name='joint0' type='fixed'>
        <parent link='parent'/>
        <child link='child0'/>
      </joint>
      <joint name='joint1' type='floating'>
        <parent link='parent'/>
        <child link='child1'/>
        <mimic joint='joint0'/>
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt);
  TakeWarning();  // The first warning is about not supporting floating joints.
                  // See issue #13691.
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*Drake only supports the mimic element for "
                           "single-dof joints.*"));
}

// Test that mimic tags in different model instances that refer to joints that
// have colliding names with joints in other model instances don't produce an
// error.
TEST_F(UrdfParserTest, MimicDifferentModelInstances) {
  // Currently the <mimic> tag is only supported by SAP.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child0'/>
      <link name='child1'/>
      <joint name='joint0' type='revolute'>
        <parent link='parent'/>
        <child link='child0'/>
      </joint>
      <joint name='joint1' type='revolute'>
        <parent link='parent'/>
        <child link='child1'/>
        <mimic joint='joint0' multiplier='1' offset='0' />
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt /* valid model instance index was parsed */);
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='b'>
      <link name='parent'/>
      <link name='child0'/>
      <link name='child1'/>
      <joint name='joint0' type='revolute'>
        <parent link='parent'/>
        <child link='child0'/>
      </joint>
      <joint name='joint1' type='revolute'>
        <parent link='parent'/>
        <child link='child1'/>
        <mimic joint='joint0' multiplier='1' offset='0' />
      </joint>
    </robot>)""",
                                   ""),
            std::nullopt /* valid model instance index was parsed */);
  EXPECT_EQ(NumErrors(), 0);
  EXPECT_EQ(NumWarnings(), 0);
}

TEST_F(UrdfParserTest, Material) {
  // Material parsing is tested fully elsewhere (see ParseMaterial()). This
  // test is just proof-of-life that top-level material stanzas are recognized.
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
     <material name='black'>
      <color rgba='0 0 0 0'/>
     </material>
     <link name="base_link">
      <visual>
       <geometry>
        <cylinder length="0.6" radius="0.2"/>
       </geometry>
       <!-- use the named material! -->
       <material name='black'/>
      </visual>
     </link>
    </robot>)""",
                                   ""),
            std::nullopt);
}

TEST_F(UrdfParserTest, FrameNameBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <frame naQQQme="broken" link="A" rpy="0 0 0" xyz="0 0 0"/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(), MatchesRegex(".*parsing frame name."));
}

TEST_F(UrdfParserTest, FrameLinkBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <frame name="frameA" liQQQnk="broken" rpy="0 0 0" xyz="0 0 0"/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*missing link name for frame frameA."));
}

TEST_F(UrdfParserTest, TransmissionTypeBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission tyQQQpe='broken'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Transmission element is missing a type."));
}

TEST_F(UrdfParserTest, TransmissionTypeUnknown) {
  std::string warning_pattern =
      ".*A <transmission> has a type that isn't 'SimpleTransmission'."
      " Drake only supports 'SimpleTransmission'; all other transmission"
      " types will be ignored.";

  // Express unknown type as attribute.
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission type='who'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(), MatchesRegex(warning_pattern));

  // Express unknown type as element.
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="b">
      <transmission>
        <type>who</type>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(), MatchesRegex(warning_pattern));
}

TEST_F(UrdfParserTest, TransmissionActuatorMissing) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission type='SimpleTransmission'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Transmission is missing an actuator element."));
}

TEST_F(UrdfParserTest, TransmissionActuatorNameBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission type='SimpleTransmission'>
        <actuator naQQQme='broken'/>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Transmission is missing an actuator name."));
}

TEST_F(UrdfParserTest, TransmissionJointMissing) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission type='SimpleTransmission'>
        <actuator name='a'/>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Transmission is missing a joint element."));
}

TEST_F(UrdfParserTest, TransmissionJointNameBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission type='SimpleTransmission'>
        <actuator name='a'/>
        <joint naQQQme='broken'/>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Transmission is missing a joint name."));
}

TEST_F(UrdfParserTest, TransmissionJointNotExist) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="a">
      <transmission type='SimpleTransmission'>
        <actuator name='a'/>
        <joint name='nowhere'/>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(), MatchesRegex(".*Transmission specifies joint"
                                        " 'nowhere' which does not exist."));
}

TEST_F(UrdfParserTest, TransmissionJointBadLimits) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <limit {}/>
      </joint>
      <transmission type='SimpleTransmission'>
        <actuator name='a'/>
        <joint name='a'/>
      </transmission>
    </robot>)""";
  EXPECT_NE(AddModelFromUrdfString(fmt::format(base, "effort='0'"), ""),
            std::nullopt);
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*Skipping transmission since it's attached to joint \"a\""
                   " which has a zero effort limit 0.*"));

  EXPECT_NE(AddModelFromUrdfString(fmt::format(base, "effort='-3'"), "b"),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Transmission specifies joint 'a' which has a negative"
                   " effort limit."));

  const auto attrs = std::array<std::string, 5>{"lower", "upper", "velocity",
                                                "drake:acceleration", "effort"};
  for (const auto& attr : attrs) {
    EXPECT_NE(AddModelFromUrdfString(fmt::format(base, attr + "='1 2'"), attr),
              std::nullopt);
    EXPECT_THAT(TakeError(),
                MatchesRegex(".*Expected single value.*" + attr + ".*"));
  }
}

// Verifies that the URDF loader can leverage a specified package map.
TEST_F(UrdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  ASSERT_EQ(plant_.num_model_instances(), 2);

  const std::string full_urdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/urdfs/box.urdf");
  std::filesystem::path package_path = full_urdf_filename;
  package_path = package_path.parent_path();
  package_path = package_path.parent_path();

  // Construct the PackageMap.
  package_map_.PopulateFromFolder(package_path.string());

  // Read in the URDF file.
  AddModelFromUrdfFile(full_urdf_filename, "");
  plant_.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant_.num_model_instances(), 3);
}

TEST_F(UrdfParserTest, DoublePendulum) {
  std::string full_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/double_pendulum.urdf");
  AddModelFromUrdfFile(full_name, "");
  plant_.Finalize();

  EXPECT_EQ(plant_.num_bodies(), 4);
  EXPECT_EQ(plant_.num_frames(), 10);

  ASSERT_TRUE(plant_.HasFrameNamed("frame_on_link1"));
  ASSERT_TRUE(plant_.HasFrameNamed("frame_on_link2"));
  ASSERT_TRUE(plant_.HasFrameNamed("link1_com"));
  ASSERT_TRUE(plant_.HasFrameNamed("link2_com"));

  // Sample a couple of frames.
  const Frame<double>& frame_on_link1 = plant_.GetFrameByName("frame_on_link1");
  EXPECT_EQ(frame_on_link1.body().index(),
            plant_.GetBodyByName("link1").index());

  math::RollPitchYaw<double> rpy_expected(-1, 0.1, 0.2);
  Vector3d xyz_expected(0.8, -0.2, 0.3);
  math::RigidTransform<double> X_BF_expected(rpy_expected.ToRotationMatrix(),
                                             xyz_expected);

  EXPECT_TRUE(
      CompareMatrices(frame_on_link1.GetFixedPoseInBodyFrame().GetAsMatrix4(),
                      X_BF_expected.GetAsMatrix4(), 1e-10));

  const Frame<double>& link2_com = plant_.GetFrameByName("link2_com");
  EXPECT_EQ(link2_com.body().index(), plant_.GetBodyByName("link2").index());
}

// This test verifies that we're able to successfully look up meshes using the
// `package://` syntax internally to the URDF (at least for packages which are
// successfully found in the same directory at the URDF.
TEST_F(UrdfParserTest, TestAtlasMinimalContact) {
  const std::string full_name =
      FindRunfile("drake_models/atlas/atlas_minimal_contact.urdf").abspath;
  AddModelFromUrdfFile(full_name, "");
  for (int k = 0; k < 30; k++) {
    EXPECT_THAT(TakeWarning(), MatchesRegex(".*safety_controller.*ignored.*"));
  }
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*attached to a fixed joint.*"));
  plant_.Finalize();

  EXPECT_EQ(plant_.num_positions(), 37);
  EXPECT_EQ(plant_.num_velocities(), 36);

  // Verify that joint actuator limits are set correctly.
  ASSERT_TRUE(plant_.HasJointActuatorNamed("back_bkz_motor"));
  const JointActuator<double>& actuator =
      plant_.GetJointActuatorByName("back_bkz_motor");
  EXPECT_EQ(actuator.effort_limit(), 106);
}

TEST_F(UrdfParserTest, TestAddWithQuaternionFloatingDof) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string model_file =
      FindResourceOrThrow(resource_dir + "zero_dof_robot.urdf");
  AddModelFromUrdfFile(model_file, "");
  plant_.Finalize();

  EXPECT_EQ(plant_.num_positions(), 7);
  EXPECT_EQ(plant_.num_velocities(), 6);
}

TEST_F(UrdfParserTest, TestRegisteredSceneGraph) {
  const std::string full_name =
      FindRunfile("drake_models/atlas/atlas_minimal_contact.urdf").abspath;
  // Test that registration with scene graph results in visual geometries.
  AddModelFromUrdfFile(full_name, "");
  // Mostly ignore warnings here; they are tested in detail elsewhere.
  EXPECT_GT(warning_records_.size(), 30);
  warning_records_.clear();
  plant_.Finalize();
  EXPECT_NE(plant_.num_visual_geometries(), 0);
}

TEST_F(UrdfParserTest, JointParsingTest) {
  // We currently need kSap for the mimic element to parse without error.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test.urdf");
  AddModelFromUrdfFile(full_name, "");
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*has a zero effort limit.*"));
  plant_.Finalize();

  // Revolute joint
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<RevoluteJoint>("revolute_joint"));
  const RevoluteJoint<double>& revolute_joint =
      plant_.GetJointByName<RevoluteJoint>("revolute_joint");
  EXPECT_EQ(revolute_joint.name(), "revolute_joint");
  EXPECT_EQ(revolute_joint.parent_body().name(), "link1");
  EXPECT_EQ(revolute_joint.child_body().name(), "link2");
  EXPECT_EQ(revolute_joint.revolute_axis(), Vector3d::UnitZ());
  EXPECT_EQ(revolute_joint.default_damping(), 0.1);
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_lower_limits(), Vector1d(-1)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_upper_limits(), Vector1d(2)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_lower_limits(), Vector1d(-100)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_upper_limits(), Vector1d(100)));
  EXPECT_TRUE(CompareMatrices(revolute_joint.acceleration_lower_limits(),
                              Vector1d(-200)));
  EXPECT_TRUE(CompareMatrices(revolute_joint.acceleration_upper_limits(),
                              Vector1d(200)));

  // Revolute actuator
  const JointActuator<double>& revolute_actuator =
      plant_.GetJointActuatorByName("revolute_actuator");
  EXPECT_EQ(revolute_actuator.effort_limit(), 100);

  // Prismatic joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<PrismaticJoint>("prismatic_joint"));
  const PrismaticJoint<double>& prismatic_joint =
      plant_.GetJointByName<PrismaticJoint>("prismatic_joint");
  EXPECT_EQ(prismatic_joint.name(), "prismatic_joint");
  EXPECT_EQ(prismatic_joint.parent_body().name(), "link2");
  EXPECT_EQ(prismatic_joint.child_body().name(), "link3");
  EXPECT_EQ(prismatic_joint.translation_axis(), Vector3d::UnitZ());
  EXPECT_EQ(prismatic_joint.default_damping(), 0.1);
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_lower_limits(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_upper_limits(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_lower_limits(), Vector1d(-5)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_upper_limits(), Vector1d(5)));
  EXPECT_TRUE(CompareMatrices(prismatic_joint.acceleration_lower_limits(),
                              Vector1d(-10)));
  EXPECT_TRUE(CompareMatrices(prismatic_joint.acceleration_upper_limits(),
                              Vector1d(10)));
  EXPECT_FALSE(plant_.HasJointActuatorNamed("prismatic_actuator"));

  // Ball joint
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<BallRpyJoint>("ball_joint"));
  const BallRpyJoint<double>& ball_joint =
      plant_.GetJointByName<BallRpyJoint>("ball_joint");
  EXPECT_EQ(ball_joint.name(), "ball_joint");
  EXPECT_EQ(ball_joint.parent_body().name(), "link3");
  EXPECT_EQ(ball_joint.child_body().name(), "link4");
  EXPECT_EQ(ball_joint.default_damping(), 0.1);
  const Vector3d inf3(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  const Vector3d neg_inf3(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(ball_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.velocity_upper_limits(), inf3));
  EXPECT_GT(ball_joint.index(), prismatic_joint.index());

  // Limitless revolute joint
  const Joint<double>& no_limit_joint =
      plant_.GetJointByName("revolute_joint_no_limits");
  const Vector1d inf(std::numeric_limits<double>::infinity());
  const Vector1d neg_inf(-std::numeric_limits<double>::infinity());

  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(no_limit_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.acceleration_upper_limits(), inf));
  EXPECT_GT(no_limit_joint.index(), ball_joint.index());

  // Limitless revolute actuator
  const JointActuator<double>& revolute_actuator_no_limits =
      plant_.GetJointActuatorByName("revolute_actuator_no_limits");
  EXPECT_EQ(revolute_actuator_no_limits.effort_limit(), inf(0));

  // Universal joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<UniversalJoint>("universal_joint"));
  const UniversalJoint<double>& universal_joint =
      plant_.GetJointByName<UniversalJoint>("universal_joint");
  EXPECT_EQ(universal_joint.name(), "universal_joint");
  EXPECT_EQ(universal_joint.parent_body().name(), "link5");
  EXPECT_EQ(universal_joint.child_body().name(), "link6");
  EXPECT_EQ(universal_joint.default_damping(), 0.1);
  const Vector2d inf2(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  const Vector2d neg_inf2(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(
      CompareMatrices(universal_joint.position_lower_limits(), neg_inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.position_upper_limits(), inf2));
  EXPECT_TRUE(
      CompareMatrices(universal_joint.velocity_lower_limits(), neg_inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.velocity_upper_limits(), inf2));

  // Planar joint
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<PlanarJoint>("planar_joint"));
  const PlanarJoint<double>& planar_joint =
      plant_.GetJointByName<PlanarJoint>("planar_joint");
  EXPECT_EQ(planar_joint.name(), "planar_joint");
  EXPECT_EQ(planar_joint.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint.child_body().name(), "link7");
  EXPECT_TRUE(
      CompareMatrices(planar_joint.default_damping(), Vector3d::Constant(0.1)));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_upper_limits(), inf3));

  // Continuous joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<RevoluteJoint>("continuous_joint"));
  const RevoluteJoint<double>& continuous_joint =
      plant_.GetJointByName<RevoluteJoint>("continuous_joint");
  EXPECT_EQ(continuous_joint.name(), "continuous_joint");
  EXPECT_EQ(continuous_joint.parent_body().name(), "link7");
  EXPECT_EQ(continuous_joint.child_body().name(), "link8");
  EXPECT_EQ(continuous_joint.revolute_axis(), Vector3d::UnitZ());
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(continuous_joint.position_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(continuous_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.acceleration_upper_limits(), inf));

  // Screw joint
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<ScrewJoint>("screw_joint"));
  const ScrewJoint<double>& screw_joint =
      plant_.GetJointByName<ScrewJoint>("screw_joint");
  EXPECT_EQ(screw_joint.name(), "screw_joint");
  EXPECT_EQ(screw_joint.parent_body().name(), "link8");
  EXPECT_EQ(screw_joint.child_body().name(), "link9");
  EXPECT_EQ(screw_joint.screw_axis(), Vector3d::UnitX());
  EXPECT_EQ(screw_joint.screw_pitch(), 0.04);
  EXPECT_EQ(screw_joint.default_damping(), 0.1);
  EXPECT_TRUE(CompareMatrices(screw_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(screw_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.acceleration_upper_limits(), inf));

  // Revolute joint with mimic
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName("revolute_joint_with_mimic_forward_reference"));
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName("revolute_joint_with_mimic_backward_reference"));
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName("revolute_joint_to_mimic"));

  const Joint<double>& joint_with_mimic_forward_reference =
      plant_.GetJointByName("revolute_joint_with_mimic_forward_reference");
  const Joint<double>& joint_with_mimic_backward_reference =
      plant_.GetJointByName("revolute_joint_with_mimic_backward_reference");
  const Joint<double>& joint_to_mimic =
      plant_.GetJointByName("revolute_joint_to_mimic");

  EXPECT_EQ(plant_.num_constraints(), 2);
  EXPECT_EQ(plant_.num_coupler_constraints(), 2);

  // Use the fact that we know the order the <mimic> tags are parsed and there
  // for the order the constraints are created to check the correct constraint.
  std::vector<internal::CouplerConstraintSpec> specs;
  for (const auto& item : plant_.get_coupler_constraint_specs()) {
    specs.push_back(item.second);
  }
  std::sort(specs.begin(), specs.end(),
            [](const auto& specA, const auto& specB) {
              return specA.id < specB.id;
            });

  EXPECT_EQ(specs[0].joint0_index, joint_with_mimic_forward_reference.index());
  EXPECT_EQ(specs[0].joint1_index, joint_to_mimic.index());
  // These must be kept in sync with the values in joint_parsing_test.urdf.
  EXPECT_EQ(specs[0].gear_ratio, 1.23);
  EXPECT_EQ(specs[0].offset, 4.56);

  EXPECT_EQ(specs[1].joint0_index, joint_with_mimic_backward_reference.index());
  EXPECT_EQ(specs[1].joint1_index, joint_to_mimic.index());
  // These must be kept in sync with the values in joint_parsing_test.urdf.
  EXPECT_EQ(specs[1].gear_ratio, 6.54);
  EXPECT_EQ(specs[1].offset, 3.21);

  // periodic curvilinear joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_periodic"));
  const CurvilinearJoint<double>& curvilinear_joint =
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_periodic");
  EXPECT_EQ(curvilinear_joint.name(), "curvilinear_periodic");
  EXPECT_EQ(curvilinear_joint.parent_body().name(), "link12");
  EXPECT_EQ(curvilinear_joint.child_body().name(), "link13");
  EXPECT_EQ(curvilinear_joint.default_damping(), 0.1);
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.position_lower_limits(),
                      Vector1d(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.position_upper_limits(),
                      Vector1d(std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.velocity_lower_limits(),
                      Vector1d(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.velocity_upper_limits(),
                      Vector1d(std::numeric_limits<double>::infinity())));
  const PiecewiseConstantCurvatureTrajectory<double> joint_curve =
      curvilinear_joint.get_trajectory();
  EXPECT_EQ(joint_curve.is_periodic(), true);
  std::vector<double> breaks_expected{0., 1., 1. + M_PI, 1. + 2 * M_PI};
  std::vector<double> turning_rates_expected{0., 0.5, -0.5};

  PiecewiseConstantCurvatureTrajectory<double> joint_curve_expected{
      breaks_expected,   turning_rates_expected, Vector3d::UnitZ(),
      Vector3d::UnitX(), Vector3d::Zero(),       true};
  const double curve_length = joint_curve.length();
  const double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(curve_length, joint_curve_expected.length(), kEpsilon);
  EXPECT_TRUE(joint_curve.CalcPose(0.).IsNearlyEqualTo(
      joint_curve_expected.CalcPose(0.), kEpsilon));
  EXPECT_TRUE(joint_curve.CalcPose(curve_length - kEpsilon)
                  .IsNearlyEqualTo(
                      joint_curve_expected.CalcPose(curve_length - kEpsilon),
                      kEpsilon));
  EXPECT_TRUE(
      joint_curve.CalcPose(curve_length * 1.5)
          .IsNearlyEqualTo(joint_curve_expected.CalcPose(curve_length * 1.5),
                           kEpsilon));

  // non-periodic curvilinear planar joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_aperiodic"));
  const CurvilinearJoint<double>& curvilinear_joint2 =
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_aperiodic");
  const PiecewiseConstantCurvatureTrajectory<double> joint2_curve =
      curvilinear_joint2.get_trajectory();
  EXPECT_EQ(curvilinear_joint2.name(), "curvilinear_aperiodic");
  EXPECT_EQ(curvilinear_joint2.parent_body().name(), "link13");
  EXPECT_EQ(curvilinear_joint2.child_body().name(), "link14");
  EXPECT_EQ(curvilinear_joint2.default_damping(), 0.1);
  EXPECT_TRUE(CompareMatrices(curvilinear_joint2.position_lower_limits(),
                              Vector1d(0.)));
  EXPECT_TRUE(CompareMatrices(curvilinear_joint2.position_upper_limits(),
                              Vector1d(joint2_curve.length())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint2.velocity_lower_limits(),
                      Vector1d(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint2.velocity_upper_limits(),
                      Vector1d(std::numeric_limits<double>::infinity())));
  PiecewiseConstantCurvatureTrajectory<double> joint2_curve_expected{
      breaks_expected,   turning_rates_expected, Vector3d::UnitZ(),
      Vector3d::UnitX(), Vector3d::Zero(),       false};
  EXPECT_EQ(joint2_curve.is_periodic(), false);
  const double curve2_length = joint2_curve.length();
  EXPECT_NEAR(curve2_length, joint2_curve_expected.length(), kEpsilon);
  EXPECT_TRUE(joint2_curve.CalcPose(0.).IsNearlyEqualTo(
      joint2_curve_expected.CalcPose(0.), kEpsilon));
  EXPECT_TRUE(joint2_curve.CalcPose(curve_length - kEpsilon)
                  .IsNearlyEqualTo(
                      joint2_curve_expected.CalcPose(curve_length - kEpsilon),
                      kEpsilon));
  EXPECT_TRUE(
      joint2_curve.CalcPose(curve_length * 1.5)
          .IsNearlyEqualTo(joint2_curve_expected.CalcPose(curve_length * 1.5),
                           kEpsilon));
}

// Custom planar joints were not necessary, but long supported. See #18730.
TEST_F(UrdfParserTest, LegacyPlanarJointAsCustomTest) {
  constexpr const char* model = R"""(
    <robot name='a'>
      <link name="link1"/>
      <link name="link2"/>
      <drake:joint name="planar_joint" type="planar">
        <parent link="link1"/>
        <child link="link2"/>
      </drake:joint>
    </robot>)""";
  EXPECT_NE(AddModelFromUrdfString(model, ""), std::nullopt);
}

TEST_F(UrdfParserTest, JointParsingTagMismatchTest) {
  // Improperly declared joints.
  const std::string full_name_mismatch_1 = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test_tag_mismatch_1.urdf");
  AddModelFromUrdfFile(full_name_mismatch_1, "");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Joint fixed_joint of type fixed is a standard joint type,"
                   " and should be a <joint>"));

  const std::string full_name_mismatch_2 = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test_tag_mismatch_2.urdf");
  AddModelFromUrdfFile(full_name_mismatch_2, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Joint ball_joint of type ball is a custom joint"
                           " type, and should be a <drake:joint>"));
}

TEST_F(UrdfParserTest, JointParsingTagMissingScrewParametersTest) {
  // Screw joint with missing thread pitch parameter.
  const std::string full_name_missing_element = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test_missing_screw_thread_pitch.urdf");
  AddModelFromUrdfFile(full_name_missing_element, "");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*A screw joint is missing the <drake:screw_thread_pitch>"
                   " tag."));

  const std::string full_name_missing_attribute = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test_missing_screw_thread_pitch_attribute.urdf");
  AddModelFromUrdfFile(full_name_missing_attribute, "");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*A screw joint has a <drake:screw_thread_pitch> tag"
                   " that is missing the 'value' attribute."));
}

// We allow users to declare the "world" link for the purpose of declaring
// "anchored" geometry (visual and collision). Specifying inertial properties
// is not *strictly* an error -- a warning will be written to the console.
// We can't test the warning, but we'll confirm there's no error.
//
// As for the geometry, we'll simply confirm that the expected numbers of
// geometries get instantiated (with expected roles). We'll assume that because
// the geometry parsing got triggered, it is correct and ignore the other
// details.
TEST_F(UrdfParserTest, AddingGeometriesToWorldLink) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="world">
    <!-- Declaring mass properties on the "world" link is bad. But it won't
     cause the parser to throw. -->
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.2 0.3"/>
        <material>
          <color rgba="0.8 0.7 0.6 0.5"/>
        </material>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.25"/>
      </geometry>
    </collision>
  </link>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*<inertial> tag is being ignored.*"));

  const auto& inspector = scene_graph_.model_inspector();
  EXPECT_EQ(inspector.num_geometries(), 2);
  EXPECT_EQ(inspector.NumGeometriesForFrame(scene_graph_.world_frame_id()), 2);
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                scene_graph_.world_frame_id(), geometry::Role::kProximity),
            1);
  // This does not total three geometries; the sphere has two roles,
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                scene_graph_.world_frame_id(), geometry::Role::kIllustration),
            1);
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                scene_graph_.world_frame_id(), geometry::Role::kPerception),
            1);
}

TEST_F(UrdfParserTest, CurvilinearJointErrors0) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*missing.*drake:curves.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors1) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
      <drake:line_segment/>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*missing.*length.*attribute.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors2) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
      <drake:line_segment length="-22"/>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*negative.*length.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors3) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
      <drake:circular_arc radQQQius="2.0" angle="1.57079632679489661923"/>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*missing.*radius.*attribute.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors4) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
      <drake:circular_arc radius="-2.0" angle="1.57079632679489661923"/>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*negative.*radius.*attribute.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors5) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
      <drake:circular_arc radius="2.0" anQQQgle="1.57079632679489661923"/>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*missing.*angle.*attribute.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors6) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
      <drake:circQQQular_arc radius="2.0" angle="1.57079632679489661923"/>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*invalid.*circQQQular_arc.*"));
}

TEST_F(UrdfParserTest, CurvilinearJointErrors7) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="a"/>
  <link name="b"/>
  <drake:joint name="curvilinear_periodic" type="curvilinear">
    <origin xyz="3 2 1" rpy="0 0 0"/>
    <parent link="a"/>
    <child link="b"/>
    <drake:initial_tangent xyz="0 0 1"/>
    <drake:plane_normal xyz="1 0 0"/>
    <drake:is_periodic value="true"/>
    <dynamics damping="0.1"/>
    <drake:curves>
    </drake:curves>
  </drake:joint>
</robot>
)""";
  AddModelFromUrdfString(test_urdf, "urdf");
  EXPECT_THAT(TakeError(), MatchesRegex(".*empty.*curves.*"));
}

// Reports if the frame with the given id has a geometry with the given role
// whose name is the same as what ShapeType{}.type_name() would produce.
template <typename ShapeType>
::testing::AssertionResult FrameHasShape(geometry::FrameId frame_id,
                                         geometry::Role role,
                                         const SceneGraph<double>& scene_graph,
                                         const ShapeType& shape) {
  const auto& inspector = scene_graph.model_inspector();
  const std::string name{shape.type_name()};
  try {
    // Note: MBP prepends the model index to the geometry name; in this case
    // that model instance name is "test_robot".
    const geometry::GeometryId geometry_id =
        inspector.GetGeometryIdByName(frame_id, role, "test_robot::" + name);
    const std::string_view shape_type =
        inspector.GetShape(geometry_id).type_name();
    if (shape_type != name) {
      return ::testing::AssertionFailure() << fmt::format(
                 "Geometry with role {} has wrong shape type.\n"
                 "  Expected: {}\n"
                 "  Found: {}",
                 role, name, shape_type);
    }
  } catch (const std::exception& e) {
    return ::testing::AssertionFailure() << fmt::format(
               "Frame {} does not have a geometry with role {} and name {}.\n"
               "  Exception message: {}",
               frame_id, role, name, e.what());
  }
  return ::testing::AssertionSuccess();
}

class UrdfParsedGeometryTest : public UrdfParserTest {
 public:
  // Confirms that all supported geometries in an URDF file are registered. The
  // *details* of the geometries are ignored -- we assume that that
  // functionality is tested in detail_urdf_geometry_test.cc. This merely makes
  // sure that *that* functionality is exercised appropriately.
  void TestForParsedGeometry(const char* sdf_name, geometry::Role role) {
    const std::string full_name = FindResourceOrThrow(sdf_name);
    AddModelFromUrdfFile(full_name, "");
    plant_.Finalize();

    const auto frame_id =
        plant_.GetBodyFrameIdOrThrow(plant_.GetBodyByName("link1").index());

    const std::string mesh_uri = "drake/multibody/parsing/test/tri_cube.obj";

    // Note: the parameters for the various example shapes do not matter to this
    // test.
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Box{0.1, 0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Capsule{0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Convex{mesh_uri, 1.0}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Cylinder{0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Ellipsoid{0.1, 0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Mesh{mesh_uri, 1.0}));
    EXPECT_TRUE(
        FrameHasShape(frame_id, role, scene_graph_, geometry::Sphere{0.1}));
  }
};

TEST_F(UrdfParsedGeometryTest, CollisionGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "all_geometries_as_collision.urdf",
      geometry::Role::kProximity);
}

TEST_F(UrdfParsedGeometryTest, VisualGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "all_geometries_as_visual.urdf",
      geometry::Role::kPerception);
}

TEST_F(UrdfParserTest, TestVisualAndCollisionNameOverlap) {
  // The visual and collision namespaces are distinct; you can use the same name
  // for both without triggering any warnings related to renaming.
  std::string robot = R"""(
    <robot name='a'>
      <link name='b'>
        <visual name='hello'>
          <geometry><box size='1 2 3'/></geometry>
        </visual>
        <collision name='hello'>
          <geometry><box size='1 2 3'/></geometry>
        </collision>
      </link>
    </robot>)""";
  EXPECT_NE(AddModelFromUrdfString(robot, ""), std::nullopt);
  // The test criterion is no warnings, which is already automatically checked.
}

TEST_F(UrdfParserTest, EntireInertialTagOmitted) {
  // Test that parsing a link with no inertial tag yields the expected result
  // (mass = 0, ixx = ixy = ixz = iyy = iyz = izz = 0).
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='entire_inertial_tag_omitted'>
      <link name='entire_inertial_tag_omitted'/>
    </robot>)""",
                                   ""),
            std::nullopt);

  const auto& body = dynamic_cast<const RigidBody<double>&>(
      plant_.GetBodyByName("entire_inertial_tag_omitted"));
  EXPECT_EQ(body.default_mass(), 0.);
  EXPECT_TRUE(body.default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body.default_rotational_inertia().get_products().isZero());
}

TEST_F(UrdfParserTest, InertiaTagOmitted) {
  // Test that parsing a link with no inertia tag yields the expected result
  // (mass as specified, ixx = ixy = ixz = iyy = iyz = izz = 0).
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='inertia_tag_omitted'>
      <link name='inertia_tag_omitted'>
        <inertial>
          <mass value="2"/>
        </inertial>
      </link>
    </robot>)""",
                                   ""),
            std::nullopt);
  const auto& body = dynamic_cast<const RigidBody<double>&>(
      plant_.GetBodyByName("inertia_tag_omitted"));
  EXPECT_EQ(body.default_mass(), 2.);
  EXPECT_TRUE(body.default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body.default_rotational_inertia().get_products().isZero());
}

TEST_F(UrdfParserTest, MassTagOmitted) {
  // Test that parsing a link with no mass tag yields the expected result
  // (mass 0, inertia as specified). Note that, because the default mass is 0,
  // we specify zero inertia here - otherwise the parsing would fail (See
  // ZeroMassNonZeroInertia below).
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='mass_tag_omitted'>
      <link name='mass_tag_omitted'>
        <inertial>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
      </link>
    </robot>)""",
                                   ""),
            std::nullopt);
  const auto& body = dynamic_cast<const RigidBody<double>&>(
      plant_.GetBodyByName("mass_tag_omitted"));
  EXPECT_EQ(body.default_mass(), 0.);
  EXPECT_TRUE(body.default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body.default_rotational_inertia().get_products().isZero());
}

TEST_F(UrdfParserTest, MasslessBody) {
  // Test that massless bodies can be parsed.
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='has_massless_link'>
      <link name='massless_link'>
        <inertial>
          <mass value="0"/>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
      </link>
    </robot>)""",
                                   ""),
            std::nullopt);
  const auto& body = dynamic_cast<const RigidBody<double>&>(
      plant_.GetBodyByName("massless_link"));
  EXPECT_EQ(body.default_mass(), 0.);
  EXPECT_TRUE(body.default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body.default_rotational_inertia().get_products().isZero());
}

TEST_F(UrdfParserTest, PointMass) {
  // Test that point masses don't get sent through the massless body branch.
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='point_mass'>
      <link name='point_mass'>
        <inertial>
          <mass value="1"/>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
      </link>
    </robot>)""",
                                   ""),
            std::nullopt);
  const auto& body = dynamic_cast<const RigidBody<double>&>(
      plant_.GetBodyByName("point_mass"));
  EXPECT_EQ(body.default_mass(), 1.);
  EXPECT_TRUE(body.default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body.default_rotational_inertia().get_products().isZero());
}

TEST_F(UrdfParserTest, BadInertiaFormats) {
  // Test various mis-formatted inputs.
  constexpr const char* base = R"""(
    <robot name='point_mass'>
      <link name='point_mass'>
        <inertial>
          <mass {}/>
          <inertia {}/>
        </inertial>
      </link>
    </robot>)""";
  AddModelFromUrdfString(
      fmt::format(base, "value='1 2 3'",
                  "ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'"),
      "");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*value.*"));
  AddModelFromUrdfString(
      fmt::format(base, "value='1'",
                  "ixx='0 2 3' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'"),
      "a");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*ixx.*"));
  AddModelFromUrdfString(
      fmt::format(base, "value='1'",
                  "ixx='0' ixy='0 2 3' ixz='0' iyy='0' iyz='0' izz='0'"),
      "b");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*ixy.*"));
  AddModelFromUrdfString(
      fmt::format(base, "value='1'",
                  "ixx='0' ixy='0' ixz='0 2 3' iyy='0' iyz='0' izz='0'"),
      "c");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*ixz.*"));
  AddModelFromUrdfString(
      fmt::format(base, "value='1'",
                  "ixx='0' ixy='0' ixz='0' iyy='0 2 3' iyz='0' izz='0'"),
      "d");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*iyy.*"));
  AddModelFromUrdfString(
      fmt::format(base, "value='1'",
                  "ixx='0' ixy='0' ixz='0' iyy='0' iyz='0 2 3' izz='0'"),
      "e");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*iyz.*"));
  AddModelFromUrdfString(
      fmt::format(base, "value='1'",
                  "ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0 2 3'"),
      "f");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*izz.*"));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingGood) {
  // Test successful parsing.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");

  // MBP will always create a UniformGravityField, so the only other
  // ForceElement should be the LinearSpringDamper element parsed.
  EXPECT_EQ(plant_.num_force_elements(), 2);

  const LinearSpringDamper<double>& linear_spring_damper =
      plant_.GetForceElement<LinearSpringDamper>(ForceElementIndex(1));

  EXPECT_STREQ(linear_spring_damper.bodyA().name().c_str(), "A");
  EXPECT_STREQ(linear_spring_damper.bodyB().name().c_str(), "B");
  EXPECT_EQ(linear_spring_damper.bodyA().model_instance(),
            linear_spring_damper.model_instance());
  EXPECT_EQ(linear_spring_damper.p_AP(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(linear_spring_damper.p_BQ(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(linear_spring_damper.free_length(), 7.0);
  EXPECT_EQ(linear_spring_damper.stiffness(), 8.0);
  EXPECT_EQ(linear_spring_damper.damping(), 9.0);
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNoBody) {
  // Test missing body tag.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to find the "
                           "<drake:linear_spring_damper_body_B> tag"));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNoBodyName) {
  // Test missing body tag name attribute.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'name' attribute for the "
                           "<drake:linear_spring_damper_body_B> tag"));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNonExistentBody) {
  // Test non-existent body tag.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B name="C"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Body: C specified for "
                   "<drake:linear_spring_damper_body_B> does not exist in the "
                   "model."));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNoPosition) {
  // Test no body A position tag.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to find the "
                           "<drake:linear_spring_damper_p_AP> tag"));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNoPositionAttribute) {
  // Test no value attribute on the body A position tag.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'value' attribute for the "
                           "<drake:linear_spring_damper_p_AP> tag"));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingInvalidFreeLength) {
  // Test zero free length.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="0"/>
          <drake:linear_spring_damper_stiffness value="8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*The 'value' attribute for the "
                   "<drake:linear_spring_damper_free_length> tag must be "
                   "strictly positive."));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNoStiffness) {
  // Test with missing stiffness tag.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to find the "
                           "<drake:linear_spring_damper_stiffness> tag"));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingInvalidStiffness) {
  // Test negative stiffness.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness value="-8.0"/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'value' attribute for the "
                           "<drake:linear_spring_damper_stiffness> tag must be "
                           "non-negative."));
}

TEST_F(UrdfParserTest, LinearSpringDamperParsingNoStiffnessValue) {
  // Test negative stiffness.
  const std::string model_string = R"""(
    <robot name="Model">
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A name="A"/>
          <drake:linear_spring_damper_p_AP value="1 2 3"/>
          <drake:linear_spring_damper_body_B name="B"/>
          <drake:linear_spring_damper_p_BQ value="4 5 6"/>
          <drake:linear_spring_damper_free_length value="7.0"/>
          <drake:linear_spring_damper_stiffness/>
          <drake:linear_spring_damper_damping value="9.0"/>
        </drake:linear_spring_damper>
    </robot>)""";

  AddModelFromUrdfString(model_string, "");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'value' attribute for the "
                           "<drake:linear_spring_damper_stiffness> tag"));
}

TEST_F(UrdfParserTest, BushingParsing) {
  // Test successful parsing.
  const std::string good_bushing_model = R"""(
    <robot name="bushing_test">
        <link name='A'/>
        <link name='C'/>
        <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
        <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
        <drake:linear_bushing_rpy>
            <drake:bushing_frameA name="frameA"/>
            <drake:bushing_frameC name="frameC"/>
            <drake:bushing_torque_stiffness value="1 2 3"/>
            <drake:bushing_torque_damping   value="4 5 6"/>
            <drake:bushing_force_stiffness  value="7 8 9"/>
            <drake:bushing_force_damping    value="10 11 12"/>
        </drake:linear_bushing_rpy>
    </robot>)""";

  EXPECT_NE(AddModelFromUrdfString(good_bushing_model, "b1"), std::nullopt);
  EXPECT_NE(AddModelFromUrdfString(good_bushing_model, "b2"), std::nullopt);

  // MBP will always create a UniformGravityField, so the only other
  // ForceElement should be the LinearBushingRollPitchYaw element parsed.
  EXPECT_EQ(plant_.num_force_elements(), 3);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant_.GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(1));

  EXPECT_STREQ(bushing.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing.torque_stiffness_constants(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(bushing.torque_damping_constants(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(bushing.force_stiffness_constants(), Eigen::Vector3d(7, 8, 9));
  EXPECT_EQ(bushing.force_damping_constants(), Eigen::Vector3d(10, 11, 12));

  const LinearBushingRollPitchYaw<double>& bushing2 =
      plant_.GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(2));

  EXPECT_STREQ(bushing2.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing2.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing2.frameA().model_instance(), bushing2.model_instance());
  EXPECT_NE(bushing.model_instance(), bushing2.model_instance());
}

TEST_F(UrdfParserTest, BushingMissingFrameTag) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="bushing_test">
      <link name='A'/>
      <link name='C'/>
      <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
      <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA name="frameA"/>
        <!-- missing the drake:bushing_frameC tag -->
        <drake:bushing_torque_stiffness value="1 2 3"/>
        <drake:bushing_torque_damping   value="4 5 6"/>
        <drake:bushing_force_stiffness  value="7 8 9"/>
        <drake:bushing_force_damping    value="10 11 12"/>
      </drake:linear_bushing_rpy>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to find the <drake:bushing_frameC> tag"));
}

TEST_F(UrdfParserTest, BushingFrameTagNameBroken) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="bushing_test">
      <link name='A'/>
      <link name='C'/>
      <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
      <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA name="frameA"/>
        <!-- drake:bushing_frameC tag has broken name attribute -->
        <drake:bushing_frameC naQQQme="frameC"/>
        <drake:bushing_torque_stiffness value="1 2 3"/>
        <drake:bushing_torque_damping   value="4 5 6"/>
        <drake:bushing_force_stiffness  value="7 8 9"/>
        <drake:bushing_force_damping    value="10 11 12"/>
      </drake:linear_bushing_rpy>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'name' attribute for the"
                           " <drake:bushing_frameC> tag"));
}

TEST_F(UrdfParserTest, BushingFrameNotExist) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="bushing_test">
      <link name='A'/>
      <link name='C'/>
      <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
      <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA name="frameA"/>
        <drake:bushing_frameC name="frameZ"/>
        <!-- frameZ does not exist in the model -->
        <drake:bushing_torque_stiffness value="1 2 3"/>
        <drake:bushing_torque_damping   value="4 5 6"/>
        <drake:bushing_force_stiffness  value="7 8 9"/>
        <drake:bushing_force_damping    value="10 11 12"/>
      </drake:linear_bushing_rpy>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Frame: frameZ specified for <drake:bushing_frameC> does"
                   " not exist in the model."));
}

TEST_F(UrdfParserTest, BushingMissingDamping) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="bushing_test">
      <link name='A'/>
      <link name='C'/>
      <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
      <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA name="frameA"/>
        <drake:bushing_frameC name="frameC"/>
        <drake:bushing_torque_stiffness value="1 2 3"/>
        <!-- missing the drake:bushing_torque_damping tag -->
        <drake:bushing_force_stiffness  value="7 8 9"/>
        <drake:bushing_force_damping    value="10 11 12"/>
      </drake:linear_bushing_rpy>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Unable to find the <drake:bushing_torque_damping> tag"));
}

TEST_F(UrdfParserTest, BushingMissingValueAttribute) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name="bushing_test">
      <link name='A'/>
      <link name='C'/>
      <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
      <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA name="frameA"/>
        <drake:bushing_frameC name="frameC"/>
        <!-- missing `value` attribute -->
        <drake:bushing_torque_stiffness />
        <drake:bushing_torque_damping   value="4 5 6"/>
        <drake:bushing_force_stiffness  value="7 8 9"/>
        <drake:bushing_force_damping    value="10 11 12"/>
      </drake:linear_bushing_rpy>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'value' attribute for the"
                           " <drake:bushing_torque_stiffness> tag"));
}

TEST_F(UrdfParserTest, TendonConstraint) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='tendon_constraint_test'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='revolute_AB' type='revolute'>
        <axis xyz='0 0 1'/>
        <parent link='A'/>
        <child link='B'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <joint name='prismatic_BC' type='prismatic'>
        <axis xyz='0 0 1'/>
        <parent link='B'/>
        <child link='C'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <drake:tendon_constraint>
        <drake:tendon_constraint_joint name='revolute_AB' a='10'/>
        <drake:tendon_constraint_joint name='prismatic_BC' a='20'/>
        <drake:tendon_constraint_offset value="0.3"/>
        <drake:tendon_constraint_lower_limit value="-5.0"/>
        <drake:tendon_constraint_upper_limit value="5.0"/>
        <drake:tendon_constraint_stiffness value="0.4"/>
        <drake:tendon_constraint_damping value="0.09"/>
      </drake:tendon_constraint>
    </robot>)""",
                                   ""),
            std::nullopt);

  EXPECT_EQ(plant_.num_constraints(), 1);
  EXPECT_EQ(plant_.num_tendon_constraints(), 1);

  const std::map<MultibodyConstraintId, TendonConstraintSpec>&
      tendon_constraints = plant_.get_tendon_constraint_specs();

  ASSERT_EQ(ssize(tendon_constraints), 1);

  const MultibodyConstraintId id = tendon_constraints.begin()->first;
  const TendonConstraintSpec& tendon_constraint =
      tendon_constraints.begin()->second;

  ASSERT_EQ(ssize(tendon_constraint.joints), 2);
  ASSERT_EQ(ssize(tendon_constraint.a), 2);

  EXPECT_EQ(tendon_constraint.joints[0],
            plant_.GetJointByName("revolute_AB").index());
  EXPECT_EQ(tendon_constraint.a[0], 10.0);
  EXPECT_EQ(tendon_constraint.joints[1],
            plant_.GetJointByName("prismatic_BC").index());
  EXPECT_EQ(tendon_constraint.a[1], 20.0);

  EXPECT_EQ(tendon_constraint.offset, 0.3);
  EXPECT_EQ(tendon_constraint.lower_limit, -5.0);
  EXPECT_EQ(tendon_constraint.upper_limit, 5.0);
  EXPECT_EQ(tendon_constraint.stiffness, 0.4);
  EXPECT_EQ(tendon_constraint.damping, 0.09);
  EXPECT_EQ(tendon_constraint.id, id);
}

class BallConstraintTest : public UrdfParserTest {
 public:
  BallConstraintTest() {
    // TODO(joemasterjohn): Currently ball constraints are only supported in
    // SAP.
    // Add coverage for other solvers and continuous mode when available.
    plant_.set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);
  }

  void VerifyParameters(const std::string& body_A, const std::string& body_B,
                        const Vector3d& p_AP, const Vector3d& p_BQ) {
    std::string text = fmt::format(
        kTestString,
        fmt::format("<drake:ball_constraint_body_A name=\"{}\"/>", body_A),
        fmt::format("<drake:ball_constraint_body_B name=\"{}\"/>", body_B),
        fmt::format("<drake:ball_constraint_p_AP value=\"{} {} {}\"/>",
                    p_AP.x(), p_AP.y(), p_AP.z()),
        fmt::format("<drake:ball_constraint_p_BQ value=\"{} {} {}\"/>",
                    p_BQ.x(), p_BQ.y(), p_BQ.z()));
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

    const std::map<MultibodyConstraintId, BallConstraintSpec>&
        ball_constraints = plant_.get_ball_constraint_specs();
    ASSERT_EQ(ssize(ball_constraints), 1);

    const MultibodyConstraintId ball_id = ball_constraints.begin()->first;
    const BallConstraintSpec& ball_spec = ball_constraints.begin()->second;

    EXPECT_EQ(ball_id, ball_spec.id);
    EXPECT_EQ(ball_spec.body_A, plant_.GetBodyByName(body_A).index());
    EXPECT_EQ(ball_spec.body_B, plant_.GetBodyByName(body_B).index());
    EXPECT_EQ(ball_spec.p_AP, p_AP);
    EXPECT_EQ(ball_spec.p_BQ, p_BQ);
  }

  void ProvokeError(const std::optional<const std::string>& body_A,
                    const std::optional<const std::string>& body_B,
                    const std::optional<const Vector3d>& p_AP,
                    const std::optional<const Vector3d>& p_BQ,
                    const std::string& error_pattern) {
    std::string text = fmt::format(
        kTestString,
        body_A.has_value()
            ? fmt::format("<drake:ball_constraint_body_A name=\"{}\"/>",
                          body_A.value())
            : "",
        body_B.has_value()
            ? fmt::format("<drake:ball_constraint_body_B name=\"{}\"/>",
                          body_B.value())
            : "",
        p_AP.has_value()
            ? fmt::format("<drake:ball_constraint_p_AP value=\"{} {} {}\"/>",
                          p_AP.value().x(), p_AP.value().y(), p_AP.value().z())
            : "",
        p_BQ.has_value()
            ? fmt::format("<drake:ball_constraint_p_BQ value=\"{} {} {}\"/>",
                          p_BQ.value().x(), p_BQ.value().y(), p_BQ.value().z())
            : "");
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);
    EXPECT_THAT(TakeError(), MatchesRegex(error_pattern));
  }

 protected:
  // Common URDF string with format options for the two custom tags.
  static constexpr const char* kTestString = R"""(
    <robot name='ball_constraint_test'>
      <link name='A'/>
      <link name='B'/>
      <drake:ball_constraint>
        {0}
        {1}
        {2}
        {3}
      </drake:ball_constraint>
    </robot>)""";
};

TEST_F(BallConstraintTest, AllParameters) {
  // Test successful parsing of all parameters.
  VerifyParameters("A", "B", Vector3d(1, 2, 3), Vector3d(4, 5, 6));
}

TEST_F(BallConstraintTest, MissingBodyA) {
  ProvokeError({}, "B", Vector3d(1, 2, 3), Vector3d(4, 5, 6),
               ".*Unable to find the <drake:ball_constraint_body_A> tag");
}

TEST_F(BallConstraintTest, MissingBodyB) {
  ProvokeError("A", {}, Vector3d(1, 2, 3), Vector3d(4, 5, 6),
               ".*Unable to find the <drake:ball_constraint_body_B> tag");
}

TEST_F(BallConstraintTest, Missing_p_AP) {
  ProvokeError("A", "B", {}, Vector3d(4, 5, 6),
               ".*Unable to find the <drake:ball_constraint_p_AP> tag");
}

TEST_F(BallConstraintTest, Missing_p_BQ) {
  ProvokeError("A", "B", Vector3d(1, 2, 3), {},
               ".*Unable to find the <drake:ball_constraint_p_BQ> tag");
}

TEST_F(BallConstraintTest, InvalidBody) {
  ProvokeError(
      "INVALID", "B", Vector3d(1, 2, 3), Vector3d(4, 5, 6),
      ".*Body: INVALID specified for <drake:ball_constraint_body_A> does not"
      " exist in the model.");
}

class TendonConstraintTest : public UrdfParserTest {
 public:
  void VerifyParameters(
      const std::vector<std::pair<std::string, double>>& joints,
      const double offset, const double lower_limit, const double upper_limit,
      const double stiffness, const double damping) {
    std::string joint_text = "";
    for (const auto& [joint_name, joint_a] : joints) {
      joint_text +=
          fmt::format("<drake:tendon_constraint_joint name=\"{0}\" a=\"{1}\"/>",
                      joint_name, joint_a);
    }

    std::string text = fmt::format(
        kTestString, joint_text,
        fmt::format("<drake:tendon_constraint_offset value=\"{}\"/>", offset),
        fmt::format("<drake:tendon_constraint_lower_limit value=\"{}\"/>",
                    lower_limit),
        fmt::format("<drake:tendon_constraint_upper_limit value=\"{}\"/>",
                    upper_limit),
        fmt::format("<drake:tendon_constraint_stiffness value=\"{}\"/>",
                    stiffness),
        fmt::format("<drake:tendon_constraint_damping value=\"{}\"/>",
                    damping));
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

    const std::map<MultibodyConstraintId, TendonConstraintSpec>&
        tendon_constraints = plant_.get_tendon_constraint_specs();
    ASSERT_EQ(ssize(tendon_constraints), 1);

    const MultibodyConstraintId tendon_id = tendon_constraints.begin()->first;
    const TendonConstraintSpec& tendon_spec =
        tendon_constraints.begin()->second;

    EXPECT_EQ(tendon_id, tendon_spec.id);
    EXPECT_EQ(tendon_spec.offset, offset);
    EXPECT_EQ(tendon_spec.lower_limit, lower_limit);
    EXPECT_EQ(tendon_spec.upper_limit, upper_limit);
    EXPECT_EQ(tendon_spec.stiffness, stiffness);
    EXPECT_EQ(tendon_spec.damping, damping);
  }

  void ProvokeError(const std::vector<std::pair<std::string, double>>& joints,
                    const std::optional<double> offset,
                    const std::optional<double> lower_limit,
                    const std::optional<double> upper_limit,
                    const std::optional<double> stiffness,
                    const std::optional<double> damping,
                    const std::string& error_pattern) {
    std::string joint_text = "";
    for (const auto& [joint_name, joint_a] : joints) {
      joint_text +=
          fmt::format("<drake:tendon_constraint_joint name=\"{0}\" a=\"{1}\"/>",
                      joint_name, joint_a);
    }

    std::string text = fmt::format(
        kTestString, joint_text,
        offset.has_value()
            ? fmt::format("<drake:tendon_constraint_offset value=\"{}\"/>",
                          offset.value())
            : "",
        lower_limit.has_value()
            ? fmt::format("<drake:tendon_constraint_lower_limit value=\"{}\"/>",
                          lower_limit.value())
            : "",
        upper_limit.has_value()
            ? fmt::format("<drake:tendon_constraint_upper_limit value=\"{}\"/>",
                          upper_limit.value())
            : "",
        stiffness.has_value()
            ? fmt::format("<drake:tendon_constraint_stiffness value=\"{}\"/>",
                          stiffness.value())
            : "",
        damping.has_value()
            ? fmt::format("<drake:tendon_constraint_damping value=\"{}\"/>",
                          damping.value())
            : "");

    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);
    EXPECT_THAT(TakeError(), MatchesRegex(error_pattern));
  }

 protected:
  // Common URDF string with format options for the tendon constraint.
  static constexpr const char* kTestString = R"""(
    <robot name='tendon_constraint_test'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='revolute_AB' type='revolute'>
        <axis xyz='0 0 1'/>
        <parent link='A'/>
        <child link='B'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <joint name='prismatic_BC' type='prismatic'>
        <axis xyz='0 0 1'/>
        <parent link='B'/>
        <child link='C'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <drake:tendon_constraint>
        {0}
        {1}
        {2}
        {3}
        {4}
        {5}
      </drake:tendon_constraint>
    </robot>)""";
};

TEST_F(TendonConstraintTest, AllParameters) {
  // Test successful parsing of all parameters.
  VerifyParameters({{"revolute_AB", 0.0}, {"prismatic_BC", 0.0}}, 1.0, -2.0,
                   2.0, 5.0, 0.1);
}

TEST_F(TendonConstraintTest, MissingOffset) {
  ProvokeError({{"revolute_AB", 0.0}, {"prismatic_BC", 0.0}}, {}, -2.0, 2.0,
               5.0, 0.1,
               ".*Unable to find the <drake:tendon_constraint_offset> tag");
}

TEST_F(TendonConstraintTest, MissingLowerLimit) {
  ProvokeError(
      {{"revolute_AB", 0.0}, {"prismatic_BC", 0.0}}, 1.0, {}, 2.0, 5.0, 0.1,
      ".*Unable to find the <drake:tendon_constraint_lower_limit> tag");
}

TEST_F(TendonConstraintTest, MissingUpperLimit) {
  ProvokeError(
      {{"revolute_AB", 0.0}, {"prismatic_BC", 0.0}}, 1.0, -2.0, {}, 5.0, 0.1,
      ".*Unable to find the <drake:tendon_constraint_upper_limit> tag");
}

TEST_F(TendonConstraintTest, MissingStiffness) {
  ProvokeError({{"revolute_AB", 0.0}, {"prismatic_BC", 0.0}}, 1.0, -2.0, 2.0,
               {}, 0.1,
               ".*Unable to find the <drake:tendon_constraint_stiffness> tag");
}

TEST_F(TendonConstraintTest, MissingDamping) {
  ProvokeError({{"revolute_AB", 0.0}, {"prismatic_BC", 0.0}}, 1.0, -2.0, 2.0,
               5.0, {},
               ".*Unable to find the <drake:tendon_constraint_damping> tag");
}

TEST_F(TendonConstraintTest, InvalidJoint) {
  ProvokeError({{"not_a_joint", 0.0}, {"prismatic_BC", 0.0}}, 1.0, -2.0, 2.0,
               5.0, 0.1,
               ".*Joint 'not_a_joint' specified for "
               "<drake:tendon_constraint_joint> does not exist in the model.");
}

TEST_F(TendonConstraintTest, MissingJointName) {
  std::string text = fmt::format(
      kTestString,
      // no "name" attribute
      "<drake:tendon_constraint_joint a = '1.0'/>"
      "<drake:tendon_constraint_joint name='prismatic_BC' a='2.0'/>",
      "<drake:tendon_constraint_offset value='0.0'/>",
      "<drake:tendon_constraint_lower_limit value='-1.0'/>",
      "<drake:tendon_constraint_upper_limit value='1.0'/>",
      "<drake:tendon_constraint_stiffness value='1.0'/>",
      "<drake:tendon_constraint_damping value='10.0'/>");
  EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

  // Two errors are produced because 1) the name attribute fails to parse and
  // defaults to empty-string, and 2) the empty-string joint does not exist in
  // the model.
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The tag <drake:tendon_constraint_joint> does not "
                           "specify the required attribute \"name\"."));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:tendon_constraint>: Joint '' specified for "
          "<drake:tendon_constraint_joint> does not exist in the model."));
}

TEST_F(TendonConstraintTest, MissingJointCoeff) {
  std::string text = fmt::format(
      kTestString,
      // no "a" attribute
      "<drake:tendon_constraint_joint name='revolute_AB'/>"
      "<drake:tendon_constraint_joint name='prismatic_BC' a='2.0'/>",
      "<drake:tendon_constraint_offset value='0.0'/>",
      "<drake:tendon_constraint_lower_limit value='-1.0'/>",
      "<drake:tendon_constraint_upper_limit value='1.0'/>",
      "<drake:tendon_constraint_stiffness value='1.0'/>",
      "<drake:tendon_constraint_damping value='10.0'/>");
  EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'a' attribute for the "
                           "<drake:tendon_constraint_joint> tag"));
}

TEST_F(TendonConstraintTest, MissingValueAttribute) {
  std::string text = fmt::format(
      kTestString,
      "<drake:tendon_constraint_joint name='revolute_AB' a='1.0'/>"
      "<drake:tendon_constraint_joint name='prismatic_BC' a='2.0'/>",
      // no "value" attribute
      "<drake:tendon_constraint_offset not_value='0.0'/>",
      "<drake:tendon_constraint_lower_limit value='-1.0'/>",
      "<drake:tendon_constraint_upper_limit value='1.0'/>",
      "<drake:tendon_constraint_stiffness value='1.0'/>",
      "<drake:tendon_constraint_damping value='10.0'/>");
  EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unable to read the 'value' attribute for the"
                           " <drake:tendon_constraint_offset> tag"));
}

class ReflectedInertiaTest : public UrdfParserTest {
 public:
  void VerifyParameters(const std::string& rotor_inertia_text,
                        const std::string& gear_ratio_text,
                        double rotor_inertia, double gear_ratio) {
    std::string text =
        fmt::format(kTestString, rotor_inertia_text, gear_ratio_text);
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

    const JointActuator<double>& actuator =
        plant_.GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), rotor_inertia);
    EXPECT_EQ(actuator.default_gear_ratio(), gear_ratio);
  }

  void ProvokeError(const std::string& rotor_inertia_text,
                    const std::string& gear_ratio_text,
                    const std::string& error_pattern) {
    std::string text =
        fmt::format(kTestString, rotor_inertia_text, gear_ratio_text);
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);
    EXPECT_THAT(TakeError(), MatchesRegex(error_pattern));
  }

 protected:
  // Common URDF string with format options for the two custom tags.
  static constexpr const char* kTestString = R"""(
    <robot name='reflected_inertia_test'>
      <link name='A'/>
      <link name='B'/>
      <joint name='revolute_AB' type='revolute'>
        <axis xyz='0 0 1'/>
        <parent link='A'/>
        <child link='B'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <transmission>
        <type>transmission_interface/SimpleTransmission</type>
        <joint name='revolute_AB'>
          <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name='revolute_AB'>
          {0}
          {1}
        </actuator>
      </transmission>
    </robot>)""";
};

TEST_F(ReflectedInertiaTest, Both) {
  // Test successful parsing of both parameters.
  VerifyParameters("<drake:rotor_inertia value='1.5' />",
                   "<drake:gear_ratio value='300.0' />", 1.5, 300);
}

TEST_F(ReflectedInertiaTest, DefaultGearRatio) {
  // Test successful parsing of rotor_inertia and default value for gear_ratio.
  VerifyParameters("<drake:rotor_inertia value='1.5' />", "", 1.5, 1.0);
}

TEST_F(ReflectedInertiaTest, RotorInertiaNoValue) {
  ProvokeError("<drake:rotor_inertia />", "",
               ".*joint actuator revolute_AB's drake:rotor_inertia does not"
               " have a \"value\" attribute!");
}

TEST_F(ReflectedInertiaTest, RotorInertiaManyValues) {
  ProvokeError("<drake:rotor_inertia value='1 2 3'/>", "",
               ".*Expected single value.*value.*");
}

TEST_F(ReflectedInertiaTest, DefaultRotorInertia) {
  // Test successful parsing of gear_ratio and default value for rotor_inertia.
  VerifyParameters("", "<drake:gear_ratio value='300.0' />", 0.0, 300.0);
}

TEST_F(ReflectedInertiaTest, GearRatioNoValue) {
  ProvokeError("", "<drake:gear_ratio />",
               ".*joint actuator revolute_AB's drake:gear_ratio does not have"
               " a \"value\" attribute!");
}

TEST_F(ReflectedInertiaTest, GearRatioManyValues) {
  ProvokeError("<drake:gear_ratio value='1 2 3'/>", "",
               ".*Expected single value.*value.*");
}

class ControllerGainsTest : public UrdfParserTest {
 public:
  void VerifyParameters(const std::string& controller_gains_text,
                        double expected_p, double expected_d) {
    std::string text = fmt::format(kTestString, controller_gains_text);
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);

    const JointActuator<double>& actuator =
        plant_.GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.get_controller_gains().p, expected_p);
    EXPECT_EQ(actuator.get_controller_gains().d, expected_d);
  }

  void ProvokeError(const std::string& controller_gains_text,
                    const std::string& error_pattern) {
    std::string text = fmt::format(kTestString, controller_gains_text);
    EXPECT_NE(AddModelFromUrdfString(text, ""), std::nullopt);
    EXPECT_THAT(TakeError(), MatchesRegex(error_pattern));
  }

 protected:
  // Common URDF string with format options for the custom tag with two
  // attributes.
  static constexpr const char* kTestString = R"""(
    <robot name='reflected_inertia_test'>
      <link name='A'/>
      <link name='B'/>
      <joint name='revolute_AB' type='revolute'>
        <axis xyz='0 0 1'/>
        <parent link='A'/>
        <child link='B'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <transmission>
        <type>transmission_interface/SimpleTransmission</type>
        <joint name='revolute_AB'>
          <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name='revolute_AB'>
          {0}
        </actuator>
      </transmission>
    </robot>)""";
};

TEST_F(ControllerGainsTest, Both) {
  // Test successful parsing of both parameters.
  VerifyParameters("<drake:controller_gains p='10000' d='100'/>", 10000, 100);
}

TEST_F(ControllerGainsTest, MissingP) {
  ProvokeError(
      "<drake:controller_gains d='100'/>",
      ".*joint actuator revolute_AB's drake:controller_gains does not have a"
      " \'p\' attribute!");
}

TEST_F(ControllerGainsTest, MissingD) {
  ProvokeError(
      "<drake:controller_gains p='10000'/>",
      ".*joint actuator revolute_AB's drake:controller_gains does not have a"
      " \'d\' attribute!");
}

TEST_F(ControllerGainsTest, PTooManyValues) {
  ProvokeError("<drake:controller_gains p='1 2 3' d='100'/>",
               ".*Expected single value for attribute 'p'.*");
}

TEST_F(ControllerGainsTest, DTooManyValues) {
  ProvokeError("<drake:controller_gains p='10000' d='1 2 3'/>",
               ".*Expected single value for attribute 'd'.*");
}

// TODO(SeanCurtis-TRI) The logic testing for collision filter group parsing
// belongs in detail_common_test.cc. Urdf and Sdf parsing just need enough
// testing to indicate that the method is being invoked correctly.
TEST_F(UrdfParserTest, CollisionFilterGroupParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "collision_filter_group_parsing_test.urdf");
  AddModelFromUrdfFile(full_name, "");

  // Get geometry ids for all the bodies.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph_.model_inspector();
  static constexpr int kNumLinks = 10;
  std::vector<GeometryId> ids(1 + kNumLinks);  // allow 1-based indices.
  for (int k = 1; k <= kNumLinks; ++k) {
    const auto geometry_id = inspector.GetGeometryIdByName(
        plant_.GetBodyFrameIdOrThrow(
            plant_.GetBodyByName(fmt::format("link{}", k)).index()),
        geometry::Role::kProximity,
        fmt::format("collision_filter_group_parsing_test::link{}_sphere", k));
    ids[k] = geometry_id;
  }

  // Make sure the plant is not finalized such that the adjacent joint filter
  // has not taken into effect yet. This guarantees that the collision filtering
  // is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant_.is_finalized());

  // We have six geometries and 15 possible pairs, each with a particular
  // disposition.
  // (1, 2) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[2]));
  // (1, 3) - filtered by group_link3 ignores group_link14
  EXPECT_TRUE(inspector.CollisionFiltered(ids[1], ids[3]));
  // (1, 4) - filtered by group_link14 ignores itself
  EXPECT_TRUE(inspector.CollisionFiltered(ids[1], ids[4]));
  // (1, 5) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[5]));
  // (1, 6) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[6]));
  // (2, 3) - filtered by group_link2 ignores group_link3
  EXPECT_TRUE(inspector.CollisionFiltered(ids[2], ids[3]));
  // (2, 4) - unfiltered (although declared in an *ignored* self-filtering
  // group_link24).
  EXPECT_FALSE(inspector.CollisionFiltered(ids[2], ids[4]));
  // (2, 5) - filtered by group_link56 ignored group_link2
  EXPECT_TRUE(inspector.CollisionFiltered(ids[2], ids[5]));
  // (2, 6) - filtered by group_link56 ignored group_link2
  EXPECT_TRUE(inspector.CollisionFiltered(ids[2], ids[6]));
  // (3, 4) - filtered by group_link3 ignores group_link14
  EXPECT_TRUE(inspector.CollisionFiltered(ids[3], ids[4]));
  // (3, 5) - filtered by group_link56 ignored group_link3
  EXPECT_TRUE(inspector.CollisionFiltered(ids[3], ids[5]));
  // (3, 6) - filtered by group_link56 ignored group_link3
  EXPECT_TRUE(inspector.CollisionFiltered(ids[3], ids[6]));
  // (4, 5) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[4], ids[5]));
  // (4, 6) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[4], ids[6]));
  // (5, 6) - filtered by group_link56 ignores itself
  EXPECT_TRUE(inspector.CollisionFiltered(ids[5], ids[6]));

  // To test composite group building, we have a separate set of geometries
  // 7-10. They are combined into a single self-ignoring group.
  EXPECT_TRUE(inspector.CollisionFiltered(ids[7], ids[8]));
  EXPECT_TRUE(inspector.CollisionFiltered(ids[7], ids[9]));
  EXPECT_TRUE(inspector.CollisionFiltered(ids[7], ids[10]));
  EXPECT_TRUE(inspector.CollisionFiltered(ids[8], ids[9]));
  EXPECT_TRUE(inspector.CollisionFiltered(ids[8], ids[10]));
  EXPECT_TRUE(inspector.CollisionFiltered(ids[9], ids[10]));

  // Spot check that the two sets are separate.
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[10]));
  EXPECT_FALSE(inspector.CollisionFiltered(ids[6], ids[7]));

  EXPECT_FALSE(last_parsed_groups_.empty());

  // Make sure we can add the model a second time.
  AddModelFromUrdfFile(full_name, "model2");
}

TEST_F(UrdfParserTest, CollisionFilterGroupMissingName) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot'>
      <link name='a'/>
        <drake:collision_filter_group/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*The tag <drake:collision_filter_group> does not specify"
                   " the required attribute \"name\"."));
}

TEST_F(UrdfParserTest, CollisionFilterGroupMissingLink) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot'>
      <link name='a'/>
      <drake:collision_filter_group name="group_a">
        <drake:member/>
      </drake:collision_filter_group>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*The tag <drake:member> does not specify the required "
                   "attribute \"link\"."));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'robot::group_a'.*no members"));
}

TEST_F(UrdfParserTest, CollisionFilterGroupMissingMemberGroupName) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot'>
      <link name='a'/>
      <drake:collision_filter_group name="group_a">
        <drake:member_group/>
      </drake:collision_filter_group>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The tag <drake:member_group> does not specify the"
                           " required attribute \"name\"."));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'robot::group_a'.*no members"));
}

TEST_F(UrdfParserTest, IgnoredCollisionFilterGroupMissingName) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot'>
      <link name='a'/>
      <drake:collision_filter_group name="group_a">
        <drake:ignored_collision_filter_group/>
      </drake:collision_filter_group>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeError(), MatchesRegex(".*'robot::group_a'.*no members"));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*The tag <drake:ignored_collision_filter_group> does not"
                   " specify the required attribute \"name\"."));
}

// Here follow tests to verify that Drake issues a warning when it ignores
// something thought to be a documented URDF element or attribute.

TEST_F(UrdfParserTest, UnsupportedVersionIgnored) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot' version='0.99'>
      <link name='a'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*version.*ignored.*"));
}

TEST_F(UrdfParserTest, UnsupportedLinkTypeIgnored) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot'>
      <link name='a' type='unknown'/>
    </robot>)""",
                                   ""),
            std::nullopt);
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*type.*link.*ignored.*"));
}

TEST_F(UrdfParserTest, UnsupportedJointStuffIgnored) {
  const std::array<std::string, 2> tags{"calibration", "safety_controller"};
  for (const auto& tag : tags) {
    EXPECT_NE(AddModelFromUrdfString(fmt::format(R"""(
    <robot>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
        <{}/>
      </joint>
    </robot>)""",
                                                 tag),
                                     tag),
              std::nullopt);
    EXPECT_THAT(TakeWarning(),
                MatchesRegex(fmt::format(".*{}.*ignored.*", tag)));
  }
}

TEST_F(UrdfParserTest, UnsupportedTransmissionStuffIgnored) {
  const std::array<std::string, 7> tags{"leftActuator",
                                        "rightActuator",
                                        "flexJoint",
                                        "rollJoint",
                                        "gap_joint",
                                        "passive_joint",
                                        "use_simulated_gripper_joint"};
  for (const auto& tag : tags) {
    EXPECT_NE(AddModelFromUrdfString(fmt::format(R"""(
    <robot>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
      <transmission type='SimpleTransmission'>
        <actuator name='a'/>
        <joint name='a'/>
        <{}/>
      </transmission>
    </robot>)""",
                                                 tag),
                                     tag),
              std::nullopt);
    EXPECT_THAT(TakeWarning(),
                MatchesRegex(fmt::format(".*{}.*ignored.*", tag)));
  }
}

// Here follow tests to verify that Drake is silent (as documented elsewhere)
// when it ignores something thought to be a documented URDF element or
// attribute.

TEST_F(UrdfParserTest, UnsupportedGazeboIgnoredSilent) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='robot'>
      <link name='a'/>
      <gazebo/>
    </robot>)""",
                                   ""),
            std::nullopt);
}

TEST_F(UrdfParserTest, UnsupportedTransmissionActuatorStuffIgnoredSilent) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
      <transmission type='SimpleTransmission'>
        <actuator name='a'>
          <hardwareInterface/>
        </actuator>
        <joint name='a'/>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
}

TEST_F(UrdfParserTest, UnsupportedTransmissionJointStuffIgnoredSilent) {
  EXPECT_NE(AddModelFromUrdfString(R"""(
    <robot name ='a'>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
      <transmission type='SimpleTransmission'>
        <actuator name='a'/>
        <joint name='a'>
          <hardwareInterface/>
        </joint>
      </transmission>
    </robot>)""",
                                   ""),
            std::nullopt);
}

// Here follow tests to verify that Drake applies special treatment (as
// documented elsewhere) when it ignores something thought to be a documented
// URDF element or attribute.

TEST_F(UrdfParserTest, UnsupportedMechanicalReductionIgnoredMaybe) {
  // Two substitution slots: actuator, then transmission.
  constexpr const char* robot_template = R"""(
    <robot>
      <link name='parent'/>
      <link name='child'/>
      <joint name='a' type='revolute'>
        <parent link='parent'/>
        <child link='child'/>
      </joint>
      <transmission type='SimpleTransmission'>
        <actuator name='a'>
          {}
        </actuator>
        <joint name='a'/>
        {}
      </transmission>
    </robot>)""";
  // Match the expected warning.
  constexpr char pattern[] = ".*mechanicalReduction.*default.*not.*support.*";

  struct Case {
    std::string input;
    bool provokes_warning{};
  };
  const std::array<Case, 8> cases{{
      {"<mechanicalReduction>3500.25</mechanicalReduction>", true},
      {"<mechanicalReduction>22 79 15</mechanicalReduction>", true},
      {"<mechanicalReduction>QQQ</mechanicalReduction>", true},
      {"<mechanicalReduction/>", false},
      {"<mechanicalReduction></mechanicalReduction>", false},
      {"<mechanicalReduction> </mechanicalReduction>", false},
      {"<mechanicalReduction>1</mechanicalReduction>", false},
      {"<mechanicalReduction>1.0</mechanicalReduction>", false},
  }};

  for (const Case& acase : cases) {
    // Within actuator.
    EXPECT_NE(
        AddModelFromUrdfString(fmt::format(robot_template, acase.input, ""),
                               acase.input + "_actuator"),
        std::nullopt);
    if (acase.provokes_warning) {
      EXPECT_THAT(TakeWarning(), MatchesRegex(pattern));
    }
    // Within transmission.
    EXPECT_NE(
        AddModelFromUrdfString(fmt::format(robot_template, "", acase.input),
                               acase.input + "_transmission"),
        std::nullopt);
    if (acase.provokes_warning) {
      EXPECT_THAT(TakeWarning(), MatchesRegex(pattern));
    }
  }
}

TEST_F(UrdfParserTest, PlanarJointAxisRespected) {
  constexpr const char* model = R"""(
    <robot name='a'>
      <link name="link1"/>
      <link name="link2"/>
      <drake:joint name="planar_joint" type="planar">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz = "0 1 0" />
      </drake:joint>
    </robot>)""";
  EXPECT_NE(AddModelFromUrdfString(model, ""), std::nullopt);
  plant_.Finalize();
  plant_.GetMutableJointByName<PlanarJoint>("planar_joint")
      .set_default_translation(Vector2<double>(1.2, 3.4));
  auto context = plant_.CreateDefaultContext();
  const math::RigidTransform<double>& X_WB =
      plant_.EvalBodyPoseInWorld(*context, plant_.GetBodyByName("link2"));
  const Vector3d p_WB_F(1.2, 3.4, 0.0);
  // The rotation that aligns +Mz with +By is the right-handed rotation around
  // Bx by 270 degrees.
  const math::RotationMatrixd R_BM =
      math::RotationMatrixd::MakeXRotation(3 * M_PI_2);
  // R_WB and R_MF are both identities.
  const math::RotationMatrixd R_WF = R_BM;
  const Vector3d p_WB = R_WF * p_WB_F;
  EXPECT_TRUE(CompareMatrices(X_WB.translation(), p_WB,
                              4.0 * std::numeric_limits<double>::epsilon()));
}

TEST_F(UrdfParserTest, PlanarJointCanonicalFrame) {
  constexpr const char* model = R"""(
    <robot name='a'>
      <link name="link1"/>
      <link name="link2"/>
      <drake:joint name="planar_joint" type="planar">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz = "0 0 1" />
      </drake:joint>
    </robot>)""";
  EXPECT_NE(AddModelFromUrdfString(model, ""), std::nullopt);
  plant_.Finalize();
  plant_.GetMutableJointByName<PlanarJoint>("planar_joint")
      .set_default_translation(Vector2<double>(1.2, 3.4));
  auto context = plant_.CreateDefaultContext();
  const math::RigidTransform<double>& X_WB =
      plant_.EvalBodyPoseInWorld(*context, plant_.GetBodyByName("link2"));
  // When the joint axis is (0, 0, 1), we shouldn't flip the x and y axes.
  const Vector3d expected_translation(1.2, 3.4, 0);
  EXPECT_EQ(X_WB.translation(), expected_translation);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
