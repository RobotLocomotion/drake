#include "drake/multibody/parsing/model_directives.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/schema/rotation.h"
#include "drake/common/schema/stochastic.h"
#include "drake/common/schema/transform.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/yaml/yaml_io.h"

using drake::yaml::LoadYamlString;
using testing::MatchesRegex;

namespace drake {
namespace multibody {
namespace parsing {
namespace {

GTEST_TEST(ModelDirectivesTest, Success) {
  const char* contents = R"""(
directives:
- add_model:
    name: new_model
    file: base.sdf
    default_joint_positions:
      joint1: [0.3]
      joint2: [0.4, 0.5]
    default_free_body_pose:
      body1:
        translation: [1, 2, 3]
        rotation: !Rpy { deg: [5, 6, 7] }
      body2:
        translation: [-1, -2, -3]
        rotation: !Rpy { deg: [-5, -6, -7] }
- add_weld:
    parent: parent_frame
    child: child_frame
    X_PC:
      translation: [1, 2, 3]
      rotation: !Rpy { deg: [7, 8, 9]}
- add_frame:
    name: new_frame_a
    X_PF:
      base_frame: world
      translation: [1, 2, 3]
      rotation: !Rpy { deg: [5, 6, 7] }
- add_frame:
    name: new_frame_b
    X_PF:
      base_frame: world
      translation: [1, 2, 3]
      rotation: !Rpy { deg: [5, 6, 7] }
- add_directives:
    file: child.yaml
- add_directives:
    file: child.yaml
    model_namespace: right
- add_collision_filter_group:
    name: group1
    members: [new_model::link, right::robot::link]
    ignored_collision_filter_groups: [group1, right::robot::group]
)""";
  // Here we copy-paste the code from LoadModelDirectivesFromString so that we
  // can check IsValid with a test assertion, instead of a DRAKE_DEMAND.
  const ModelDirectives defaults;
  const auto directives = LoadYamlString<ModelDirectives>(
      contents, std::nullopt /* child_name */, defaults);
  EXPECT_TRUE(directives.IsValid());
}

class ModelDirectivesErrorTest : public test::DiagnosticPolicyTestBase {
 public:
  ModelDirectivesErrorTest() { RecordErrors(); }
};

TEST_F(ModelDirectivesErrorTest, AddModelEmptyFile) {
  AddModel dut;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_model: `file` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddModelEmptyName) {
  AddModel dut;
  dut.file = "base.sdf";
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_model: `name` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddWeldEmptyParent) {
  AddWeld dut;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_weld: `parent` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddWeldEmptyChild) {
  AddWeld dut;
  dut.parent = "parent_frame";
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_weld: `child` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddFrameEmptyName) {
  AddFrame dut;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_frame: `name` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddFrameMissingBaseFrame) {
  AddFrame dut;
  dut.name = "f";
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_frame: `X_PF.base_frame` must be defined.*"));
}

TEST_F(ModelDirectivesErrorTest, AddModelInstanceEmptyName) {
  AddModelInstance dut;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_model_instance: `name` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddDirectivesEmptyFile) {
  AddDirectives dut;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_directives: `file` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddWeldXpcBaseFrameRejected) {
  AddWeld dut;
  dut.parent = "parent_frame";
  dut.child = "child_frame";
  schema::Transform X_PC;
  X_PC.base_frame = "world";
  dut.X_PC = X_PC;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*add_weld: `X_PC` must not specify a `base_frame`.*"));
}

TEST_F(ModelDirectivesErrorTest, AddWeldXpcNonDeterministic) {
  AddWeld dut;
  dut.parent = "parent_frame";
  dut.child = "child_frame";
  schema::Transform X_PC;
  X_PC.rotation.value = schema::Rotation::Uniform{};
  dut.X_PC = X_PC;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*add_weld: `X_PC` must specify a deterministic.*"));
}

TEST_F(ModelDirectivesErrorTest, AddModelDefaultFreeBodyPoseNonDeterministic) {
  AddModel dut;
  dut.file = "base.sdf";
  dut.name = "m";
  schema::Transform pose;
  pose.rotation.value = schema::Rotation::Uniform{};
  dut.default_free_body_pose["body1"] = pose;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(), MatchesRegex(".*add_model: `default_free_body_pose` "
                                        "must specify a deterministic.*"));
}

TEST_F(ModelDirectivesErrorTest, AddFrameXpfNonDeterministic) {
  AddFrame dut;
  dut.name = "f";
  dut.X_PF.base_frame = "world";
  dut.X_PF.rotation.value = schema::Rotation::Uniform{};
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*add_frame: `X_PF` must specify a deterministic.*"));
}

TEST_F(ModelDirectivesErrorTest, AddCollisionFilterGroupEmptyName) {
  AddCollisionFilterGroup dut;
  dut.members = {"link"};
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*add_collision_filter_group: `name` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, AddCollisionFilterGroupEmptyMembers) {
  AddCollisionFilterGroup dut;
  dut.name = "group1";
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(
                  ".*add_collision_filter_group:.*at least one of `members` or "
                  "`member_groups` must be non-empty.*"));
}

TEST_F(ModelDirectivesErrorTest, ModelDirectiveNotUnique) {
  ModelDirective dut;
  // Zero alternatives set => not unique.
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(), MatchesRegex(".*directive: Specify one of.*"));

  AddModel add_model;
  add_model.file = "base.sdf";
  add_model.name = "m";
  AddModelInstance add_model_instance;
  add_model_instance.name = "mi";
  dut.add_model = add_model;
  dut.add_model_instance = add_model_instance;
  EXPECT_FALSE(dut.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(), MatchesRegex(".*directive: Specify one of.*"));
}

TEST_F(ModelDirectivesErrorTest, ModelDirectivesPropagatesErrors) {
  ModelDirectives directives;
  ModelDirective directive;
  AddModelInstance add_model_instance;
  add_model_instance.name = "";
  directive.add_model_instance = add_model_instance;
  directives.directives.push_back(directive);

  EXPECT_FALSE(directives.IsValid(diagnostic_policy_));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*add_model_instance: `name` must be non-empty.*"));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
