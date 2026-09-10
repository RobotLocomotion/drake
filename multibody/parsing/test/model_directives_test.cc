#include "drake/multibody/parsing/model_directives.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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
