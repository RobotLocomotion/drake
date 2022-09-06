// Minimal test to make sure stuff doesn't explode.
#include "drake/multibody/parsing/model_directives.h"

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

using drake::yaml::LoadYamlString;

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

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
