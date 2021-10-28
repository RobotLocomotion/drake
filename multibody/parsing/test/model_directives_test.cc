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
- add_weld:
    parent: parent_frame
    child: child_frame
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
)""";
  const auto directives = LoadYamlString<ModelDirectives>(contents);
  EXPECT_TRUE(directives.IsValid());
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
