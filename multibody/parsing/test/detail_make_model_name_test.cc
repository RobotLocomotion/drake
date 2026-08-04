#include "drake/multibody/parsing/detail_make_model_name.h"

#include <string>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;

class MakeModelNameTest : public testing::Test {
 public:
  // Making model names does not involve parser delegation.
  static ParserInterface& NoSelect(const drake::internal::DiagnosticPolicy&,
                                   const std::string&) {
    DRAKE_UNREACHABLE();
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  DiagnosticPolicy policy_;
  MultibodyPlant<double> plant_{0.0};
  CollisionFilterGroupResolver resolver_{&plant_};
  ParsingWorkspace workspace_{options_, package_map_, policy_, nullptr,
                              &plant_,  &resolver_,   NoSelect};
};

TEST_F(MakeModelNameTest, Identity) {
  EXPECT_EQ("robot", MakeModelName("robot", {}, workspace_));
}

TEST_F(MakeModelNameTest, Prefix) {
  EXPECT_EQ("prefix::robot", MakeModelName("robot", "prefix", workspace_));
}

TEST_F(MakeModelNameTest, AutoRename) {
  options_.enable_auto_renaming = true;

  // Subscripts are compact and start at 1.
  plant_.AddModelInstance("robot");
  EXPECT_EQ("robot_1", MakeModelName("robot", {}, workspace_));
  plant_.AddModelInstance("robot_1");
  EXPECT_EQ("robot_2", MakeModelName("robot", {}, workspace_));

  // Subscripts of different base names are independent.
  plant_.AddModelInstance("thing");
  EXPECT_EQ("thing_1", MakeModelName("thing", {}, workspace_));
  plant_.AddModelInstance("thing_1");
  EXPECT_EQ("thing_2", MakeModelName("thing", {}, workspace_));
}

TEST_F(MakeModelNameTest, PrefixAndAutoRename) {
  options_.enable_auto_renaming = true;
  plant_.AddModelInstance("prefix::robot");
  EXPECT_EQ("prefix::robot_1", MakeModelName("robot", "prefix", workspace_));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
