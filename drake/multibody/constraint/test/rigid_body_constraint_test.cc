#include "drake/multibody/constraint/rigid_body_constraint.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using std::cerr;
using std::endl;
using std::make_unique;

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(RigidBodyConstraintTest, TestWorldComConstraint) {
  auto tree = make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

  ASSERT_NE(tree, nullptr);

  Eigen::Vector2d tspan(0, 1);
  Eigen::Vector3d kc1_lb = Eigen::Vector3d::Zero();
  Eigen::Vector3d kc1_ub = Eigen::Vector3d::Zero();
  std::unique_ptr<WorldCoMConstraint> world_com_constraint;
  EXPECT_NO_THROW(world_com_constraint =
      make_unique<WorldCoMConstraint>(tree.get(), kc1_lb, kc1_ub, tspan));
  EXPECT_NE(world_com_constraint, nullptr);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
