#include <iostream>
#include <map>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
// TODO(liang.fok) Remove this once the layering violation of this file relying
// on drake/examples is resolved.
#include "drake/examples/examples_package_map.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parser_common.h"
#include "drake/multibody/parser_urdf.h"

using std::cerr;
using std::endl;
using std::make_unique;

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(RigidBodyConstraintTest, TestWorldComConstraint) {
  auto tree = make_unique<RigidBodyTree<double>>();
  parsers::PackageMap package_map;
  examples::AddExamplePackages(&package_map);
  parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf",
      package_map, multibody::joints::kRollPitchYaw,
      nullptr /* weld_to_frame */, tree.get());

  ASSERT_NE(tree, nullptr);

  Eigen::Vector2d tspan;
  tspan << 0, 1;
  Eigen::Vector3d kc1_lb, kc1_ub;
  kc1_lb << 0, 0, 0;
  kc1_ub << 0, 0, 0;
  std::unique_ptr<WorldCoMConstraint> world_com_constraint;
  EXPECT_NO_THROW(world_com_constraint =
      make_unique<WorldCoMConstraint>(tree.get(), kc1_lb, kc1_ub, tspan));
  EXPECT_NE(world_com_constraint, nullptr);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
