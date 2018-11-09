#include "drake/multibody/rigid_body_constraint.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/symbolic.h"
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
      FindResourceOrThrow(
          "drake/examples/atlas/urdf/atlas_minimal_contact.urdf"),
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

GTEST_TEST(MinDistanceConstraintTests, PenaltyTest) {
  // Verifies that `MinDistanceConstraint::Penalty()`implements the following
  // piecewise function:
  //
  //           ⎧
  //           ⎪ (1 - x/d) exp(1/(x/d - 1)), x < d
  //    f(x) = ⎨
  //           ⎪                          0, x ≥ d
  //           ⎩

  symbolic::Variable distance_symbolic{"x"};
  symbolic::Variable distance_threshold_symbolic{"d"};
  symbolic::Expression penalty_above_threshold{0.};
  symbolic::Expression penalty_below_threshold{
      (1 - distance_symbolic / distance_threshold_symbolic) *
      exp(1 / (distance_symbolic / distance_threshold_symbolic - 1))};
  auto penalty_symbolic = symbolic::if_then_else(
      distance_symbolic < distance_threshold_symbolic,
      penalty_below_threshold, penalty_above_threshold);
  auto dpenalty_ddistance_symbolic = symbolic::if_then_else(
      distance_symbolic < distance_threshold_symbolic,
      penalty_below_threshold.Differentiate(distance_symbolic),
      penalty_above_threshold.Differentiate(distance_symbolic));
  constexpr int num_evaluation_points = 100;
  auto distance_numeric =
      VectorX<double>::LinSpaced(num_evaluation_points, -1, 1);
  VectorX<double> penalty;
  VectorX<double> dpenalty_ddistance;
  double tolerance = 128 * std::numeric_limits<double>::epsilon();
  for (double distance_threshold_numeric : {0.01, 0.1, 1.}) {
    MinDistanceConstraint::Penalty(distance_numeric, distance_threshold_numeric,
                                   &penalty, &dpenalty_ddistance);
    for (int i = 0; i < num_evaluation_points; ++i) {
      EXPECT_NEAR(
          penalty(i),
          penalty_symbolic.Evaluate(
              {{distance_symbolic, distance_numeric(i)},
               {distance_threshold_symbolic, distance_threshold_numeric}}),
          tolerance);
      EXPECT_NEAR(
          dpenalty_ddistance(i),
          dpenalty_ddistance_symbolic.Evaluate(
              {{distance_symbolic, distance_numeric(i)},
               {distance_threshold_symbolic, distance_threshold_numeric}}),
          tolerance);
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
