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

  Vector2<double> tspan(0, 1);
  Vector3<double> kc1_lb = Vector3<double>::Zero();
  Vector3<double> kc1_ub = Vector3<double>::Zero();
  std::unique_ptr<WorldCoMConstraint> world_com_constraint;
  EXPECT_NO_THROW(world_com_constraint = make_unique<WorldCoMConstraint>(
                      tree.get(), kc1_lb, kc1_ub, tspan));
  EXPECT_NE(world_com_constraint, nullptr);
}

GTEST_TEST(MinDistanceConstraintTests, PenaltyTest) {
  // Verifies that `MinDistanceConstraint::Penalty()` implements the following
  // piecewise function:
  //
  //           ⎧
  //           ⎪ - x exp(1/x), x < 0
  //    f(x) = ⎨
  //           ⎪            0, x ≥ 0
  //           ⎩

  symbolic::Variable distance_symbolic{"x"};
  symbolic::Expression cost_positive_distance{0.};
  symbolic::Expression cost_negative_distance{-distance_symbolic *
                                              exp(1 / distance_symbolic)};
  auto cost_symbolic = symbolic::if_then_else(
      distance_symbolic < 0, cost_negative_distance, cost_positive_distance);
  auto dcost_ddistance_symbolic = symbolic::if_then_else(
      distance_symbolic < 0,
      cost_negative_distance.Differentiate(distance_symbolic),
      cost_positive_distance.Differentiate(distance_symbolic));
  constexpr int num_evaluation_points = 100;
  auto distance_numeric =
      VectorX<double>::LinSpaced(num_evaluation_points, -1, 1);
  VectorX<double> cost;
  VectorX<double> dcost_ddistance;
  MinDistanceConstraint::Penalty(distance_numeric, &cost, &dcost_ddistance);
  for (int i = 0; i < num_evaluation_points; ++i) {
    EXPECT_NEAR(
        cost(i),
        cost_symbolic.Evaluate({{distance_symbolic, distance_numeric(i)}}),
        std::numeric_limits<double>::epsilon());
    EXPECT_NEAR(dcost_ddistance(i),
                dcost_ddistance_symbolic.Evaluate(
                    {{distance_symbolic, distance_numeric(i)}}),
                std::numeric_limits<double>::epsilon());
  }
}

GTEST_TEST(MinDistanceConstraintTests, ScaleDistanceTest) {
  // Verifies that `MinDistanceConstraint::ScaleDistance()` implements the
  // following function:
  //
  //    f(x, c) = c⋅x - 1
  //
  symbolic::Variable distance_symbolic{"x"};
  symbolic::Variable scaling_factor_symbolic{"c"};
  symbolic::Expression scaled_distance_symbolic{
      scaling_factor_symbolic * distance_symbolic - 1};
  constexpr int num_evaluation_points = 100;
  auto distance_numeric =
      VectorX<double>::LinSpaced(num_evaluation_points, -1, 1);
  VectorX<double> scaled_distance;
  double dscaled_distance_ddistance;
  for (double scaling_factor_numeric : {0., 0.1, 2.}) {
    MinDistanceConstraint::ScaleDistance(
        distance_numeric, scaling_factor_numeric, &scaled_distance,
        &dscaled_distance_ddistance);
    for (int i = 0; i < num_evaluation_points; ++i) {
      EXPECT_NEAR(scaled_distance(i),
                  scaled_distance_symbolic.Evaluate(
                      {{distance_symbolic, distance_numeric(i)},
                       {scaling_factor_symbolic, scaling_factor_numeric}}),
                  std::numeric_limits<double>::epsilon());
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
