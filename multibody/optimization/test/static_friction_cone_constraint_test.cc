#include "drake/multibody/optimization/static_friction_cone_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"
namespace drake {
namespace multibody {

const double kInf = std::numeric_limits<double>::infinity();

GTEST_TEST(StaticFrictionConeConstraint, TestEval) {
  const CoulombFriction<double> ground_friction(1.5 /* static friction */,
                                                0.8 /* dynamic friction */);

  const double radius = 0.1;
  const CoulombFriction<double> sphere_friction = ground_friction;

  test::FreeSpheresAndBoxes<AutoDiffXd> dut(
      {test::SphereSpecification(radius, 1E3, sphere_friction)},
      {} /* no box. */, ground_friction);

  ContactWrenchFromForceInWorldFrameEvaluator contact_wrench_evaluator(
      &(dut.plant()), dut.get_mutable_plant_context(),
      SortedPair<geometry::GeometryId>(dut.sphere_geometry_ids()[0],
                                       dut.ground_geometry_id()));

  StaticFrictionConeConstraint constraint(&contact_wrench_evaluator);

  // Check the size of the constraint.
  EXPECT_EQ(constraint.num_outputs(), 2);
  // The variables are [q; λ].
  EXPECT_EQ(constraint.num_vars(), 7 + 3);

  // Check constraint bounds.
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(), Eigen::Vector2d(kInf, kInf)));

  // Evaluate the constraint.
  Eigen::VectorXd x_val_satisfied(constraint.num_vars());
  // The first 7 entries are the generalized positions q, which is arbitrary.
  // The last 3 entries are the contact force from the ball to the ground,
  // expressed in the world frame.
  // First test a contact force within the friction cone.
  x_val_satisfied << 1.2, 2.4, 0.5, 3.2, 1.5, 0.1, 0.4, 0.1, 0.2, -1;
  EXPECT_TRUE(constraint.CheckSatisfied(x_val_satisfied));
  // friction force outside of friction cone.
  Eigen::VectorXd x_val_unsatisfied = x_val_satisfied;
  x_val_unsatisfied.tail<3>() << 2, 4, -1;
  EXPECT_FALSE(constraint.CheckSatisfied(x_val_unsatisfied));
  // The normal force points to the opposite direction of the friction cone.
  x_val_unsatisfied.tail<3>() << 0.1, 0.2, 1;
  EXPECT_FALSE(constraint.CheckSatisfied(x_val_unsatisfied));

  // Check the gradient computation against numerical gradient.
  // Set the derivative of x to arbitrary value.
  Eigen::MatrixXd dx(10, 2);
  for (int i = 0; i < 10; ++i) {
    dx(i, 0) = i * 2 + 0.1;
    dx(i, 1) = 0.5 * i + 2;
  }
  auto x_ad = math::InitializeAutoDiff(x_val_satisfied, dx);
  AutoDiffVecXd y_ad;
  constraint.Eval(x_ad, &y_ad);
  std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                     Eigen::VectorXd*)>
      eval_fun = [&constraint](const Eigen::Ref<const Eigen::VectorXd>& x,
                               Eigen::VectorXd* y) { constraint.Eval(x, y); };
  auto dy_dx = math::ComputeNumericalGradient(eval_fun, x_val_satisfied);
  const double gradient_tol = 1E-5;
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_ad),
                              dy_dx * dx, gradient_tol));
}
}  // namespace multibody
}  // namespace drake
