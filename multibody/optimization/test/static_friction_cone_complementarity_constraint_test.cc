#include "drake/multibody/optimization/static_friction_cone_complementarity_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"

namespace drake {
namespace multibody {

const double kInf = std::numeric_limits<double>::infinity();

GTEST_TEST(StaticFrictionConeComplementarityNonlinearConstraint, TestEval) {
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

  double complementarity_tolerance{1E-3};
  internal::StaticFrictionConeComplementarityNonlinearConstraint constraint(
      &contact_wrench_evaluator, complementarity_tolerance);

  // Check the size of the constraint.
  EXPECT_EQ(constraint.num_outputs(), 4);
  // The variables are [q; λ; α; β].
  EXPECT_EQ(constraint.num_vars(), 7 + 3 + 2);

  // Check constraint bounds.
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector4d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(),
                      Eigen::Vector4d(kInf, 0, 0, complementarity_tolerance)));

  // Test update complementarity tolerance.
  complementarity_tolerance = 1E-4;
  constraint.UpdateComplementarityTolerance(complementarity_tolerance);

  // Check constraint bounds again after updating the complementarity tolerance.
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector4d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(),
                      Eigen::Vector4d(kInf, 0, 0, complementarity_tolerance)));

  // Evaluates the constraint for some arbitrary x.
  Eigen::VectorXd x_val(constraint.num_vars());
  x_val << 1.2, 2.4, 0.4, 3.2, 1.5, 0.45, 0.3, 0.2, 0.6, 0.5, 0.8, 1.3;
  Eigen::VectorXd y_val;
  constraint.Eval(x_val, &y_val);
  // Now evaluate the constraint manually. First retrieve each sub-vector from
  // x.
  Eigen::VectorXd q_val, lambda_val;
  double alpha_val, beta_val;
  constraint.DecomposeX<double>(x_val, &q_val, &lambda_val, &alpha_val,
                                &beta_val);
  EXPECT_TRUE(CompareMatrices(q_val, x_val.head<7>()));
  EXPECT_TRUE(CompareMatrices(lambda_val, x_val.segment<3>(7)));
  EXPECT_EQ(alpha_val, x_val(10));
  EXPECT_EQ(beta_val, x_val(11));

  Eigen::Vector4d y_val_expected;
  const Eigen::Vector3d n_W = -Eigen::Vector3d::UnitZ();
  y_val_expected(0) =
      lambda_val.dot(((std::pow(ground_friction.static_friction(), 2) + 1) *
                          n_W * n_W.transpose() -
                      Eigen::Matrix3d::Identity()) *
                     lambda_val);

  y_val_expected(1) = n_W.dot(lambda_val) - alpha_val;
  y_val_expected(2) = (q_val(6) - radius) - beta_val;
  y_val_expected(3) = alpha_val * beta_val;
  const double tol = 1E-14;
  EXPECT_TRUE(CompareMatrices(y_val, y_val_expected, tol));

  // Check gradient computation against numerical gradient.
  // Set the derivative of x to arbitrary value.
  Eigen::MatrixXd dx(12, 2);
  for (int i = 0; i < 12; ++i) {
    dx(i, 0) = i * 2 + 1;
    dx(i, 1) = 0.3 * i + 2;
  }
  auto x_ad = math::InitializeAutoDiff(x_val, dx);
  AutoDiffVecXd y_ad;
  constraint.Eval(x_ad, &y_ad);

  std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                     Eigen::VectorXd*)>
      eval_fun = [&constraint](const Eigen::Ref<const Eigen::VectorXd>& x,
                               Eigen::VectorXd* y) { constraint.Eval(x, y); };
  auto dy_dx = math::ComputeNumericalGradient(eval_fun, x_val);
  const double gradient_tol = 1E-5;
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_ad),
                              dy_dx * dx, gradient_tol));

  x_val.head<7>() << 1, 0, 0, 0, 0, 0, radius;
  // Try a friction force that is not in the friction cone. The constraint
  // should not be satisfied.
  x_val.segment<3>(7) << 1, 2, -1;
  // x(10) is alpha, equals to -f_W(z) = -x(9).
  x_val(10) = -x_val(9);
  // x(11) is beta, equals to x(6) - radius.
  x_val(11) = x_val(6) - radius;
  EXPECT_FALSE(constraint.CheckSatisfied(x_val));
  // Now test a friction force within the friction cone. The constraint should
  // be satisfied.
  x_val.segment<3>(7) << 1, 1, -1;
  EXPECT_TRUE(constraint.CheckSatisfied(x_val));
}
}  // namespace multibody
}  // namespace drake
