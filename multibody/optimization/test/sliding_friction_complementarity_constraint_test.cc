#include "drake/multibody/optimization/sliding_friction_complementarity_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

const double kInf = std::numeric_limits<double>::infinity();

void CompareAutoDiff(const AutoDiffXd& x1, const AutoDiffXd& x2, double tol) {
  EXPECT_NEAR(x1.value(), x2.value(), tol);
  EXPECT_TRUE(CompareMatrices(x1.derivatives(), x2.derivatives(), tol));
}

void CompareAutoDiff(const Eigen::Ref<const AutoDiffVecXd>& x1,
                     const Eigen::Ref<const AutoDiffVecXd>& x2, double tol) {
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(x1),
                              math::ExtractValue(x2), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(x1),
                              math::ExtractGradient(x2), tol));
}

template <typename T>
void ComputeRelativeMotion(test::FreeSpheresAndBoxes<T>* spheres,
                           const Eigen::Ref<const VectorX<T>>& q,
                           const Eigen::Ref<const VectorX<T>>& v,
                           double sphere2_radius, Vector3<T>* nhat_SaSb_W,
                           Vector3<T>* v_SaCb_W) {
  const auto& plant = spheres->plant();
  plant.SetPositions(spheres->get_mutable_plant_context(), q);
  plant.SetVelocities(spheres->get_mutable_plant_context(), v);
  // Now compute v_SaCb_W, first I need to compute the poses of the two spheres.
  const math::RigidTransform<T> X_WSa = plant.CalcRelativeTransform(
      spheres->plant_context(), plant.world_frame(),
      plant.get_body(spheres->sphere_body_indices()[0]).body_frame());
  const math::RigidTransform<AutoDiffXd> X_WSb = plant.CalcRelativeTransform(
      spheres->plant_context(), plant.world_frame(),
      plant.get_body(spheres->sphere_body_indices()[1]).body_frame());

  *nhat_SaSb_W = (X_WSb.translation() - X_WSa.translation()).normalized();
  const Vector3<T> p_SbCb_W = -(*nhat_SaSb_W) * sphere2_radius;

  const SpatialVelocity<T> V_SaSb_W =
      plant.get_body(spheres->sphere_body_indices()[1])
          .body_frame()
          .CalcSpatialVelocity(
              spheres->plant_context(),
              plant.get_body(spheres->sphere_body_indices()[0]).body_frame(),
              plant.world_frame());

  const SpatialVelocity<T> V_SaCb_W = V_SaSb_W.Shift(p_SbCb_W);
  *v_SaCb_W = V_SaCb_W.translational();
}

GTEST_TEST(SlidingFrictionComplementarityNonlinearConstraintTest, Constructor) {
  const CoulombFriction<double> sphere1_friction(1.1, 0.9);
  const test::SphereSpecification sphere1_spec(0.1, 1e3, sphere1_friction);
  const CoulombFriction<double> sphere2_friction(1.2, 0.8);
  const test::SphereSpecification sphere2_spec(0.2, 1.2e3, sphere2_friction);
  const CoulombFriction<double> ground_friction(0.6, 0.5);
  test::FreeSpheresAndBoxes<AutoDiffXd> spheres(
      {sphere1_spec, sphere2_spec}, {} /* no box. */, ground_friction);

  ContactWrenchFromForceInWorldFrameEvaluator contact_wrench_evaluator(
      &(spheres.plant()), spheres.get_mutable_plant_context(),
      SortedPair<geometry::GeometryId>(spheres.sphere_geometry_ids()[0],
                                       spheres.sphere_geometry_ids()[1]));

  double complementarity_tolerance = 1E-3;
  SlidingFrictionComplementarityNonlinearConstraint constraint(
      &contact_wrench_evaluator, complementarity_tolerance);

  EXPECT_EQ(constraint.num_constraints(), 11);
  EXPECT_EQ(constraint.num_vars(),
            spheres.plant().num_positions() + spheres.plant().num_velocities() +
                contact_wrench_evaluator.num_lambda() + 7);
  Eigen::Matrix<double, 11, 1> lower_bound_expected, upper_bound_expected;
  lower_bound_expected.setZero();
  upper_bound_expected.setZero();
  lower_bound_expected.segment<3>(3).setConstant(-complementarity_tolerance);
  upper_bound_expected.segment<3>(3).setConstant(complementarity_tolerance);
  upper_bound_expected(6) = kInf;
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), lower_bound_expected));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), upper_bound_expected));

  // Test update complementarity tolerance.
  complementarity_tolerance = 1E-4;
  constraint.UpdateComplementarityTolerance(complementarity_tolerance);
  lower_bound_expected.segment<3>(3).setConstant(-complementarity_tolerance);
  upper_bound_expected.segment<3>(3).setConstant(complementarity_tolerance);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), lower_bound_expected));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), upper_bound_expected));

  // Test Eval for some arbitrary x.
  Eigen::VectorXd x_val(constraint.num_vars());
  x_val.head<14>() << 0.2, 1.4, 0.5, 0.3, 1.2, 0.7, -0.3, 0.4, -0.5, 0.2, 1.5,
      1.4, 0.8, -0.2;  // q
  x_val.segment<12>(14) << -0.3, 0.2, -1.5, 0.3, 0.2, 2.5, 0.2, -0.4, 0.3, 1.5,
      0.6, -2.3;                           // v
  x_val.segment<3>(26) << 0.2, 0.5, 1.5;   // lambda
  x_val.segment<3>(29) << 0.4, 0.2, -1.2;  // f_static
  x_val.segment<3>(32) << 0.3, 0.2, 0.9;   // f_sliding
  x_val(35) = 0.5;                         // c
  // Set the derivatives of x to some arbitrary value.
  Eigen::MatrixXd x_grad(x_val.rows(), 2);
  for (int i = 0; i < x_grad.rows(); ++i) {
    x_grad(i, 0) = 2 * i + 1;
    x_grad(i, 1) = 0.3 * std::sin(i) - 0.2;
  }
  const AutoDiffVecXd x_autodiff =
      math::InitializeAutoDiff(x_val, x_grad);

  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);

  AutoDiffVecXd q, v, lambda;
  Vector3<AutoDiffXd> f_static, f_sliding;
  AutoDiffXd c;
  constraint.DecomposeX<AutoDiffXd>(x_autodiff, &q, &v, &lambda, &f_static,
                                    &f_sliding, &c);

  // Now evaluate the constraint manually.
  const auto& plant = spheres.plant();
  plant.SetPositions(spheres.get_mutable_plant_context(), q);
  plant.SetVelocities(spheres.get_mutable_plant_context(), v);
  // First evaluate the contact force f_AB_W.
  const Vector3<AutoDiffXd> f_AB_W = lambda;
  const double tol = 1E-12;
  CompareAutoDiff(y_autodiff.head<3>(), f_AB_W - f_static - f_sliding, tol);
  // Now compute v_SaCb_W, first I need to compute the poses of the two spheres.
  Vector3<AutoDiffXd> n_SaSb_W, v_SaCb_W;
  ComputeRelativeMotion<AutoDiffXd>(&spheres, q, v, sphere2_spec.radius,
                                    &n_SaSb_W, &v_SaCb_W);
  const Vector3<AutoDiffXd> v_tangential_SaCb_W =
      (Eigen::Matrix3d::Identity() - n_SaSb_W * n_SaSb_W.transpose()) *
      v_SaCb_W;
  const AutoDiffXd f_static_normal_magnitude = f_static.dot(n_SaSb_W);

  CompareAutoDiff(y_autodiff.segment<3>(3),
                  v_tangential_SaCb_W * (f_static.dot(-n_SaSb_W)), tol);

  // Evaluate constraint (3)
  const CoulombFriction<double> combined_friction =
      CalcContactFrictionFromSurfaceProperties(sphere1_friction,
                                               sphere2_friction);
  CompareAutoDiff(y_autodiff(6), f_sliding.dot(n_SaSb_W), tol);

  const Vector3<AutoDiffXd> f_sliding_tangential =
      (Eigen::Matrix3d::Identity() - n_SaSb_W * n_SaSb_W.transpose()) *
      f_sliding;
  CompareAutoDiff(
      y_autodiff(7),
      f_sliding_tangential.squaredNorm() -
          pow((combined_friction.dynamic_friction() * f_sliding.dot(n_SaSb_W)),
              2),
      tol);

  // Evaluate constraint (4)
  CompareAutoDiff(y_autodiff.tail<3>(),
                  f_sliding_tangential + c * v_tangential_SaCb_W, tol);

  // Check the gradient sparsity pattern.
  // The gradient in x_autodiff2 is identity.
  const AutoDiffVecXd x_autodiff2 = math::InitializeAutoDiff(x_val);
  AutoDiffVecXd y_autodiff2;
  constraint.Eval(x_autodiff2, &y_autodiff2);
  // We take a zero matrix y_grad, set y_grad(i, j) = y_grad_expected(i, j) if
  // the pair (i, j) is in gradient_sparsity_pattern, and then compare y_grad
  // against y_grad_expected. This way we make sure that
  // gradient_sparsity_pattern doesn't miss any entry.
  const Eigen::MatrixXd y_grad_expected = math::ExtractGradient(y_autodiff2);
  Eigen::MatrixXd y_grad(y_grad_expected.rows(), y_grad_expected.cols());
  y_grad.setZero();
  const auto gradient_sparsity_pattern =
      constraint.GetConstraintSparsityPattern();
  for (const auto& row_col_pair : gradient_sparsity_pattern) {
    y_grad(row_col_pair.first, row_col_pair.second) =
        y_grad_expected(row_col_pair.first, row_col_pair.second);
  }
  EXPECT_TRUE(CompareMatrices(y_grad, y_grad_expected));
}
}  // namespace
}  // namespace internal

namespace {
GTEST_TEST(SlidingFrictionComplementarityConstraintTest, AddConstraint) {
  const CoulombFriction<double> sphere1_friction(1.1, 0.9);
  const test::SphereSpecification sphere1_spec(0.1, 1e3, sphere1_friction);
  const CoulombFriction<double> sphere2_friction(1.2, 0.8);
  const test::SphereSpecification sphere2_spec(0.2, 1.2e3, sphere2_friction);
  const CoulombFriction<double> ground_friction(0.6, 0.5);
  test::FreeSpheresAndBoxes<AutoDiffXd> spheres(
      {sphere1_spec, sphere2_spec}, {} /* no box. */, ground_friction);

  const ContactWrenchFromForceInWorldFrameEvaluator contact_wrench_evaluator(
      &(spheres.plant()), spheres.get_mutable_plant_context(),
      SortedPair<geometry::GeometryId>(spheres.sphere_geometry_ids()[0],
                                       spheres.sphere_geometry_ids()[1]));

  const double complementarity_tolerance = 1E-3;

  const auto& plant = spheres.plant();
  solvers::MathematicalProgram prog;
  auto q_vars = prog.NewContinuousVariables(plant.num_positions());
  auto v_vars = prog.NewContinuousVariables(plant.num_velocities());
  auto lambda_vars = prog.NewContinuousVariables<3>();
  auto bindings = AddSlidingFrictionComplementarityExplicitContactConstraint(
      &contact_wrench_evaluator, complementarity_tolerance, q_vars, v_vars,
      lambda_vars, &prog);

  EXPECT_EQ(prog.num_vars(), bindings.first.variables().rows());

  // Now check if the added constraint is satisfied when the sliding friction
  // force satisfies the complementarity constraint.

  // Set q_val and v_val to arbitrary values
  Eigen::Matrix<double, 14, 1> q_val;
  q_val.head<4>() << 0.5, -0.5, 0.5, -0.5;
  q_val.segment<3>(4) << 0.1, 0.2, 0.3;
  q_val.segment<4>(7) << 1.0 / 3, 2.0 / 3, 2.0 / 3, 0;
  q_val.tail<3>() << -0.2, 0.3, 0.1;
  Eigen::Matrix<double, 12, 1> v_val;
  v_val << 0.1, 0.2, 0.3, -0.4, 0.5, 0.6, -1.5, 0.3, 0.6, -0.2, 0.7, -1.2;
  Vector3<AutoDiffXd> nhat_SaSb_W_autodiff, v_SaCb_W_autodiff;
  internal::ComputeRelativeMotion<AutoDiffXd>(
      &spheres, q_val.cast<AutoDiffXd>(), v_val.cast<AutoDiffXd>(),
      sphere2_spec.radius, &nhat_SaSb_W_autodiff, &v_SaCb_W_autodiff);
  Eigen::Vector3d nhat_SaSb_W = math::ExtractValue(nhat_SaSb_W_autodiff);
  Eigen::Vector3d v_SaCb_W = math::ExtractValue(v_SaCb_W_autodiff);

  Eigen::Vector3d v_tangential_SaCb_W =
      (Eigen::Matrix3d::Identity() - nhat_SaSb_W * nhat_SaSb_W.transpose()) *
      v_SaCb_W;
  double c_val1 = 0.1;
  Eigen::Vector3d f_sliding_tangential = -c_val1 * v_tangential_SaCb_W;
  const CoulombFriction<double> combined_friction =
      CalcContactFrictionFromSurfaceProperties(sphere1_friction,
                                               sphere2_friction);
  Eigen::Vector3d f_sliding_normal = nhat_SaSb_W * f_sliding_tangential.norm() /
                                     combined_friction.dynamic_friction();
  Eigen::Vector3d f_sliding = f_sliding_normal + f_sliding_tangential;
  Eigen::Vector3d f_static(0, 0, 0);
  Eigen::Vector3d lambda = f_sliding + f_static;
  Eigen::VectorXd x_satisfied;
  bindings.first.evaluator()->ComposeX<double>(q_val, v_val, lambda, f_static,
                                               f_sliding, c_val1, &x_satisfied);
  EXPECT_TRUE(bindings.first.evaluator()->CheckSatisfied(x_satisfied, 1E-14));
}
}  // namespace
}  // namespace multibody
}  // namespace drake
