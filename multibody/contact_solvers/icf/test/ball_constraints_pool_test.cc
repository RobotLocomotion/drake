#include "drake/multibody/contact_solvers/icf/ball_constraints_pool.h"

#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

/* Checks that model.CalcData does not incur any heap allocations on a problem
with ball constraints. */
GTEST_TEST(BallConstraintsPool, LimitMallocOnCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddBallConstraints(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 2);
  EXPECT_EQ(model.num_ball_constraints(), 2);

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.ball_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10.0, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

/* Checks that pool.ReduceInto does not incur any heap allocations on a
problem with ball constraints. */
GTEST_TEST(BallConstraintsPool, LimitMallocOnReduceInto) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddBallConstraints(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // Do a not-smaller reduction to allocate memory in the reduced model.
  MakeModelReducible(&model, {});
  model.ReduceInto(&reduced_model, &mapping);

  // Given prior allocation of a big enough model, the constraint pool
  // reduction does not allocate.
  {
    drake::test::LimitMalloc guard;
    model.ball_constraints_pool().ReduceInto(
        mapping, &reduced_model.ball_constraints_pool());
  }
}

/* Verifies that ball constraints produce correct data. */
GTEST_TEST(BallConstraintsPool, Data) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.ball_constraints_data().num_constraints(), 0);

  // At this point there should be no ball constraints.
  EXPECT_EQ(model.num_ball_constraints(), 0);
  EXPECT_EQ(model.num_constraints(), 0);

  // Add ball constraints.
  AddBallConstraints(&model);
  EXPECT_EQ(model.num_ball_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 2);

  // Re-set sparsity since ball constraints introduce cross-clique coupling.
  model.SetSparsityPattern();

  // Resize data to include ball constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.ball_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const BallConstraintsDataPool<AutoDiffXd>& ball_data =
      data.ball_constraints_data();
  EXPECT_EQ(ball_data.num_constraints(), 2);

  // The ball constraints should add positive cost (non-zero constraint error).
  EXPECT_GT(ball_data.cost().value(), 0.0);

  // Impulses should be finite and non-zero.
  for (int k = 0; k < 2; ++k) {
    const Vector3<AutoDiffXd>& gamma = ball_data.gamma(k);
    EXPECT_TRUE(math::ExtractValue(gamma).allFinite());
    EXPECT_GT(math::ExtractValue(gamma).norm(), 0.0);
  }

  // The total cost should include the ball contribution.
  EXPECT_GT(data.cost().value(), data.momentum_cost().value());

  // Verify accumulated total cost and gradients via AutoDiff.
  const VectorXd total_cost_derivatives = data.cost().derivatives();
  const VectorXd total_gradient_value = math::ExtractValue(data.gradient());
  EXPECT_TRUE(CompareMatrices(total_gradient_value, total_cost_derivatives,
                              2 * kEpsilon, MatrixCompareType::relative));

  // Verify contributions to Hessian. The autodiff derivatives of the gradient
  // give the exact Hessian, so the difference from the analytically computed
  // Hessian is pure floating-point round-off, which scales with the magnitude
  // of the (large, ~O(10³)) intermediate products rather than with each
  // individual entry. We therefore use a scale-aware absolute tolerance; a
  // per-entry relative tolerance would be fragile on the ball's small
  // rotational-block entries (which lack the weld's large R⁻¹·I diagonal term).
  auto ball_hessian = model.MakeHessian(data);
  MatrixXd ball_hessian_value =
      math::ExtractValue(ball_hessian->MakeDenseMatrix());
  MatrixXd ball_gradient_derivatives = math::ExtractGradient(data.gradient());
  const double hessian_scale = ball_hessian_value.cwiseAbs().maxCoeff();
  EXPECT_TRUE(CompareMatrices(ball_hessian_value, ball_gradient_derivatives,
                              100 * kEpsilon * hessian_scale,
                              MatrixCompareType::absolute));

  // The cross-clique ball (body 2, clique 1 to body 3, clique 2) should
  // produce non-zero off-diagonal blocks in the Hessian.
  const double off_diag_norm = ball_hessian_value.block<6, 6>(6, 12).norm();
  EXPECT_GT(off_diag_norm, 0.0);

  // Check CalcCostAlongLine for ball constraints.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(
      nv, 0.1, -0.2);  // Arbitrary search direction.
  IcfSearchDirectionData<AutoDiffXd> search_data;

  // Set data with constant value of v.
  VectorX<AutoDiffXd> v_constant =
      VectorX<AutoDiffXd>::LinSpaced(nv, -10, 10.0);
  model.CalcData(v_constant, &data);
  model.CalcSearchDirectionData(data, w, &search_data);

  const AutoDiffXd alpha = {
      0.35 /* arbitrary value */,
      VectorXd::Ones(1) /* This is the independent variable */};
  AutoDiffXd dcost, d2cost;
  const AutoDiffXd cost =
      model.CalcCostAlongLine(alpha, data, search_data, &dcost, &d2cost);

  const double scale = std::abs(dcost.value());
  EXPECT_NEAR(dcost.value(), cost.derivatives()[0], scale * kEpsilon);
  EXPECT_NEAR(d2cost.value(), dcost.derivatives()[0], scale * kEpsilon);
}

/* Sets up a simple model with 3 bodies in separate cliques, suitable for
testing ball constraints. Body 0 is the world (anchored), bodies 1-3 are
dynamic with 6 DOFs each. */
template <typename T>
void MakeModelForBall(IcfModel<T>* model, double time_step = 0.01) {
  const int nv = 18;

  std::unique_ptr<IcfParameters<T>> params = model->ReleaseParameters();
  ASSERT_TRUE(params != nullptr);

  params->time_step = time_step;
  params->v0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  // Sparse mass matrix with three cliques of size 6.
  const Matrix6<T> A1 = 0.3 * Matrix6<T>::Identity();
  const Matrix6<T> A2 = 2.3 * Matrix6<T>::Identity();
  const Matrix6<T> A3 = 1.5 * Matrix6<T>::Identity();

  MatrixX<T>& M0 = params->M0;
  M0 = MatrixX<T>::Identity(nv, nv);
  M0.template block<6, 6>(0, 0) = A1;
  M0.template block<6, 6>(6, 6) = A2;
  M0.template block<6, 6>(12, 12) = A3;

  params->D0 = VectorX<T>::Constant(nv, 0.1);
  params->k0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  params->clique_sizes = {6, 6, 6};

  // Body 0 = world (anchored), body 1 = floating, body 2 = floating,
  // body 3 = non-floating (uses non-identity Jacobian).
  params->body_is_floating = {0, 1, 1, 0};
  params->body_mass = {1.0e20, 0.3, 2.3, 1.5};

  // Body-to-clique mapping. World is anchored (clique = -1).
  params->body_to_clique = {-1, 0, 1, 2};

  const Matrix6<T> J_WB3 = VectorX<T>::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);

  params->J_WB.Resize(4, 6, 6);
  params->J_WB[0] = Matrix6<T>::Identity();  // World (ignored).
  params->J_WB[1] = Matrix6<T>::Identity();  // Floating body.
  params->J_WB[2] = Matrix6<T>::Identity();  // Floating body.
  params->J_WB[3] = J_WB3;

  // No joint locking.
  auto& reduction = params->reduction;
  reduction.unlocked_dofs = {0,  1,  2,  3,  4,  5,   // BR
                             6,  7,  8,  9,  10, 11,  //
                             12, 13, 14, 15, 16, 17};
  reduction.per_clique_unlocked_dofs = {
      {0, 1, 2, 3, 4, 5},
      {0, 1, 2, 3, 4, 5},
      {0, 1, 2, 3, 4, 5},
  };

  model->ResetParameters(std::move(params));
}

/* Verifies basic construction and accessors. */
GTEST_TEST(BallConstraintsPool, BasicConstruction) {
  IcfModel<double> model;
  MakeModelForBall(&model);

  BallConstraintsPool<double>& balls = model.ball_constraints_pool();
  EXPECT_EQ(balls.num_constraints(), 0);

  balls.Resize(2);
  EXPECT_EQ(balls.num_constraints(), 2);
}

/* Verifies that the ball constraint correctly computes impulses, cost,
gradient, and Hessian for a simple case: a ball between a floating body
(body 1) and the world (body 0, anchored). Expected values are computed
by hand from the ball constraint formulation. */
GTEST_TEST(BallConstraintsPool, BallToWorld) {
  IcfModel<double> model;
  MakeModelForBall(&model);

  BallConstraintsPool<double>& balls = model.ball_constraints_pool();
  balls.Resize(1);

  // Small constraint error: P and Q are slightly offset.
  const Vector3<double> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<double> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<double> p_PQ_W(0.05, 0.0, 0.0);  // Small translational err.

  // Body 0 (anchored) = A, body 1 (floating, clique 0) = B.
  balls.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_PQ_W);

  // Set the sparsity pattern (required for Hessian).
  model.SetSparsityPattern();

  // Set up data.
  IcfData<double> data;
  model.ResizeData(&data);

  // Use v = v0 for simplicity.
  const VectorXd& v0 = model.v0();
  const VectorXd v = v0;
  model.CalcData(v, &data);

  // --- Hand calculation of expected ball constraint values ---
  //
  // Body 1 is floating in clique 0, so V_WB[1] = v0[0:6].
  const double dt = model.time_step();
  const double mass_B = model.body_mass(1);

  // Body B spatial velocity components (body 1, floating).
  const Vector3<double> w_WB = v0.head<3>();       // angular
  const Vector3<double> v_WBo = v0.segment<3>(3);  // linear at Bo

  // Midpoint M is halfway between P and Q.
  // Bm is on B, coincident with M: p_BoBm = p_BQ - 0.5*p_PQ.
  const Vector3<double> p_BoBm_W = p_BQ_W - 0.5 * p_PQ_W;
  // Linear velocity of Bm: v_WBm = v_WBo + w_WB × p_BoBm.
  const Vector3<double> v_WBm = v_WBo + w_WB.cross(p_BoBm_W);

  // Body A is anchored, so vc = v_WBm (relative to zero). The ball constraint
  // velocity is purely translational.
  const Vector3<double> vc = v_WBm;

  // Near-rigid regularization (from ball_constraints_pool.cc constants).
  constexpr double kBeta = IcfModel<double>::kBeta;
  const double taud = kBeta * dt / M_PI;
  const double dt_plus_taud = dt + taud;

  // Constraint function g0 = p_PQ (there is no rotational component).
  const Vector3<double> g0 = p_PQ_W;

  // Bias velocity: v̂ = -g0 / (dt + τ_d).
  const Vector3<double> v_hat = -g0 / dt_plus_taud;

  // Regularization R is a uniform diagonal: R = (r_scale/mass_B) * I₃,
  // (body A is anchored so only B contributes).
  const double r_scale =
      (kBeta * kBeta * dt * dt) / (4.0 * M_PI * M_PI * dt * dt_plus_taud);
  const double R_scalar = r_scale / mass_B;

  // Expected impulse: γ = R⁻¹(v̂ - vc), with R diagonal and uniform.
  const Vector3<double> expected_gamma = (v_hat - vc) / R_scalar;

  // Expected ball cost: ℓ = ½(v̂ - vc)ᵀ R⁻¹ (v̂ - vc) = ½(v̂ - vc)ᵀ γ.
  const double expected_ball_cost = 0.5 * (v_hat - vc).dot(expected_gamma);

  // Expected momentum cost for v = v0.
  // A = M + dt*D (block diagonal), r = A*v0 - dt*k0.
  // momentum_cost = v0ᵀ(½A v0 - r).
  MatrixXd A_mat = model.M0();
  A_mat.diagonal() += dt * model.D0();
  const VectorXd Av0 = A_mat * v0;
  const VectorXd r = Av0 - dt * model.k0();
  const double expected_momentum_cost = v0.dot(0.5 * Av0 - r);
  const double expected_total_cost =
      expected_momentum_cost + expected_ball_cost;

  // --- Verify computed values match hand calculation ---

  const Vector3<double>& gamma = data.ball_constraints_data().gamma(0);
  EXPECT_TRUE(CompareMatrices(gamma, expected_gamma, 4 * kEpsilon,
                              MatrixCompareType::relative));

  const double ball_cost = data.ball_constraints_data().cost();
  EXPECT_NEAR(ball_cost, expected_ball_cost,
              4 * kEpsilon * std::abs(expected_ball_cost));

  EXPECT_NEAR(data.momentum_cost(), expected_momentum_cost,
              4 * kEpsilon * std::abs(expected_momentum_cost));
  EXPECT_NEAR(data.cost(), expected_total_cost,
              4 * kEpsilon * std::abs(expected_total_cost));

  // --- Hand calculation of expected Hessian block for body B (clique 0) ---
  //
  // The full Hessian is H = A + ball_contribution, where A = M + dt*D is block
  // diagonal. For a ball-to-world constraint with floating body B, the ball
  // contribution to the (c_b, c_b) block is:
  //   H_BB = G_Bp = Φ(p_BoBm)ᵀ G Φ(p_BoBm)
  // where G = diag(0, R⁻¹ I₃) (only translational, i.e. the weld's Gr = 0) and
  // Φ(p) = [I₃, 0; -p×, I₃] is the spatial shift operator. Body B is floating
  // so J_WB = I₆, giving H_BB = G_Bp directly.

  const double R_inv = 1.0 / R_scalar;

  // p_BoBm_W was already computed above for the velocity calculation.

  // Skew-symmetric matrix [p_BoBm]×.
  const Eigen::Matrix3d px = math::VectorToSkewSymmetric(p_BoBm_W);
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  // G_Bp = Φᵀ G Φ with Gr = 0 and Gt = R⁻¹ I₃.
  // G_Bp = R⁻¹ [ -p×⋅p×  p× ]
  //            [   -p×   I₃ ]
  Matrix6<double> expected_H_BB;
  expected_H_BB.topLeftCorner<3, 3>() = -R_inv * (px * px);
  expected_H_BB.topRightCorner<3, 3>() = R_inv * px;
  expected_H_BB.bottomLeftCorner<3, 3>() = -R_inv * px;
  expected_H_BB.bottomRightCorner<3, 3>() = R_inv * I3;

  // Expected Hessian block: A_clique0 + H_BB.
  const Matrix6<double> A_clique0 = A_mat.block<6, 6>(0, 0);
  const Matrix6<double> expected_hessian_block = A_clique0 + expected_H_BB;

  // Build Hessian, verify it is finite, and check the result for bodyB.
  const MatrixXd hessian = model.MakeHessian(data)->MakeDenseMatrix();
  EXPECT_TRUE(hessian.allFinite());

  const Matrix6<double> hessian_block_B = hessian.block<6, 6>(0, 0);
  EXPECT_TRUE(CompareMatrices(hessian_block_B, expected_hessian_block,
                              4 * kEpsilon, MatrixCompareType::relative));
}

/* Verifies that the ball constraint applies zero impulse on axes with no
constraint error and no constraint velocity. We construct a case where the
constraint is perfectly satisfied (g0 = 0) and the body is at rest, so the
impulse must be exactly zero. */
GTEST_TEST(BallConstraintsPool, ZeroImpulseWhenSatisfied) {
  IcfModel<double> model;
  MakeModelForBall(&model);

  // Set v0 = 0 so the (only) floating body B is at rest.
  std::unique_ptr<IcfParameters<double>> params = model.ReleaseParameters();
  params->v0 = VectorXd::Zero(18);
  model.ResetParameters(std::move(params));

  BallConstraintsPool<double>& balls = model.ball_constraints_pool();
  balls.Resize(1);

  // No constraint error: P and Q are coincident (p_PQ = 0).
  const Vector3<double> p_AP_W(0.1, -0.2, 0.05);
  const Vector3<double> p_BQ_W(0.1, -0.2, 0.05);
  const Vector3<double> p_PQ_W(0.0, 0.0, 0.0);
  balls.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_PQ_W);

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  model.CalcData(model.v0(), &data);

  // With zero error and zero velocity, the impulse and cost must be zero.
  const Vector3<double>& gamma = data.ball_constraints_data().gamma(0);
  EXPECT_TRUE(CompareMatrices(gamma, Vector3<double>::Zero(), kEpsilon,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(data.ball_constraints_data().cost(), 0.0, kEpsilon);
}

/* Verifies the ball constraint between two dynamic bodies in different
cliques (cross-clique case). Parameterized to test both clique orderings:
  - false: bodyA=1 (clique 0), bodyB=2 (clique 1), i.e. c_b > c_a.
  - true:  bodyA=2 (clique 1), bodyB=1 (clique 0), i.e. c_a > c_b. */
class CrossCliqueBallTest : public testing::TestWithParam<bool> {};

TEST_P(CrossCliqueBallTest, CrossCliqueBall) {
  const bool reverse_bodies = GetParam();

  IcfModel<double> model;
  MakeModelForBall(&model);

  BallConstraintsPool<double>& balls = model.ball_constraints_pool();
  balls.Resize(1);

  // Ball between body 1 (clique 0) and body 2 (clique 1).
  const Vector3<double> p_AP_W(0.1, 0.2, 0.3);
  const Vector3<double> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<double> p_PQ_W(0.02, -0.01, 0.03);

  if (reverse_bodies) {
    // Body 2 = A (clique 1), Body 1 = B (clique 0), i.e. c_a > c_b.
    balls.Set(0, /*bodyA=*/2, /*bodyB=*/1, p_AP_W, p_BQ_W, p_PQ_W);
  } else {
    // Body 1 = A (clique 0), Body 2 = B (clique 1), i.e. c_b > c_a.
    balls.Set(0, /*bodyA=*/1, /*bodyB=*/2, p_AP_W, p_BQ_W, p_PQ_W);
  }

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  const VectorXd v = model.v0();
  model.CalcData(v, &data);

  // Verify cost and impulse.
  EXPECT_GT(data.ball_constraints_data().cost(), 0.0);
  const Vector3<double>& gamma = data.ball_constraints_data().gamma(0);
  EXPECT_TRUE(gamma.allFinite());
  EXPECT_GT(gamma.norm(), 0.0);

  // Build Hessian and verify it has off-diagonal blocks.
  auto hessian = model.MakeHessian(data);
  const MatrixXd H_dense = hessian->MakeDenseMatrix();
  EXPECT_TRUE(H_dense.allFinite());

  // The off-diagonal block between clique 0 and clique 1 should be non-zero.
  const double off_diag_norm = H_dense.block<6, 6>(0, 6).norm();
  EXPECT_GT(off_diag_norm, 0.0);
}

INSTANTIATE_TEST_SUITE_P(
    BallConstraintsPool, CrossCliqueBallTest, testing::Bool(),
    [](const testing::TestParamInfo<bool>& test_param_info) {
      return test_param_info.param ? "CliqueAGreater" : "CliqueBGreater";
    });

/* Verifies that CalcCostAlongLine produces consistent derivatives using
AutoDiff. For each sampled α value, we verify the cost, first derivative, and
second derivative by comparing CalcCostAlongLine outputs against the
full-precision AutoDiff result obtained by evaluating CalcData at v + α·w. */
GTEST_TEST(BallConstraintsPool, CalcCostAlongLine) {
  IcfModel<AutoDiffXd> model;
  MakeModelForBall(&model);

  BallConstraintsPool<AutoDiffXd>& balls = model.ball_constraints_pool();
  balls.Resize(1);

  const Vector3<AutoDiffXd> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_PQ_W(0.05, 0.0, 0.0);

  balls.Set(0, 0, 1, p_AP_W, p_BQ_W, p_PQ_W);

  model.SetSparsityPattern();

  const int nv = model.num_velocities();
  IcfData<AutoDiffXd> data, scratch;
  model.ResizeData(&data);
  model.ResizeData(&scratch);

  // Compute data at the base point v = v0.
  const VectorX<AutoDiffXd> v = VectorXd::LinSpaced(nv, -1.0, 1.0);
  model.CalcData(v, &data);

  // Arbitrary search direction.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(nv, 0.1, 0.5);

  // Precompute search direction data.
  IcfSearchDirectionData<AutoDiffXd> search_data;
  model.CalcSearchDirectionData(data, w, &search_data);

  // Verify at several α values.
  for (double alpha_value : {-0.45, 0.0, 0.15, 0.34, 0.93, 1.32}) {
    // Make α the independent variable for AutoDiff.
    const AutoDiffXd alpha = {alpha_value, VectorXd::Ones(1)};

    // Compute the expected cost and its derivative w.r.t. α via AutoDiff.
    const VectorX<AutoDiffXd> v_alpha = v + alpha * w;
    model.CalcData(v_alpha, &scratch);
    const double cost_expected = scratch.cost().value();
    const double dcost_expected = scratch.cost().derivatives()[0];
    const VectorXd w_times_H = math::ExtractGradient(scratch.gradient());
    const double d2cost_expected = w_times_H.dot(math::ExtractValue(w));

    // Compare against CalcCostAlongLine.
    AutoDiffXd dcost, d2cost;
    const AutoDiffXd cost =
        model.CalcCostAlongLine(alpha, data, search_data, &dcost, &d2cost);
    EXPECT_NEAR(cost.value(), cost_expected,
                8 * kEpsilon * std::abs(cost_expected));
    EXPECT_NEAR(dcost.value(), dcost_expected,
                8 * kEpsilon * std::abs(dcost_expected));
    EXPECT_NEAR(d2cost.value(), d2cost_expected,
                8 * kEpsilon * std::abs(d2cost_expected));
  }
}

/* Verifies gradient consistency using AutoDiff. */
GTEST_TEST(BallConstraintsPool, GradientConsistency) {
  IcfModel<AutoDiffXd> model;
  MakeModelForBall(&model);

  BallConstraintsPool<AutoDiffXd>& balls = model.ball_constraints_pool();
  balls.Resize(1);

  const Vector3<AutoDiffXd> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_PQ_W(0.05, 0.0, 0.0);

  balls.Set(0, 0, 1, p_AP_W, p_BQ_W, p_PQ_W);

  model.SetSparsityPattern();

  const int nv = model.num_velocities();
  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);

  const VectorXd v_values = math::ExtractValue(model.v0());
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);

  // The AutoDiff derivatives of the cost give the full-precision gradient.
  const VectorXd cost_derivatives = data.cost().derivatives();
  const VectorXd gradient_value = math::ExtractValue(data.gradient());

  EXPECT_TRUE(CompareMatrices(gradient_value, cost_derivatives, 100 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Verifies that reducing the ball constraint pool produces correct data. */
GTEST_TEST(BallConstraintsPool, Reduce) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddBallConstraints(&model);

  IcfData<double> data;
  model.ResizeData(&data);
  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model.CalcData(v, &data);

  auto check_reduced = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs [{}]", fmt::join(locked_dofs, ", ")));
    MakeModelReducible(&model, locked_dofs);
    IcfModel<double> reduced_model;
    ReducedMapping mapping;
    model.ReduceInto(&reduced_model, &mapping);

    // Check the data transmitted by pool.ReduceInto().
    const auto& full_pool = model.ball_constraints_pool();
    const auto& reduced_pool = reduced_model.ball_constraints_pool();

    int r_k{0};  // Reduced constraints cursor.
    for (int k = 0; k < full_pool.num_constraints(); ++k) {
      SCOPED_TRACE(
          fmt::format("full constraint {} vs. reduced constraint {}", k, r_k));
      const auto& [a, b] = full_pool.body_pairs()[k];
      const int clique_b = model.params().body_to_clique[b];
      const int clique_a = model.params().body_to_clique[a];
      const bool have_b = mapping.clique_subsequence.participates(clique_b);
      const bool have_a =
          clique_a >= 0 && mapping.clique_subsequence.participates(clique_a);
      if (!(have_a || have_b)) {
        continue;
      }
      const bool is_flipped = have_a && !have_b;
      const auto& [r_a, r_b] = reduced_pool.body_pairs()[r_k];
      SCOPED_TRACE(fmt::format("flipped? {} have a? {} have b? {}", is_flipped,
                               have_a, have_b));
      if (is_flipped) {
        EXPECT_EQ(r_a, b);
        EXPECT_EQ(r_b, a);
        EXPECT_EQ(reduced_pool.p_AP_W()[r_k], full_pool.p_BQ_W()[k]);
        EXPECT_EQ(reduced_pool.p_BQ_W()[r_k], full_pool.p_AP_W()[k]);
        EXPECT_EQ(reduced_pool.p_PQ_W()[r_k], -full_pool.p_PQ_W()[k]);
        EXPECT_EQ(reduced_pool.g0()[r_k], -full_pool.g0()[k]);
      } else {
        EXPECT_EQ(r_a, a);
        EXPECT_EQ(r_b, b);
        EXPECT_EQ(reduced_pool.p_AP_W()[r_k], full_pool.p_AP_W()[k]);
        EXPECT_EQ(reduced_pool.p_BQ_W()[r_k], full_pool.p_BQ_W()[k]);
        EXPECT_EQ(reduced_pool.p_PQ_W()[r_k], full_pool.p_PQ_W()[k]);
        EXPECT_EQ(reduced_pool.g0()[r_k], full_pool.g0()[k]);
      }
      ++r_k;
    }
    EXPECT_EQ(ssize(reduced_pool.R()), r_k);
    EXPECT_EQ(reduced_pool.hessian_blocks_size(), r_k);
    EXPECT_EQ(reduced_pool.num_constraints(), r_k);
  };

  // Reduce by none; essentially, just copy.
  const std::vector<int> none_locked;
  check_reduced(none_locked);

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  check_reduced(arbitrary_locked);

  // Lock clique 0.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  check_reduced(clique0_locked);

  // Lock clique 1.
  const std::vector<int> clique1_locked = {6, 7, 8, 9, 10, 11};
  check_reduced(clique1_locked);

  // Lock clique 2.
  const std::vector<int> clique2_locked = {12, 13, 14, 15, 16, 17};
  check_reduced(clique2_locked);

  // Lock everything.
  std::vector<int> all_locked(model.num_velocities());
  std::iota(all_locked.begin(), all_locked.end(), 0);
  check_reduced(all_locked);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
