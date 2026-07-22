#include "drake/multibody/contact_solvers/icf/distance_constraints_pool.h"

#include <cmath>
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
with distance constraints. */
GTEST_TEST(DistanceConstraintsPool, LimitMallocOnCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddDistanceConstraints(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 2);
  EXPECT_EQ(model.num_distance_constraints(), 2);

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.distance_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10.0, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

/* Checks that pool.ReduceInto does not incur any heap allocations on a
problem with distance constraints. */
GTEST_TEST(DistanceConstraintsPool, LimitMallocOnReduceInto) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddDistanceConstraints(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // Do a not-smaller reduction to allocate memory in the reduced model.
  MakeModelReducible(&model, {});
  model.ReduceInto(&reduced_model, &mapping);

  // Given prior allocation of a big enough model, the constraint pool
  // reduction does not allocate.
  {
    drake::test::LimitMalloc guard;
    model.distance_constraints_pool().ReduceInto(
        mapping, &reduced_model.distance_constraints_pool());
  }
}

/* Verifies that distance constraints produce correct data. Uses a rigid and a
compliant constraint (see AddDistanceConstraints), exercising both R branches.
*/
GTEST_TEST(DistanceConstraintsPool, Data) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.distance_constraints_data().num_constraints(), 0);

  // At this point there should be no distance constraints.
  EXPECT_EQ(model.num_distance_constraints(), 0);
  EXPECT_EQ(model.num_constraints(), 0);

  // Add distance constraints.
  AddDistanceConstraints(&model);
  EXPECT_EQ(model.num_distance_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 2);

  // Re-set sparsity since distance constraints introduce cross-clique coupling.
  model.SetSparsityPattern();

  // Resize data to include distance constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.distance_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const DistanceConstraintsDataPool<AutoDiffXd>& distance_data =
      data.distance_constraints_data();
  EXPECT_EQ(distance_data.num_constraints(), 2);

  // The distance constraints should add positive cost (non-zero error).
  EXPECT_GT(distance_data.cost().value(), 0.0);

  // Impulses should be finite and non-zero scalars.
  for (int k = 0; k < 2; ++k) {
    const AutoDiffXd& gamma = distance_data.gamma(k)(0);
    EXPECT_TRUE(std::isfinite(gamma.value()));
    EXPECT_GT(std::abs(gamma.value()), 0.0);
  }

  // The total cost should include the distance contribution.
  EXPECT_GT(data.cost().value(), data.momentum_cost().value());

  // Verify accumulated total cost and gradients via AutoDiff.
  const VectorXd total_cost_derivatives = data.cost().derivatives();
  const VectorXd total_gradient_value = math::ExtractValue(data.gradient());
  EXPECT_TRUE(CompareMatrices(total_gradient_value, total_cost_derivatives,
                              2 * kEpsilon, MatrixCompareType::relative));

  // Verify contributions to Hessian. The autodiff derivatives of the gradient
  // give the exact Hessian, so the difference from the analytically computed
  // Hessian is pure floating-point round-off, which scales with the magnitude
  // of the (large) intermediate products rather than with each individual
  // entry. We therefore use a scale-aware absolute tolerance.
  auto distance_hessian = model.MakeHessian(data);
  MatrixXd distance_hessian_value =
      math::ExtractValue(distance_hessian->MakeDenseMatrix());
  MatrixXd distance_gradient_derivatives =
      math::ExtractGradient(data.gradient());
  const double hessian_scale = distance_hessian_value.cwiseAbs().maxCoeff();
  EXPECT_TRUE(CompareMatrices(
      distance_hessian_value, distance_gradient_derivatives,
      100 * kEpsilon * hessian_scale, MatrixCompareType::absolute));

  // The cross-clique distance (body 2, clique 1 to body 3, clique 2) should
  // produce non-zero off-diagonal blocks in the Hessian.
  const double off_diag_norm = distance_hessian_value.block<6, 6>(6, 12).norm();
  EXPECT_GT(off_diag_norm, 0.0);

  // Check CalcCostAlongLine for distance constraints.
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
testing distance constraints. Body 0 is the world (anchored), bodies 1-3 are
dynamic with 6 DOFs each. */
template <typename T>
void MakeModelForDistance(IcfModel<T>* model, double time_step = 0.01) {
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
GTEST_TEST(DistanceConstraintsPool, BasicConstruction) {
  IcfModel<double> model;
  MakeModelForDistance(&model);

  DistanceConstraintsPool<double>& distances =
      model.distance_constraints_pool();
  EXPECT_EQ(distances.num_constraints(), 0);

  distances.Resize(2);
  EXPECT_EQ(distances.num_constraints(), 2);
}

/* Verifies that a RIGID distance constraint correctly computes impulse, cost,
gradient, and Hessian for a simple case: a distance constraint between a
floating body (body 1) and the world (body 0, anchored). Expected values are
computed by hand from the distance constraint formulation. */
GTEST_TEST(DistanceConstraintsPool, DistanceToWorld) {
  IcfModel<double> model;
  MakeModelForDistance(&model);

  DistanceConstraintsPool<double>& distances =
      model.distance_constraints_pool();
  distances.Resize(1);

  const Vector3<double> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<double> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<double> p_hat_W(1.0, 0.0, 0.0);  // Unit direction.
  const double g0 = 0.05;                        // Distance error d₀ − ℓ.
  const double kInf = std::numeric_limits<double>::infinity();

  // Body 0 (anchored) = A, body 1 (floating, clique 0) = B. Rigid (k = ∞).
  distances.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_hat_W, g0,
                /*stiffness=*/kInf, /*damping=*/0.0);

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);

  const VectorXd& v0 = model.v0();
  model.CalcData(v0, &data);

  // --- Hand calculation of expected distance constraint values ---
  const double dt = model.time_step();
  const double mass_B = model.body_mass(1);

  // Constraint velocity vc = p̂ᵀ⋅v_W_Bq, with v_W_Bq the velocity of point Q on
  // the floating body 1. Body A (world) is anchored so it contributes nothing.
  const Vector3<double> w_WB = v0.head<3>();
  const Vector3<double> v_WBo = v0.segment<3>(3);
  const Vector3<double> v_W_Bq = v_WBo + w_WB.cross(p_BQ_W);
  const double vc = p_hat_W.dot(v_W_Bq);

  // Near-rigid regularization (rigid ⇒ near-rigid branch chosen).
  constexpr double kBeta = IcfModel<double>::kBeta;
  const double taud = kBeta * dt / M_PI;
  const double dt_plus_taud = dt + taud;
  const double r_scale =
      (kBeta * kBeta * dt * dt) / (4.0 * M_PI * M_PI * dt * dt_plus_taud);
  const double w = 1.0 / mass_B;  // Body A anchored.
  const double R = r_scale * w;
  const double v_hat = -g0 / dt_plus_taud;

  const double expected_gamma = (v_hat - vc) / R;
  const double expected_cost = 0.5 * (v_hat - vc) * expected_gamma;

  // Expected momentum cost for v = v0.
  MatrixXd A_mat = model.M0();
  A_mat.diagonal() += dt * model.D0();
  const VectorXd Av0 = A_mat * v0;
  const VectorXd r = Av0 - dt * model.k0();
  const double expected_momentum_cost = v0.dot(0.5 * Av0 - r);
  const double expected_total_cost = expected_momentum_cost + expected_cost;

  // --- Verify computed values match hand calculation ---
  // The rigid constraint must select the near-rigid regularization.
  EXPECT_NEAR(distances.R()[0], R, 4 * kEpsilon * R);
  EXPECT_NEAR(distances.v_hat()[0](0), v_hat, 4 * kEpsilon * std::abs(v_hat));

  const double gamma = data.distance_constraints_data().gamma(0)(0);
  EXPECT_NEAR(gamma, expected_gamma, 4 * kEpsilon * std::abs(expected_gamma));

  const double distance_cost = data.distance_constraints_data().cost();
  EXPECT_NEAR(distance_cost, expected_cost,
              4 * kEpsilon * std::abs(expected_cost));

  EXPECT_NEAR(data.momentum_cost(), expected_momentum_cost,
              4 * kEpsilon * std::abs(expected_momentum_cost));
  EXPECT_NEAR(data.cost(), expected_total_cost,
              4 * kEpsilon * std::abs(expected_total_cost));

  // --- Hand calculation of the (rank-1) Hessian block for body B (clique 0)
  // ---
  //
  // For a distance-to-world constraint with floating body B, the contribution
  // to the (c_B, c_B) block is H_BB = Φ(p_BQ)ᵀ⋅diag(0, Gt)⋅Φ(p_BQ), where the
  // rank-1 translational regularization Gt = R⁻¹⋅p̂⋅p̂ᵀ. Body B is floating so
  // J_WB = I₆, giving H_BB directly.
  const double R_inv = 1.0 / R;
  const Eigen::Matrix3d Gt = R_inv * p_hat_W * p_hat_W.transpose();
  const Eigen::Matrix3d px = math::VectorToSkewSymmetric(p_BQ_W);

  Matrix6<double> expected_H_BB;
  expected_H_BB.topLeftCorner<3, 3>() = -px * Gt * px;
  expected_H_BB.topRightCorner<3, 3>() = px * Gt;
  expected_H_BB.bottomLeftCorner<3, 3>() = -Gt * px;
  expected_H_BB.bottomRightCorner<3, 3>() = Gt;

  const Matrix6<double> A_clique0 = A_mat.block<6, 6>(0, 0);
  const Matrix6<double> expected_hessian_block = A_clique0 + expected_H_BB;

  const MatrixXd hessian = model.MakeHessian(data)->MakeDenseMatrix();
  EXPECT_TRUE(hessian.allFinite());

  const Matrix6<double> hessian_block_B = hessian.block<6, 6>(0, 0);
  EXPECT_TRUE(CompareMatrices(hessian_block_B, expected_hessian_block,
                              4 * kEpsilon, MatrixCompareType::relative));
}

/* Verifies the compliant (finite stiffness/damping) branch: the regularization
must be R = 1/(δt⋅(δt + τ)⋅k) with τ = c/k, and the impulse/cost must match the
hand-computed spring-damper values. */
GTEST_TEST(DistanceConstraintsPool, CompliantSpring) {
  IcfModel<double> model;
  MakeModelForDistance(&model);

  DistanceConstraintsPool<double>& distances =
      model.distance_constraints_pool();
  distances.Resize(1);

  const Vector3<double> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<double> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<double> p_hat_W(1.0, 0.0, 0.0);
  const double g0 = 0.05;
  // Soft spring so the compliant branch (not the near-rigid floor) is selected.
  const double stiffness = 100.0;
  const double damping = 1.0;

  distances.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_hat_W, g0,
                stiffness, damping);

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  const VectorXd& v0 = model.v0();
  model.CalcData(v0, &data);

  // --- Hand calculation ---
  const double dt = model.time_step();
  const double tau = damping / stiffness;
  const double R_compliant = 1.0 / (dt * (dt + tau) * stiffness);

  // Confirm the compliant branch was selected (softer than the near-rigid
  // floor).
  constexpr double kBeta = IcfModel<double>::kBeta;
  const double taud = kBeta * dt / M_PI;
  const double r_scale =
      (kBeta * kBeta * dt * dt) / (4.0 * M_PI * M_PI * dt * (dt + taud));
  const double R_near_rigid = r_scale * (1.0 / model.body_mass(1));
  ASSERT_GT(R_compliant, R_near_rigid);

  EXPECT_NEAR(distances.R()[0], R_compliant, 4 * kEpsilon * R_compliant);

  const double v_hat = -g0 / (dt + tau);
  EXPECT_NEAR(distances.v_hat()[0](0), v_hat, 4 * kEpsilon * std::abs(v_hat));

  // Constraint velocity and expected impulse/cost.
  const Vector3<double> w_WB = v0.head<3>();
  const Vector3<double> v_WBo = v0.segment<3>(3);
  const Vector3<double> v_W_Bq = v_WBo + w_WB.cross(p_BQ_W);
  const double vc = p_hat_W.dot(v_W_Bq);
  const double expected_gamma = (v_hat - vc) / R_compliant;
  const double expected_cost = 0.5 * (v_hat - vc) * expected_gamma;

  const double gamma = data.distance_constraints_data().gamma(0)(0);
  EXPECT_NEAR(gamma, expected_gamma, 4 * kEpsilon * std::abs(expected_gamma));
  EXPECT_NEAR(data.distance_constraints_data().cost(), expected_cost,
              4 * kEpsilon * std::abs(expected_cost));
}

/* Verifies the distance constraint between two dynamic bodies in different
cliques (cross-clique case). Parameterized to test both clique orderings. */
class CrossCliqueDistanceTest : public testing::TestWithParam<bool> {};

TEST_P(CrossCliqueDistanceTest, CrossCliqueDistance) {
  const bool reverse_bodies = GetParam();

  IcfModel<double> model;
  MakeModelForDistance(&model);

  DistanceConstraintsPool<double>& distances =
      model.distance_constraints_pool();
  distances.Resize(1);

  // Distance between body 1 (clique 0) and body 2 (clique 1).
  const Vector3<double> p_AP_W(0.1, 0.2, 0.3);
  const Vector3<double> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<double> p_hat_W = Vector3<double>(0.2, -0.1, 0.3).normalized();
  const double g0 = 0.02;

  if (reverse_bodies) {
    // Body 2 = A (clique 1), Body 1 = B (clique 0), i.e. c_A > c_B.
    distances.Set(0, /*bodyA=*/2, /*bodyB=*/1, p_AP_W, p_BQ_W, p_hat_W, g0,
                  /*stiffness=*/1000.0, /*damping=*/5.0);
  } else {
    // Body 1 = A (clique 0), Body 2 = B (clique 1), i.e. c_B > c_A.
    distances.Set(0, /*bodyA=*/1, /*bodyB=*/2, p_AP_W, p_BQ_W, p_hat_W, g0,
                  /*stiffness=*/1000.0, /*damping=*/5.0);
  }

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  const VectorXd v = model.v0();
  model.CalcData(v, &data);

  // Verify cost and impulse.
  EXPECT_GT(data.distance_constraints_data().cost(), 0.0);
  const double gamma = data.distance_constraints_data().gamma(0)(0);
  EXPECT_TRUE(std::isfinite(gamma));
  EXPECT_GT(std::abs(gamma), 0.0);

  // Build Hessian and verify it has off-diagonal blocks.
  auto hessian = model.MakeHessian(data);
  const MatrixXd H_dense = hessian->MakeDenseMatrix();
  EXPECT_TRUE(H_dense.allFinite());

  // The off-diagonal block between clique 0 and clique 1 should be non-zero.
  const double off_diag_norm = H_dense.block<6, 6>(0, 6).norm();
  EXPECT_GT(off_diag_norm, 0.0);
}

INSTANTIATE_TEST_SUITE_P(
    DistanceConstraintsPool, CrossCliqueDistanceTest, testing::Bool(),
    [](const testing::TestParamInfo<bool>& test_param_info) {
      return test_param_info.param ? "CliqueAGreater" : "CliqueBGreater";
    });

/* Verifies that CalcCostAlongLine produces consistent derivatives using
AutoDiff. */
GTEST_TEST(DistanceConstraintsPool, CalcCostAlongLine) {
  IcfModel<AutoDiffXd> model;
  MakeModelForDistance(&model);

  DistanceConstraintsPool<AutoDiffXd>& distances =
      model.distance_constraints_pool();
  distances.Resize(1);

  const Vector3<AutoDiffXd> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<AutoDiffXd> p_hat_W(1.0, 0.0, 0.0);

  // Use a compliant spring so this test also covers the compliant branch.
  distances.Set(0, 0, 1, p_AP_W, p_BQ_W, p_hat_W, AutoDiffXd(0.05),
                AutoDiffXd(100.0), AutoDiffXd(1.0));

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
    const AutoDiffXd alpha = {alpha_value, VectorXd::Ones(1)};

    const VectorX<AutoDiffXd> v_alpha = v + alpha * w;
    model.CalcData(v_alpha, &scratch);
    const double cost_expected = scratch.cost().value();
    const double dcost_expected = scratch.cost().derivatives()[0];
    const VectorXd w_times_H = math::ExtractGradient(scratch.gradient());
    const double d2cost_expected = w_times_H.dot(math::ExtractValue(w));

    AutoDiffXd dcost, d2cost;
    const AutoDiffXd cost =
        model.CalcCostAlongLine(alpha, data, search_data, &dcost, &d2cost);
    // Scale-aware tolerance with an absolute floor: the analytic and autodiff
    // paths agree to a few ULP, but a scalar distance constraint's derivatives
    // can be small, so a pure relative tolerance would be too tight.
    EXPECT_NEAR(cost.value(), cost_expected,
                100 * kEpsilon * (std::abs(cost_expected) + 1.0));
    EXPECT_NEAR(dcost.value(), dcost_expected,
                100 * kEpsilon * (std::abs(dcost_expected) + 1.0));
    EXPECT_NEAR(d2cost.value(), d2cost_expected,
                100 * kEpsilon * (std::abs(d2cost_expected) + 1.0));
  }
}

/* Verifies gradient consistency using AutoDiff. */
GTEST_TEST(DistanceConstraintsPool, GradientConsistency) {
  IcfModel<AutoDiffXd> model;
  MakeModelForDistance(&model);

  DistanceConstraintsPool<AutoDiffXd>& distances =
      model.distance_constraints_pool();
  distances.Resize(1);

  const Vector3<AutoDiffXd> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<AutoDiffXd> p_hat_W(1.0, 0.0, 0.0);

  distances.Set(0, 0, 1, p_AP_W, p_BQ_W, p_hat_W, AutoDiffXd(0.05),
                AutoDiffXd(100.0), AutoDiffXd(1.0));

  model.SetSparsityPattern();

  const int nv = model.num_velocities();
  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);

  const VectorXd v_values = math::ExtractValue(model.v0());
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);

  const VectorXd cost_derivatives = data.cost().derivatives();
  const VectorXd gradient_value = math::ExtractValue(data.gradient());

  EXPECT_TRUE(CompareMatrices(gradient_value, cost_derivatives, 100 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Verifies that reducing the distance constraint pool produces correct data. */
GTEST_TEST(DistanceConstraintsPool, Reduce) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddDistanceConstraints(&model);

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
    const auto& full_pool = model.distance_constraints_pool();
    const auto& reduced_pool = reduced_model.distance_constraints_pool();

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
        // The unit direction p̂ (from P to Q) negates when P and Q swap; the
        // scalar constraint function g₀ = d − ℓ is symmetric and unchanged.
        EXPECT_EQ(reduced_pool.p_hat_W()[r_k], -full_pool.p_hat_W()[k]);
        EXPECT_EQ(reduced_pool.g0()[r_k], full_pool.g0()[k]);
      } else {
        EXPECT_EQ(r_a, a);
        EXPECT_EQ(r_b, b);
        EXPECT_EQ(reduced_pool.p_AP_W()[r_k], full_pool.p_AP_W()[k]);
        EXPECT_EQ(reduced_pool.p_BQ_W()[r_k], full_pool.p_BQ_W()[k]);
        EXPECT_EQ(reduced_pool.p_hat_W()[r_k], full_pool.p_hat_W()[k]);
        EXPECT_EQ(reduced_pool.g0()[r_k], full_pool.g0()[k]);
      }
      // Compliance parameters carry over unchanged regardless of flipping.
      EXPECT_EQ(reduced_pool.stiffness()[r_k], full_pool.stiffness()[k]);
      EXPECT_EQ(reduced_pool.damping()[r_k], full_pool.damping()[k]);
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
