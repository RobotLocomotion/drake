#include "drake/multibody/contact_solvers/icf/weld_constraints_pool.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

/* Sets up a simple model with 3 bodies in separate cliques, suitable for
testing weld constraints. Body 0 is the world (anchored), bodies 1-3 are
dynamic with 6 DOFs each. */
template <typename T>
void MakeModelForWeld(IcfModel<T>* model, double time_step = 0.01) {
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
  params->clique_start = {0, 6, 12, nv};

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

  model->ResetParameters(std::move(params));
}

/* Verifies basic construction and accessors. */
GTEST_TEST(WeldConstraintsPool, BasicConstruction) {
  IcfModel<double> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<double>& welds = model.weld_constraints_pool();
  EXPECT_EQ(welds.num_constraints(), 0);

  welds.Resize(2);
  EXPECT_EQ(welds.num_constraints(), 2);
}

/* Verifies that the weld constraint correctly computes impulses, cost,
gradient, and Hessian for a simple case: a weld between a floating body
(body 1) and the world (body 0, anchored). Expected values are computed
by hand from the weld constraint formulation. */
GTEST_TEST(WeldConstraintsPool, WeldToWorld) {
  IcfModel<double> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<double>& welds = model.weld_constraints_pool();
  welds.Resize(1);

  // Small constraint error: P and Q are slightly offset.
  const Vector3<double> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<double> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<double> p_PoQo_W(0.05, 0.0, 0.0);  // Small translational err.
  const Vector3<double> a_PQ_W(0.0, 0.0, 0.01);    // Small rotational error.

  // Body 0 (anchored) = A, body 1 (floating, clique 0) = B.
  welds.Set(0, /*bodyA=*/0, /*bodyB=*/1, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);

  // Set the sparsity pattern (required for Hessian).
  model.SetSparsityPattern();

  // Set up data.
  IcfData<double> data;
  model.ResizeData(&data);

  // Use v = v0 for simplicity.
  const VectorXd& v0 = model.v0();
  const VectorXd v = v0;
  model.CalcData(v, &data);

  // --- Hand calculation of expected weld constraint values ---
  //
  // Body 1 is floating in clique 0, so V_WB[1] = v0[0:6].
  const double dt = model.time_step();
  const double mass_B = model.body_mass(1);

  // Body B spatial velocity components (body 1, floating).
  const Vector3<double> w_WB = v0.head<3>();       // angular
  const Vector3<double> v_WBo = v0.segment<3>(3);  // linear at Bo

  // Midpoint M is halfway between P and Q.
  // Bm is on B, coincident with M: p_BoBm = p_BQ - 0.5*p_PoQo.
  const Vector3<double> p_BoBm_W = p_BQ_W - 0.5 * p_PoQo_W;
  // Linear velocity of Bm: v_WBm = v_WBo + w_WB × p_BoBm.
  const Vector3<double> v_WBm = v_WBo + w_WB.cross(p_BoBm_W);

  // Body A is anchored, so vc = V_WBm (relative to zero).
  Vector6<double> vc;
  vc.head<3>() = w_WB;
  vc.tail<3>() = v_WBm;

  // Near-rigid regularization (from weld_constraints_pool.cc constants).
  constexpr double kBeta = 0.1;
  const double taud = kBeta * dt / M_PI;
  const double dt_plus_taud = dt + taud;

  // Constraint function g0 = (a_PQ, p_PoQo).
  Vector6<double> g0;
  g0 << a_PQ_W, p_PoQo_W;

  // Bias velocity: v̂ = -g0 / (dt + τ_d).
  const Vector6<double> v_hat = -g0 / dt_plus_taud;

  // Regularization R is a uniform diagonal: R = (r_scale/mass_B) * I₆,
  // (body A is anchored so only B contributes).
  const double r_scale =
      (kBeta * kBeta * dt * dt) / (4.0 * M_PI * M_PI * dt * dt_plus_taud);
  const double R_scalar = r_scale / mass_B;

  // Expected impulse: γ = R⁻¹(v̂ - vc), with R diagonal and uniform.
  const Vector6<double> expected_gamma = (v_hat - vc) / R_scalar;

  // Expected weld cost: ℓ = ½(v̂ - vc)ᵀ R⁻¹ (v̂ - vc) = ½(v̂ - vc)ᵀ γ.
  const double expected_weld_cost = 0.5 * (v_hat - vc).dot(expected_gamma);

  // Expected momentum cost for v = v0.
  // A = M + dt*D (block diagonal), r = A*v0 - dt*k0.
  // momentum_cost = v0ᵀ(½A v0 - r).
  MatrixXd A_mat = model.M0();
  A_mat.diagonal() += dt * model.D0();
  const VectorXd Av0 = A_mat * v0;
  const VectorXd r = Av0 - dt * model.k0();
  const double expected_momentum_cost = v0.dot(0.5 * Av0 - r);
  const double expected_total_cost =
      expected_momentum_cost + expected_weld_cost;

  // --- Verify computed values match hand calculation ---

  const Vector6<double>& gamma = data.weld_constraints_data().gamma(0);
  EXPECT_TRUE(CompareMatrices(gamma, expected_gamma, 4 * kEpsilon,
                              MatrixCompareType::relative));

  const double weld_cost = data.weld_constraints_data().cost();
  EXPECT_NEAR(weld_cost, expected_weld_cost,
              4 * kEpsilon * std::abs(expected_weld_cost));

  EXPECT_NEAR(data.momentum_cost(), expected_momentum_cost,
              4 * kEpsilon * std::abs(expected_momentum_cost));
  EXPECT_NEAR(data.cost(), expected_total_cost,
              4 * kEpsilon * std::abs(expected_total_cost));

  // --- Hand calculation of expected Hessian block for body B (clique 0) ---
  //
  // The full Hessian is H = A + weld_contribution, where A = M + dt*D is block
  // diagonal. For a weld-to-world constraint with floating body B, the weld
  // contribution to the (c_b, c_b) block is:
  //   H_BB = G_Bp = Φ(p_BoBm)ᵀ G Φ(p_BoBm)
  // where G = diag(R⁻¹) and Φ(p) = [I₃, 0; -p×, I₃] is the spatial shift
  // operator. Body B is floating so J_WB = I₆, giving H_BB = G_Bp directly.

  const double R_inv = 1.0 / R_scalar;

  // p_BoBm_W was already computed above for the velocity calculation.

  // Skew-symmetric matrix [p_BoBm]×.
  const Eigen::Matrix3d px = math::VectorToSkewSymmetric(p_BoBm_W);
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  // G_Bp = Φᵀ G Φ with G = R⁻¹ I₆ (uniform scalar).
  // G_Bp = R⁻¹ [ I₃ - p×⋅p×  p× ]
  //            [    -p×      I₃ ]
  Matrix6<double> expected_H_BB;
  expected_H_BB.topLeftCorner<3, 3>() = R_inv * (I3 - px * px);
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

/* Verifies the weld constraint between two dynamic bodies in different
cliques (cross-clique case). */
GTEST_TEST(WeldConstraintsPool, CrossCliqueWeld) {
  IcfModel<double> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<double>& welds = model.weld_constraints_pool();
  welds.Resize(1);

  // Weld between body 1 (clique 0) and body 2 (clique 1).
  const Vector3<double> p_AP_W(0.1, 0.2, 0.3);
  const Vector3<double> p_BQ_W(0.0, 0.1, 0.0);
  const Vector3<double> p_PoQo_W(0.02, -0.01, 0.03);
  const Vector3<double> a_PQ_W(0.005, -0.003, 0.001);

  // Body 1 = A (clique 0), Body 2 = B (clique 1).
  welds.Set(0, /*bodyA=*/1, /*bodyB=*/2, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  const VectorXd v = model.v0();
  model.CalcData(v, &data);

  // Verify cost and impulse.
  EXPECT_GT(data.weld_constraints_data().cost(), 0.0);
  const Vector6<double>& gamma = data.weld_constraints_data().gamma(0);
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

/* Verifies that CalcCostAlongLine produces consistent derivatives using
AutoDiff. For each sampled α value, we verify the cost, first derivative, and
second derivative by comparing CalcCostAlongLine outputs against the
full-precision AutoDiff result obtained by evaluating CalcData at v + α·w. */
GTEST_TEST(WeldConstraintsPool, CalcCostAlongLine) {
  IcfModel<AutoDiffXd> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<AutoDiffXd>& welds = model.weld_constraints_pool();
  welds.Resize(1);

  const Vector3<AutoDiffXd> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_PoQo_W(0.05, 0.0, 0.0);
  const Vector3<AutoDiffXd> a_PQ_W(0.0, 0.0, 0.01);

  welds.Set(0, 0, 1, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);

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
GTEST_TEST(WeldConstraintsPool, GradientConsistency) {
  IcfModel<AutoDiffXd> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<AutoDiffXd>& welds = model.weld_constraints_pool();
  welds.Resize(1);

  const Vector3<AutoDiffXd> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<AutoDiffXd> p_PoQo_W(0.05, 0.0, 0.0);
  const Vector3<AutoDiffXd> a_PQ_W(0.0, 0.0, 0.01);

  welds.Set(0, 0, 1, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);

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

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
