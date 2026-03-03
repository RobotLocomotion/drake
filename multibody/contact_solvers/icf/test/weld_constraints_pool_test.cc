#include "drake/multibody/contact_solvers/icf/weld_constraints_pool.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
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

/* Sets up a simple model with 3 bodies in separate cliques, suitable for
testing weld constraints. Body 0 is the world (anchored), bodies 1-3 are
dynamic with 6 DOFs each. */
void MakeModelForWeld(IcfModel<double>* model, double time_step = 0.01) {
  const int nv = 18;

  std::unique_ptr<IcfParameters<double>> params = model->ReleaseParameters();
  ASSERT_TRUE(params != nullptr);

  params->time_step = time_step;
  params->v0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  // Sparse mass matrix with three cliques of size 6.
  const Matrix6<double> A1 = 0.3 * Matrix6<double>::Identity();
  const Matrix6<double> A2 = 2.3 * Matrix6<double>::Identity();
  const Matrix6<double> A3 = 1.5 * Matrix6<double>::Identity();

  MatrixXd& M0 = params->M0;
  M0 = MatrixXd::Identity(nv, nv);
  M0.block<6, 6>(0, 0) = A1;
  M0.block<6, 6>(6, 6) = A2;
  M0.block<6, 6>(12, 12) = A3;

  params->D0 = VectorXd::Constant(nv, 0.1);
  params->k0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  params->clique_sizes = {6, 6, 6};
  params->clique_start = {0, 6, 12, nv};

  // Body 0 = world (anchored), body 1 = floating, body 2 = floating,
  // body 3 = non-floating (uses non-identity Jacobian).
  params->body_is_floating = {0, 1, 1, 0};
  params->body_mass = {1.0e20, 0.3, 2.3, 1.5};

  // Body-to-clique mapping. World is anchored (clique = -1).
  params->body_to_clique = {-1, 0, 1, 2};

  const Matrix6<double> J_WB3 =
      VectorXd::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);

  params->J_WB.Resize(4, 6, 6);
  params->J_WB[0] = Matrix6<double>::Identity();  // World (ignored).
  params->J_WB[1] = Matrix6<double>::Identity();  // Floating body.
  params->J_WB[2] = Matrix6<double>::Identity();  // Floating body.
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
(body 1) and the world (body 0, anchored). */
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
  const VectorXd v = model.v0();
  model.CalcData(v, &data);

  // The weld constraint should add positive cost.
  const double weld_cost = data.weld_constraints_data().cost();
  EXPECT_GT(weld_cost, 0.0);

  // Impulse should be finite and non-zero (constraint error is non-zero).
  const Vector6<double>& gamma = data.weld_constraints_data().gamma(0);
  EXPECT_TRUE(gamma.allFinite());
  EXPECT_GT(gamma.norm(), 0.0);

  // The total cost should include the weld contribution.
  EXPECT_GT(data.cost(), data.momentum_cost());

  // Build Hessian and verify it is finite.
  auto hessian = model.MakeHessian(data);
  EXPECT_TRUE(hessian->MakeDenseMatrix().allFinite());
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

/* Verifies that CalcCostAlongLine produces consistent derivatives. */
GTEST_TEST(WeldConstraintsPool, CalcCostAlongLine) {
  IcfModel<double> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<double>& welds = model.weld_constraints_pool();
  welds.Resize(1);

  const Vector3<double> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<double> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<double> p_PoQo_W(0.05, 0.0, 0.0);
  const Vector3<double> a_PQ_W(0.0, 0.0, 0.01);

  welds.Set(0, 0, 1, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  const VectorXd v = model.v0();
  model.CalcData(v, &data);

  // Search direction w.
  const VectorXd w = VectorXd::LinSpaced(model.num_velocities(), 0.1, 0.5);

  // Precompute search direction data.
  IcfSearchDirectionData<double> search_direction_data;
  model.CalcSearchDirectionData(data, w, &search_direction_data);

  // Evaluate cost along line at α = 0.
  double dcost, d2cost;
  double cost0 = model.CalcCostAlongLine(0.0, data, search_direction_data,
                                         &dcost, &d2cost);

  // Evaluate at a small α to verify derivative numerically.
  const double h = 1e-7;
  double dcost_h, d2cost_h;
  double cost_h = model.CalcCostAlongLine(h, data, search_direction_data,
                                          &dcost_h, &d2cost_h);

  const double numerical_dcost = (cost_h - cost0) / h;
  // The derivative should be reasonably accurate.
  EXPECT_NEAR(dcost, numerical_dcost, 1e-4 * (1.0 + std::abs(dcost)));
}

/* Verifies gradient consistency using AutoDiff. */
GTEST_TEST(WeldConstraintsPool, GradientConsistency) {
  IcfModel<double> model;
  MakeModelForWeld(&model);

  WeldConstraintsPool<double>& welds = model.weld_constraints_pool();
  welds.Resize(1);

  const Vector3<double> p_AP_W(0.1, 0.0, 0.0);
  const Vector3<double> p_BQ_W(0.0, 0.0, 0.0);
  const Vector3<double> p_PoQo_W(0.05, 0.0, 0.0);
  const Vector3<double> a_PQ_W(0.0, 0.0, 0.01);

  welds.Set(0, 0, 1, p_AP_W, p_BQ_W, p_PoQo_W, a_PQ_W);

  model.SetSparsityPattern();

  IcfData<double> data;
  model.ResizeData(&data);
  const VectorXd v = model.v0();
  model.CalcData(v, &data);

  // Compute the gradient numerically by perturbing each velocity.
  const int nv = model.num_velocities();
  VectorXd numerical_gradient(nv);
  const double h = 1e-4;
  IcfData<double> data_pert;
  model.ResizeData(&data_pert);

  for (int i = 0; i < nv; ++i) {
    VectorXd v_plus = v;
    v_plus(i) += h;
    model.CalcData(v_plus, &data_pert);
    double cost_plus = data_pert.cost();

    VectorXd v_minus = v;
    v_minus(i) -= h;
    model.CalcData(v_minus, &data_pert);
    double cost_minus = data_pert.cost();
    numerical_gradient(i) = (cost_plus - cost_minus) / (2.0 * h);
  }

  // N.B. We need to recompute data at v since the perturbation loop
  // overwrites data_pert.
  model.CalcData(v, &data);
  const VectorXd& analytical_gradient = data.gradient();

  // Central differences with step h have O(h²) truncation error in absolute
  // terms. We use absolute tolerance since some gradient entries are near zero,
  // making relative tolerance unreliable.
  const double tol = h * h;
  EXPECT_TRUE(CompareMatrices(analytical_gradient, numerical_gradient, tol,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
