#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"

using drake::math::RigidTransformd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

const double kEps = std::numeric_limits<double>::epsilon();

template <typename T>
void MakeModel(PooledSapModel<T>* model, bool single_clique = false) {
  const double time_step = 0.01;
  const int nv = 18;

  std::unique_ptr<PooledSapParameters<T>> params = model->ReleaseParameters();
  params->time_step = time_step;
  params->A.Clear();
  params->J_WB.Clear();
  params->v0 = VectorX<T>::Zero(nv);

  // Push back mass matrices for three cliques.
  const Matrix6<T> A1 = 0.3 * Matrix6<T>::Identity();
  const Matrix6<T> A2 = 2.3 * Matrix6<T>::Identity();
  const Matrix6<T> A3 = 1.5 * Matrix6<T>::Identity();
  if (single_clique) {
    MatrixX<T> A = MatrixX<T>::Identity(nv, nv);
    A.template block<6, 6>(0, 0) = A1;
    A.template block<6, 6>(6, 6) = A2;
    A.template block<6, 6>(12, 12) = A3;
    params->A.Add(nv, nv) = A;
  } else {
    params->A.Add(6, 6) = A1;
    params->A.Add(6, 6) = A2;
    params->A.Add(6, 6) = A3;
  }
  params->r = VectorX<T>::LinSpaced(nv, -0.5, 0.5);

  // Three bodies on different cliques. Three bodies + anchored-body (world).
  if (single_clique) {
    params->body_cliques = {-1, 0, 0, 0};
  } else {
    params->body_cliques = {-1, 0, 1, 2};
  }

  // We use non-identity Jacobians to stress-test the algebra.
  params->body_is_floating = {0, 0, 0, 0};
  params->body_mass = {1.0e20, 0.3, 2.3, 1.5};  // First body is the world.
  const Matrix6<T> J_WB0 = VectorX<T>::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);
  const Matrix6<T> J_WB1 = 1.5 * J_WB0 + 0.1 * Matrix6<T>::Identity();
  const Matrix6<T> J_WB2 = J_WB0.transpose();
  params->J_WB.Add(6, 6) = Matrix6<T>::Identity();  // World.

  if (single_clique) {
    auto J0 = params->J_WB.Add(6, 18);
    J0.setZero();
    J0.template block<6, 6>(0, 0) = J_WB0;
    auto J1 = params->J_WB.Add(6, 18);
    J1.setZero();
    J1.template block<6, 6>(0, 6) = J_WB1;
    auto J2 = params->J_WB.Add(6, 18);
    J2.setZero();
    J2.template block<6, 6>(0, 12) = J_WB2;
  } else {
    params->J_WB.Add(6, 6) = J_WB0;
    params->J_WB.Add(6, 6) = J_WB1;
    params->J_WB.Add(6, 6) = J_WB2;
  }

  // None of the bodies are marked as floating
  params->body_is_floating = {0, 0, 0, 0};

  // Scale factor for convergence check.
  params->D = VectorX<T>::Ones(nv);

  // There is no actuation in this problem.
  params->effort_limits.resize(nv);
  if (single_clique) {
    params->clique_nu = {0};
  } else {
    params->clique_nu = {0, 0, 0};
  }

  // Reset model.
  model->ResetParameters(std::move(params));

  // Add patches.
  const T dissipation = 50.0;
  const T stiffness = 1.0e6;
  const T friction = 0.5;

  typename PooledSapModel<T>::PatchConstraintsPool& patches =
      model->patch_constraints_pool();

  // Allocate space
  std::vector<int> num_pairs_per_patch = {1, 1, 1};
  patches.Resize(num_pairs_per_patch);

  // first patch
  {
    const Vector3<T> p_AB_W(0.1, 0.0, 0.0);
    patches.AddPatch(0 /* patch index */, 1 /* body A */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.AddPair(0 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Single clique patch.
  {
    const Vector3<T> p_AB_W(0.0, 0.05, 0.0);  // A = World.
    patches.AddPatch(1 /* patch index */, 0 /* World */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(0.0, 1.0, 0.0);
    const Vector3<T> p_BC_W(0.0, -0.05, 0.0);
    const T fn0 = 1.5;
    patches.AddPair(1 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Third patch
  {
    const Vector3<T> p_AB_W(-0.1, 0.0, 0.0);
    patches.AddPatch(2 /* patch index */, 3 /* World */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(-1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.AddPair(2 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Establish the sparsity pattern.
  model->SetSparsityPattern();
}

GTEST_TEST(PooledSapModel, Construction) {
  PooledSapModel<double> model;
  MakeModel(&model);
  EXPECT_EQ(model.num_bodies(), 4);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  // Updating on the same size model should cause no new allocations.
  // TODO(amcastro-tri): Creating Aldlt_ within ResetParameters triggers
  // allocations.
  {
    drake::test::LimitMalloc guard({.max_num_allocations = 36});
    MakeModel(&model);
  }
}

GTEST_TEST(PooledSapModel, CalcData) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);
  const int nv = model.num_velocities();

  SapData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());

  VectorXd v_values = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);

  // fmt::print("Mom. Cost: {}\n", data.cache().momentum_cost.value());
  const double cost_value = data.cache().cost.value();
  const VectorXd cost_derivatives = data.cache().cost.derivatives();
  const VectorXd gradient_value = math::ExtractValue(data.cache().gradient);

  fmt::print("Cost: {}\n", cost_value);
  fmt::print("Gradient: {}\n", fmt_eigen(gradient_value.transpose()));
  fmt::print(
      "|grad-grad_ref|/|grad| = {}\n",
      (gradient_value - cost_derivatives).norm() / gradient_value.norm());

  EXPECT_TRUE(CompareMatrices(gradient_value, cost_derivatives, 8 * kEps,
                              MatrixCompareType::relative));
}

/* The Hessian has a single dense block (one clique). */
GTEST_TEST(PooledSapModel, CalcDenseHessian) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, true /* single cliques */);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);
  const int nv = model.num_velocities();

  SapData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());

  VectorXd v_values = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);
  MatrixXd gradient_derivatives = math::ExtractGradient(data.cache().gradient);

  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));

  // Now we'll only update the values.
  v_values = VectorXd::LinSpaced(nv, -3.0, 7.0);
  math::InitializeAutoDiff(v_values, &v);
  model.CalcData(v, &data);
  model.UpdateHessian(data, hessian.get());

  gradient_derivatives = math::ExtractGradient(data.cache().gradient);
  hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));
}

/* Hessian has the sparsity structure inherited from the model (with multiple
 * cliques). */
GTEST_TEST(PooledSapModel, CalcSparseHessian) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);
  const int nv = model.num_velocities();

  SapData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());

  VectorXd v_values = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);
  MatrixXd gradient_derivatives = math::ExtractGradient(data.cache().gradient);

  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));

  // Now we'll only update the values.
  v_values = VectorXd::LinSpaced(nv, -3.0, 7.0);
  math::InitializeAutoDiff(v_values, &v);
  model.CalcData(v, &data);
  model.UpdateHessian(data, hessian.get());

  gradient_derivatives = math::ExtractGradient(data.cache().gradient);
  hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));
}

GTEST_TEST(PooledSapModel, SingleVsMultipleCliques) {
  PooledSapModel<double> model_single;
  MakeModel(&model_single, true);
  EXPECT_EQ(model_single.num_cliques(), 1);
  EXPECT_EQ(model_single.num_velocities(), 18);
  EXPECT_EQ(model_single.num_constraints(), 3);

  PooledSapModel<double> model_multiple;
  MakeModel(&model_multiple, false);
  EXPECT_EQ(model_multiple.num_cliques(), 3);
  EXPECT_EQ(model_multiple.num_velocities(), 18);
  EXPECT_EQ(model_multiple.num_constraints(), 3);

  // Allocate data.
  SapData<double> data_single;
  model_single.ResizeData(&data_single);
  SapData<double> data_multiple;
  model_multiple.ResizeData(&data_multiple);

  // Compute data.
  const int nv = model_single.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model_single.CalcData(v, &data_single);
  model_multiple.CalcData(v, &data_multiple);

  // Hessian and gradient should be the same, regardless of sparsity.
  EXPECT_NEAR(data_single.cache().momentum_cost,
              data_multiple.cache().momentum_cost, kEps);
  EXPECT_NEAR(data_single.cache().cost, data_multiple.cache().cost, kEps);
  EXPECT_TRUE(CompareMatrices(data_single.cache().gradient,
                              data_multiple.cache().gradient, kEps,
                              MatrixCompareType::relative));
}

GTEST_TEST(PooledSapModel, LimitMallocOnCalcData) {
  PooledSapModel<double> model;
  MakeModel(&model, false /* each body in its own clique */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  SapData<double> data;
  model.ResizeData(&data);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model.CalcData(v, &data);

  // Recomputing on the same data should cause no new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

GTEST_TEST(PooledSapModel, CostAlongLine) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, false /* Multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  // Allocate data, and additional scratch.
  SapData<AutoDiffXd> data, scratch;
  model.ResizeData(&data);
  model.ResizeData(&scratch);

  // Compute data.
  const int nv = model.num_velocities();
  const VectorX<AutoDiffXd> v = VectorXd::LinSpaced(nv, 0.05, 0.01);
  model.CalcData(v, &data);

  // Allocate search direction.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(
      nv, 0.1, -0.2);  // Arbitrary search direction.
  SearchDirectionData<AutoDiffXd> search_data;
  model.UpdateSearchDirection(data, w, &search_data);

  // Try-out a set of arbitrary values.
  for (double alpha_value : {-0.45, 0., 0.15, 0.34, 0.93, 1.32}) {
    const AutoDiffXd alpha = {
        alpha_value, VectorXd::Ones(1) /* This is the independent variable */};

    const VectorX<AutoDiffXd> v_alpha = v + alpha * w;
    model.CalcData(v_alpha, &scratch);
    const double cost_expected = scratch.cache().cost.value();
    const double momentum_cost_expected = scratch.cache().momentum_cost.value();
    const double dcost_expected = scratch.cache().cost.derivatives()[0];
    const VectorXd w_times_H = math::ExtractGradient(scratch.cache().gradient);
    const double d2cost_expected = w_times_H.dot(math::ExtractValue(w));

    // Verify pre-computed terms are correct.
    const AutoDiffXd a = search_data.a;
    const AutoDiffXd b = search_data.b;
    const AutoDiffXd c = search_data.c;
    const double momentum_cost =
        (a * alpha * alpha / 2.0 + b * alpha + c).value();
    EXPECT_NEAR(momentum_cost, momentum_cost_expected,
                8 * kEps * abs(momentum_cost_expected));

    AutoDiffXd dcost, d2cost;
    const AutoDiffXd cost = model.CalcCostAlongLine(alpha, data, search_data,
                                                    &scratch, &dcost, &d2cost);
    EXPECT_NEAR(cost.value(), cost_expected, 8 * kEps * abs(cost_expected));
    EXPECT_NEAR(dcost.value(), dcost_expected, 8 * kEps * abs(dcost_expected));
    EXPECT_NEAR(d2cost.value(), d2cost_expected,
                8 * kEps * abs(d2cost_expected));

    fmt::print("alpha: {}. cost: {}. dcost: {}. d2cost: {}\n", alpha,
               cost.value(), dcost.value(), d2cost.value());
  }
}

GTEST_TEST(PooledSapModel, GainConstraint) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  SapData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());
  EXPECT_EQ(data.num_gains(), 0);

  // At this point there should be no gain constraints.
  EXPECT_EQ(model.num_gain_constraints(), 0);

  // Add gain constraints.
  auto& gain_constraints = model.gain_constraints_pool();

  // Allocate space for two gain constraints (on cliques 0 and 2).
  const std::vector<int> actuated_clique_sizes = {model.clique_size(0),
                                                  model.clique_size(2)};
  gain_constraints.Resize(actuated_clique_sizes);

  // On clique 0:
  const int nv0 = model.clique_size(0);
  VectorX<AutoDiffXd> K0 = 1.1 * VectorX<AutoDiffXd>::Ones(nv0);
  VectorX<AutoDiffXd> u0 = -6.0 * VectorX<AutoDiffXd>::Ones(nv0);
  VectorX<AutoDiffXd> e0 = 0.9 * VectorX<AutoDiffXd>::Ones(nv0);
  gain_constraints.Add(0, 0, K0, u0, e0);

  // On clique 2:
  const int nv2 = model.clique_size(2);
  VectorX<AutoDiffXd> K2 = 2.3 * VectorX<AutoDiffXd>::Ones(nv2);
  VectorX<AutoDiffXd> u2 = -0.5 * VectorX<AutoDiffXd>::Ones(nv2);
  VectorX<AutoDiffXd> e2 = 11.1 * VectorX<AutoDiffXd>::Ones(nv2);
  // Test cases with zero gain.
  K2(1) = K2(2) = K2(4) = 0.0;
  u2(1) = -13.5;  // bias below limit.
  u2(2) = -5.5;   // bias within limits.
  u2(4) = 15.2;   // bias above limits.
  gain_constraints.Add(1, 2, K2, u2, e2);

  EXPECT_EQ(model.num_gain_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 5);

  // Resize data to include gain constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.num_gains(), 2);

  const int nv = model.num_velocities();
  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const GainConstraintsDataPool<AutoDiffXd> gains_data =
      data.cache().gain_constraints_data;
  EXPECT_EQ(gains_data.num_constraints(), 2);

  const VectorXd K0_value = math::ExtractValue(K0);
  const VectorXd u0_value = math::ExtractValue(u0);
  const VectorXd e0_value = math::ExtractValue(e0);
  const VectorXd v0_value = v_value.segment<6>(0);  // Clique 0's values.
  VectorXd tau0_expected = -K0_value.cwiseProduct(v0_value) + u0_value;

  const VectorXd K2_value = math::ExtractValue(K2);
  const VectorXd u2_value = math::ExtractValue(u2);
  const VectorXd e2_value = math::ExtractValue(e2);
  const VectorXd v2_value = v_value.segment<6>(12);  // Clique 2's values.
  VectorXd tau2_expected = -K2_value.cwiseProduct(v2_value) + u2_value;

  fmt::print("tau0_expected: {}\n", fmt_eigen(tau0_expected.transpose()));
  fmt::print("tau2_expected: {}\n", fmt_eigen(tau2_expected.transpose()));

  // For this test we verify some values are below, within and above effort
  // limits.
  EXPECT_GT(tau0_expected(0), 0.9);
  EXPECT_GT(tau0_expected(1), 0.9);
  EXPECT_GT(tau0_expected(2), 0.9);
  EXPECT_GT(tau0_expected(3), 0.9);
  EXPECT_GT(tau0_expected(4), -0.9);
  EXPECT_LT(tau0_expected(4), 0.9);
  EXPECT_LT(tau0_expected(5), -0.9);
  // Apply the limit.
  tau0_expected(0) = tau0_expected(1) = tau0_expected(2) = tau0_expected(3) =
      0.9;
  tau0_expected(5) = -0.9;
  const VectorXd gamma0_expected = tau0_expected * model.time_step().value();

  const double cost_value = gains_data.cost().value();
  const VectorXd cost_gradient = gains_data.cost().derivatives();
  fmt::print("cost: {}\n", cost_value);
  fmt::print("gradient: {}\n", fmt_eigen(cost_gradient.transpose()));

  const VectorX<AutoDiffXd> gamma0 = gains_data.gamma(0);
  EXPECT_EQ(gamma0.size(), nv0);
  const VectorXd gamma0_value = math::ExtractValue(gamma0);
  fmt::print("gamma: {}\n", fmt_eigen(gamma0_value.transpose()));
  EXPECT_TRUE(CompareMatrices(gamma0_value, gamma0_expected, kEps,
                              MatrixCompareType::relative));

  // Gradient of the cost.
  const VectorXd minus_cost0_gradient = -cost_gradient.segment<6>(0);
  EXPECT_TRUE(CompareMatrices(gamma0_value, minus_cost0_gradient, kEps,
                              MatrixCompareType::relative));

  // Verify gradients for gain on clique 2.
  const VectorX<AutoDiffXd> gamma2 = gains_data.gamma(1 /* constraint index */);
  EXPECT_EQ(gamma2.size(), nv2);
  const VectorXd gamma2_value = math::ExtractValue(gamma2);
  fmt::print("gamma: {}\n", fmt_eigen(gamma2_value.transpose()));
  const VectorXd minus_cost2_gradient =
      -cost_gradient.segment<6>(12 /* first velocity index */);
  EXPECT_TRUE(CompareMatrices(gamma2_value, minus_cost2_gradient, kEps,
                              MatrixCompareType::relative));

  // Verify accumulated total cost and gradients.
  const double total_cost_value = data.cache().cost.value();
  const VectorXd total_cost_derivatives = data.cache().cost.derivatives();
  const VectorXd total_gradient_value =
      math::ExtractValue(data.cache().gradient);

  fmt::print("Cost: {}\n", total_cost_value);
  fmt::print("Gradient: {}\n", fmt_eigen(total_gradient_value.transpose()));
  fmt::print("|grad-grad_ref|/|grad| = {}\n",
             (total_gradient_value - total_cost_derivatives).norm() /
                 total_gradient_value.norm());

  EXPECT_TRUE(CompareMatrices(total_gradient_value, total_cost_derivatives,
                              2 * kEps, MatrixCompareType::relative));

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.cache().gradient);
  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));
}

GTEST_TEST(PooledSapModel, LimitConstraint) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  SapData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());
  EXPECT_EQ(data.num_limits(), 0);

  // At this point there should be no gain constraints.
  EXPECT_EQ(model.num_limit_constraints(), 0);

  // Add limit constraints.
  auto& limits = model.limit_constraints_pool();

  const int nv = model.num_velocities();
  VectorX<AutoDiffXd> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  // Limits on clique 0.
  int k = limits.Add(0 /* clique */, 1 /* dof */, q0(1), -0.5, 0.5);  // Below.
  EXPECT_EQ(k, 0);  // First constraint on clique 0.
  k = limits.Add(0 /* clique */, 4 /* dof */, q0(2), -1.5, -1.0);  // Above.
  EXPECT_EQ(k, 0);  // Still the same clique, same constraint.
  k = limits.Add(0 /* clique */, 3 /* dof */, q0(3), -1.0, 1.0);  // Within.
  EXPECT_EQ(k, 0);  // Still the same clique, same constraint.

  EXPECT_EQ(model.num_limit_constraints(), 1);
  EXPECT_EQ(model.num_constraints(), 4);

  // Limits on clique 2.
  k = limits.Add(2 /* clique */, 0 /* dof */, q0(12), 0.5, 1.5);  // Below.
  EXPECT_EQ(k, 1);  // First constraint on clique 2.
  k = limits.Add(2 /* clique */, 2 /* dof */, q0(14), -1.0, 0.5);  // Above.
  EXPECT_EQ(k, 1);  // Still the same clique, same constraint.
  k = limits.Add(2 /* clique */, 5 /* dof */, q0(17), 0.5, 1.5);  // Within.
  EXPECT_EQ(k, 1);  // Still the same clique, same constraint.

  EXPECT_EQ(model.num_limit_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 5);

  // Resize data to include limit constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.num_limits(), 2);

  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const AutoDiffXd dt = model.time_step();
  VectorX<AutoDiffXd> q = q0 + dt * v;
  fmt::print("q: {}\n", fmt_eigen(q.transpose()));

  const LimitConstraintsDataPool<AutoDiffXd> limits_data =
      data.cache().limit_constraints_data;
  EXPECT_EQ(limits_data.num_constraints(), 2);

  const double cost_value = limits_data.cost().value();
  const VectorXd cost_gradient = limits_data.cost().derivatives();
  fmt::print("cost: {}\n", cost_value);
  fmt::print("gradient: {}\n", fmt_eigen(cost_gradient.transpose()));

  // Impulses on constraint 0, clique 0.
  const VectorX<AutoDiffXd> gamma_lower0 = limits_data.gamma_lower(0);
  const VectorX<AutoDiffXd> gamma_upper0 = limits_data.gamma_upper(0);
  // tau = Jᵀ⋅γ, and J = [1; -1] for limit constraints.
  const VectorX<AutoDiffXd> tau0 = gamma_lower0 - gamma_upper0;
  fmt::print("tau0: {}\n", fmt_eigen(tau0.transpose()));

  // Impulses on constraint 1, clique 2.
  const VectorX<AutoDiffXd> gamma_lower1 = limits_data.gamma_lower(1);
  const VectorX<AutoDiffXd> gamma_upper1 = limits_data.gamma_upper(1);
  // tau = Jᵀ⋅γ, and J = [1; -1] for limit constraints.
  const VectorX<AutoDiffXd> tau1 = gamma_lower1 - gamma_upper1;
  fmt::print("tau1: {}\n", fmt_eigen(tau1.transpose()));

  // Assemble all clique contributions into tau by hand.
  VectorX<AutoDiffXd> tau = VectorX<AutoDiffXd>::Zero(nv);
  tau.segment<6>(0) = tau0;
  tau.segment<6>(12) = tau1;

  const VectorXd tau_value = math::ExtractValue(tau);
  EXPECT_TRUE(CompareMatrices(cost_gradient, -tau_value, kEps,
                              MatrixCompareType::relative));

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.cache().gradient);
  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));
}

GTEST_TEST(PooledSapModel, CouplerConstraint) {
  PooledSapModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  SapData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());
  EXPECT_EQ(data.num_limits(), 0);

  // At this point there should be no coupler constraints.
  EXPECT_EQ(model.num_coupler_constraints(), 0);

  // Add coupler constraints.
  auto& couplers = model.coupler_constraints_pool();

  const int nv = model.num_velocities();
  VectorX<AutoDiffXd> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  // Coupler on clique 1.
  const auto q0_c1 = model.clique_segment(1, q0);
  const double rho1 = 2.5;
  const double offset1 = 0.1;
  int k = couplers.Add(1 /* clique */, 1 /* i */, 3 /* j */, q0_c1(1), q0_c1(3),
                       rho1, offset1);
  EXPECT_EQ(k, 0);
  EXPECT_EQ(model.num_coupler_constraints(), 1);
  EXPECT_EQ(model.num_constraints(), 4);

  // Resize data to include limit constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.num_couplers(), 1);

  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const double dt = model.time_step().value();
  VectorX<AutoDiffXd> q = q0 + dt * v;
  const auto q_c1 = model.clique_segment(1, q);
  const auto v_c1 = model.clique_segment(1, v);
  // const VectorXd q_value = math::ExtractValue(q);
  fmt::print("q: {}\n", fmt_eigen(q.transpose()));

  // Compute regularization.
  const double beta =
      0.1;  // Keep in sync with hard-coded value in the implementation.
  // const double eps = beta * beta / (4 * M_PI * M_PI) * (1 + beta / M_PI);
  const double m1 = 2.3;                     // "mass" for clique 1.
  const double w1 = (1 + rho1 * rho1) / m1;  // Delassus for clique 1.
  const double m1_eff = 1.0 / w1;            // "Effective" mass for clique 1.
  // const double R0 = eps * w0;
  const double omega_near_rigid =
      2 * M_PI / (beta * dt);  // period_nr = beta * dt, by definition.
  const double k1 = m1_eff * omega_near_rigid * omega_near_rigid;
  const double tau1 = beta / M_PI * dt;

  const double R1 = 1.0 / (dt * (dt + tau1) * k1);
  fmt::print("w1: {}\n", w1);
  fmt::print("R1: {}\n", R1);

  const CouplerConstraintsDataPool<AutoDiffXd> couplers_data =
      data.cache().coupler_constraints_data;
  EXPECT_EQ(couplers_data.num_constraints(), 1);

  const double cost_value = couplers_data.cost().value();
  const VectorXd cost_gradient = couplers_data.cost().derivatives();
  fmt::print("cost: {}\n", cost_value);
  fmt::print("gradient: {}\n", fmt_eigen(cost_gradient.transpose()));

  // Expected impulse.
  const AutoDiffXd g0 = q_c1(1) - rho1 * q_c1(3) - offset1;
  const AutoDiffXd gdot0 = v_c1(1) - rho1 * v_c1(3);
  const AutoDiffXd gamma0 = -dt * k1 * (g0 + tau1 * gdot0);
  fmt::print("gamma_expected: {}\n", gamma0.value());
  VectorXd tau_expected = VectorXd::Zero(nv);
  tau_expected(6 + 1) = gamma0.value();
  tau_expected(6 + 3) = -rho1 * gamma0.value();
  EXPECT_TRUE(CompareMatrices(-cost_gradient, tau_expected, kEps,
                              MatrixCompareType::relative));

  const double gamma = couplers_data.gamma(0).value();
  EXPECT_NEAR(gamma, gamma0.value(), kEps);

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.cache().gradient);
  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 10 * kEps,
                              MatrixCompareType::relative));

  // CalcCostAlongLine()
  // Allocate search direction.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(
      nv, 0.1, -0.2);  // Arbitrary search direction.
  SearchDirectionData<AutoDiffXd> search_data;
  SapData<AutoDiffXd> scratch;
  model.ResizeData(&scratch);

  // Set data with constant value of v.
  VectorX<AutoDiffXd> v_constant =
      VectorX<AutoDiffXd>::LinSpaced(nv, -10, 10.0);
  model.CalcData(v_constant, &data);
  model.UpdateSearchDirection(data, w, &search_data);

  const AutoDiffXd alpha = {
      0.35 /* arbitrary value */,
      VectorXd::Ones(1) /* This is the independent variable */};
  AutoDiffXd dcost, d2cost;
  const AutoDiffXd cost = model.CalcCostAlongLine(alpha, data, search_data,
                                                  &scratch, &dcost, &d2cost);

  EXPECT_NEAR(dcost.value(), cost.derivatives()[0], kEps);
  EXPECT_NEAR(d2cost.value(), dcost.derivatives()[0], kEps * d2cost.value());
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
