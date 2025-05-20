#include <limits>

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
  if (single_clique) {
    params->A.PushBack(MatrixX<T>::Identity(nv, nv));
  } else {
    params->A.Add(6, 6) = Matrix6<T>::Identity();
    params->A.Add(6, 6) = Matrix6<T>::Identity();
    params->A.Add(6, 6) = Matrix6<T>::Identity();
  }
  params->r = VectorX<T>::LinSpaced(nv, -0.5, 0.5);

  // Three bodies on different cliques. Three bodies + anchored-body (world).
  if (single_clique) {
    params->body_cliques = {-1, 0, 0, 0};
  } else {
    params->body_cliques = {-1, 0, 1, 2};
  }
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

  // Reset model.
  model->ResetParameters(std::move(params));

  // Add patches.
  const T dissipation = 50.0;
  const T stiffness = 1000.0;
  const T friction = 0.5;

  typename PooledSapModel<T>::PatchConstraintsPool& patches =
      model->patch_constraints_pool();
  patches.Clear();

  // first patch
  {
    const Vector3<T> p_AB_W(0.1, 0.0, 0.0);
    patches.AddPatch(1 /* body A */, 2 /* body B */, dissipation, friction,
                     p_AB_W);

    const Vector3<T> nhat_AB_W(1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.AddPair(p_BC_W, nhat_AB_W, fn0, stiffness);
  }

  // Single clique patch.
  {
    const Vector3<T> p_AB_W(0.0, 0.05, 0.0);  // A = World.
    patches.AddPatch(0 /* World */, 2 /* body B */, dissipation, friction,
                     p_AB_W);

    const Vector3<T> nhat_AB_W(0.0, 1.0, 0.0);
    const Vector3<T> p_BC_W(0.0, -0.05, 0.0);
    const T fn0 = 1.5;
    patches.AddPair(p_BC_W, nhat_AB_W, fn0, stiffness);
  }

  // Third patch
  {
    const Vector3<T> p_AB_W(-0.1, 0.0, 0.0);
    patches.AddPatch(3 /* World */, 2 /* body B */, dissipation, friction,
                     p_AB_W);

    const Vector3<T> nhat_AB_W(-1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.AddPair(p_BC_W, nhat_AB_W, fn0, stiffness);
  }
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
  MakeModel(&model);
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
  const MatrixXd gradient_derivatives =
      math::ExtractGradient(data.cache().gradient);
  const MatrixXd hessian_value = math::ExtractValue(data.cache().hessian);

  fmt::print("Cost: {}\n", cost_value);
  fmt::print("Gradient: {}\n", fmt_eigen(gradient_value.transpose()));
  fmt::print(
      "|grad-grad_ref|/|grad| = {}\n",
      (gradient_value - cost_derivatives).norm() / gradient_value.norm());
  fmt::print("Hessian:\n{}\n", fmt_eigen(hessian_value));
  fmt::print("|H-Href| = {}\n", (hessian_value - gradient_derivatives).norm());

  EXPECT_TRUE(CompareMatrices(gradient_value, cost_derivatives, 8 * kEps,
                              MatrixCompareType::relative));
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
  EXPECT_TRUE(CompareMatrices(data_single.cache().hessian,
                              data_multiple.cache().hessian, kEps,
                              MatrixCompareType::relative));
}

GTEST_TEST(PooledSapModel, LimitMallocOnCalcData) {
  PooledSapModel<double> model;
  MakeModel(&model);
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

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
