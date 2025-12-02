#include "drake/multibody/contact_solvers/icf/icf_model.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
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

/* Sets up a simple dummy model, without any constraints. This can create two
versions of the same model: one with everything stuffed into a single clique,
and one with three separate cliques. */
template <typename T>
void MakeUnconstrainedModel(IcfModel<T>* model, bool single_clique = false,
                            double time_step = 0.01) {
  const int nv = 18;

  // Release the parameters in order to set them.
  std::unique_ptr<IcfParameters<T>> params = model->ReleaseParameters();
  ASSERT_TRUE(params != nullptr);

  params->time_step = time_step;
  params->v0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  // Define a sparse mass matrix with three cliques.
  const Matrix6<T> A1 = 0.3 * Matrix6<T>::Identity();
  const Matrix6<T> A2 = 2.3 * Matrix6<T>::Identity();
  const Matrix6<T> A3 = 1.5 * Matrix6<T>::Identity();

  // Set the dense mass matrix.
  MatrixX<T>& M0 = params->M0;
  M0 = MatrixX<T>::Identity(nv, nv);
  M0.template block<6, 6>(0, 0) = A1;
  M0.template block<6, 6>(6, 6) = A2;
  M0.template block<6, 6>(12, 12) = A3;

  // Define joint damping D₀.
  params->D0 = VectorX<T>::Constant(nv, 0.1);

  // Set coriolis, centrifugal, and gravitational terms k₀.
  params->k0 = VectorX<T>::LinSpaced(nv, -1.0, 1.0);

  // Define the clique structure for the sparse linearized dynamics matrix A.
  std::vector<int>& clique_sizes = params->clique_sizes;
  std::vector<int>& clique_start = params->clique_start;
  if (single_clique) {
    clique_sizes = {nv};
    clique_start = {0, nv};
  } else {
    clique_sizes = {6, 6, 6};
    clique_start = {0, 6, 12, nv};
  }

  // We use non-identity Jacobians to stress-test the algebra.
  if (single_clique) {
    // Floating bodies don't make sense in a single-clique model.
    params->body_is_floating = {0, 0, 0, 0};
  } else {
    params->body_is_floating = {0, 0, 1, 0};
  }
  params->body_mass = {1.0e20, 0.3, 2.3, 1.5};  // First body is the world.
  const Matrix6<T> J_WB0 = VectorX<T>::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);
  const Matrix6<T> J_WB1 = 1.5 * J_WB0 + 0.1 * Matrix6<T>::Identity();
  const Matrix6<T> J_WB2 = J_WB0.transpose();

  if (single_clique) {
    params->J_WB.Resize(4, 6, 18);
    params->J_WB[0].setZero();
    params->J_WB[0].template block<6, 6>(0, 0) = Matrix6<T>::Identity();
    typename EigenPool<Matrix6X<T>>::MatrixView J0 = params->J_WB[1];
    J0.setZero();
    J0.template block<6, 6>(0, 0) = J_WB0;
    typename EigenPool<Matrix6X<T>>::MatrixView J1 = params->J_WB[2];
    J1.setZero();
    J1.template block<6, 6>(0, 6) = J_WB1;
    typename EigenPool<Matrix6X<T>>::MatrixView J2 = params->J_WB[3];
    J2.setZero();
    J2.template block<6, 6>(0, 12) = J_WB2;
  } else {
    params->J_WB.Resize(4, 6, 6);
    params->J_WB[0] = Matrix6<T>::Identity();  // World.
    params->J_WB[1] = J_WB0;
    params->J_WB[2] = J_WB1;
    params->J_WB[3] = J_WB2;
  }

  // We have three bodies and the world. The world is body -1.
  if (single_clique) {
    // All bodies in the same clique (clique 0).
    params->body_to_clique = {-1, 0, 0, 0};
  } else {
    // Each body in its own clique.
    params->body_to_clique = {-1, 0, 1, 2};
  }

  model->ResetParameters(std::move(params));
}

/* Checks that a default constructed model is empty. */
GTEST_TEST(IcfModel, EmptyModel) {
  IcfModel<double> model;
  EXPECT_EQ(model.num_bodies(), 0);
  EXPECT_EQ(model.num_cliques(), 0);
  EXPECT_EQ(model.num_velocities(), 0);
  EXPECT_EQ(model.num_constraints(), 0);
  EXPECT_EQ(model.max_clique_size(), 0);

  EXPECT_EQ(model.M0().rows(), 0);
  EXPECT_EQ(model.M0().cols(), 0);
  EXPECT_EQ(model.v0().size(), 0);
  EXPECT_EQ(model.D0().size(), 0);
  EXPECT_EQ(model.k0().size(), 0);
  EXPECT_EQ(model.scale_factor().size(), 0);
  EXPECT_EQ(model.r().size(), 0);
}

/* Checks that the model can be constructed with minimal heap allocations. */
GTEST_TEST(IcfModel, LimitMallocOnModelUpdate) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);  // Memory is allocated here.
  EXPECT_EQ(model.num_bodies(), 4);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);
  EXPECT_EQ(model.max_clique_size(), 6);

  EXPECT_EQ(model.M0().rows(), 18);
  EXPECT_EQ(model.M0().cols(), 18);
  EXPECT_EQ(model.v0().size(), 18);
  EXPECT_EQ(model.D0().size(), 18);
  EXPECT_EQ(model.k0().size(), 18);
  EXPECT_EQ(model.scale_factor().size(), 18);
  EXPECT_EQ(model.r().size(), 18);
  EXPECT_EQ(model.A(0).rows(), 6);
  EXPECT_EQ(model.A(0).cols(), 6);

  // Updating on the same size model should cause no new allocations.
  {
    drake::test::LimitMalloc guard;
    MakeUnconstrainedModel(&model);
  }
}

/* Checks that model.CalcData does not incur any heap allocations. */
GTEST_TEST(IcfModel, LimitMallocOnCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);

  IcfData<double> data;
  model.ResizeData(&data);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

/* Iterates over each body to check sizes and such. */
GTEST_TEST(IcfModel, PerBodyElements) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  EXPECT_EQ(model.num_bodies(), 4);
  EXPECT_EQ(model.num_velocities(), 18);

  const VectorXd& v = model.v0();

  // We'll fill this with ones, clique by clique.
  VectorXd mutable_vector = VectorXd::Zero(model.num_velocities());

  int num_anchored = 0;
  int num_floating = 0;
  for (int b = 0; b < model.num_bodies(); ++b) {
    const Vector6d& V_WB = model.V_WB0(b);
    const int c = model.body_to_clique(b);
    const int clique_nv = model.clique_size(c);

    if (model.is_anchored(b)) {
      ++num_anchored;
      EXPECT_LT(c, 0);
      EXPECT_EQ(clique_nv, 0);

      // Spatial velocity should be zero.
      EXPECT_TRUE(CompareMatrices(V_WB, Vector6d::Zero(), kEpsilon,
                                  MatrixCompareType::relative));
    } else if (model.is_floating(b)) {
      ++num_floating;
      EXPECT_GE(c, 0);
      EXPECT_EQ(clique_nv, 6);

      // Spatial velocity should equal v0 segment.
      const Vector6d& V_WB_expected = model.clique_segment(c, v);
      EXPECT_TRUE(CompareMatrices(V_WB, V_WB_expected, kEpsilon,
                                  MatrixCompareType::relative));

    } else {
      EXPECT_GE(c, 0);

      // Spatial velocity should equal J_WB * v0 segment.
      const Matrix6X<double>& J_WB = model.J_WB(b);
      EXPECT_EQ(J_WB.cols(), clique_nv);
      const Vector6d V_WB_expected = J_WB * model.clique_segment(c, v);
      EXPECT_TRUE(CompareMatrices(V_WB, V_WB_expected, kEpsilon,
                                  MatrixCompareType::relative));
    }

    EXPECT_GT(model.body_mass(b), 0.0);
    if (!model.is_anchored(b)) {
      EXPECT_GT(model.clique_delassus_approx(c).minCoeff(), 0.0);
      model.mutable_clique_segment(c, &mutable_vector) +=
          VectorXd::Ones(clique_nv);
    }
  }

  // Our mutable vector should now be filled with ones in each clique segment.
  // Any zeros would indicate that a segment was missed, while twos would
  // indicate that a segment was double counted.
  EXPECT_TRUE(CompareMatrices(mutable_vector,
                              VectorXd::Ones(model.num_velocities()), kEpsilon,
                              MatrixCompareType::relative));

  // We should have exactly one floating body and one anchored body.
  EXPECT_EQ(num_floating, 1);
  EXPECT_EQ(num_anchored, 1);
}

/* Checks that gradients are computed correctly for an unconstrained problem. */
GTEST_TEST(IcfModel, CalcGradients) {
  IcfModel<AutoDiffXd> model;
  // TODO(vincekurtz): run this test with constraints once they land.
  MakeUnconstrainedModel(&model);
  const int nv = model.num_velocities();

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(model.num_constraints(), 0);

  VectorXd v_values = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);

  const VectorXd cost_derivatives = data.cost().derivatives();
  const VectorXd gradient_value = math::ExtractValue(data.gradient());

  EXPECT_TRUE(CompareMatrices(gradient_value, cost_derivatives, 8 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Checks the Hessian for a problem with a single clique. */
GTEST_TEST(IcfModel, CalcDenseHessian) {
  IcfModel<AutoDiffXd> model;
  // TODO(vincekurtz): run this test with constraints once they land.
  MakeUnconstrainedModel(&model, true /* single cliques */);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), 18);
  const int nv = model.num_velocities();

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());

  VectorXd v_values = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());

  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 8 * kEpsilon,
                              MatrixCompareType::relative));

  // Now we'll only update the values.
  v_values = VectorXd::LinSpaced(nv, -3.0, 7.0);
  math::InitializeAutoDiff(v_values, &v);
  model.CalcData(v, &data);
  model.UpdateHessian(data, hessian.get());

  gradient_derivatives = math::ExtractGradient(data.gradient());
  hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 8 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Checks that we get the same result for the same problem, regardless of how we
break out the cliques. */
GTEST_TEST(IcfModel, SingleVsMultipleCliques) {
  IcfModel<double> model_single;
  // TODO(vincekurtz): run this test with constraints once they land.
  MakeUnconstrainedModel(&model_single, true);
  model_single.SetSparsityPattern();
  EXPECT_EQ(model_single.num_cliques(), 1);
  EXPECT_EQ(model_single.num_velocities(), 18);

  IcfModel<double> model_multiple;
  MakeUnconstrainedModel(&model_multiple, false);
  model_multiple.SetSparsityPattern();
  EXPECT_EQ(model_multiple.num_cliques(), 3);
  EXPECT_EQ(model_multiple.num_velocities(), 18);

  // Allocate data.
  IcfData<double> data_single;
  model_single.ResizeData(&data_single);
  IcfData<double> data_multiple;
  model_multiple.ResizeData(&data_multiple);

  // Compute data.
  const int nv = model_single.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model_single.CalcData(v, &data_single);
  model_multiple.CalcData(v, &data_multiple);

  // Compute Hessians.
  auto hessian_single = model_single.MakeHessian(data_single);
  auto hessian_multiple = model_multiple.MakeHessian(data_multiple);
  MatrixXd H_single = hessian_single->MakeDenseMatrix();
  MatrixXd H_multiple = hessian_multiple->MakeDenseMatrix();

  // Verify sparsity patterns.
  EXPECT_EQ(hessian_single->block_rows(), 1);
  EXPECT_EQ(hessian_single->block_cols(), 1);
  EXPECT_EQ(hessian_multiple->block_rows(), 3);
  EXPECT_EQ(hessian_multiple->block_cols(), 3);

  EXPECT_EQ(model_single.sparsity_pattern().block_sizes(),
            std::vector<int>({18}));
  EXPECT_EQ(model_multiple.sparsity_pattern().block_sizes(),
            std::vector<int>({6, 6, 6}));

  // Cost, gradient, and Hessian should be the same regardless of sparsity.
  EXPECT_NEAR(data_single.cost(), data_multiple.cost(), kEpsilon);
  EXPECT_TRUE(CompareMatrices(data_single.gradient(), data_multiple.gradient(),
                              kEpsilon, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(H_single, H_multiple, kEpsilon,
                              MatrixCompareType::relative));
}

/* Checks that our exact linesearch computations are correct. */
GTEST_TEST(IcfModel, CalcCostAlongLine) {
  IcfModel<AutoDiffXd> model;
  // TODO(vincekurtz): run this test with constraints once they land.
  MakeUnconstrainedModel(&model);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);

  // Allocate data, and additional scratch.
  IcfData<AutoDiffXd> data, scratch;
  model.ResizeData(&data);
  model.ResizeData(&scratch);

  // Compute data.
  const int nv = model.num_velocities();
  const VectorX<AutoDiffXd> v = VectorXd::LinSpaced(nv, 0.05, 0.01);
  model.CalcData(v, &data);

  // Allocate search direction.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(
      nv, 0.1, -0.2);  // Arbitrary search direction.
  IcfSearchDirectionData<AutoDiffXd> search_data;
  model.CalcSearchDirectionData(data, w, &search_data);

  // Try-out a set of arbitrary values.
  for (double alpha_value : {-0.45, 0., 0.15, 0.34, 0.93, 1.32}) {
    const AutoDiffXd alpha = {
        alpha_value, VectorXd::Ones(1) /* This is the independent variable */};

    const VectorX<AutoDiffXd> v_alpha = v + alpha * w;
    model.CalcData(v_alpha, &scratch);
    const double cost_expected = scratch.cost().value();
    const double momentum_cost_expected = scratch.momentum_cost().value();
    const double dcost_expected = scratch.cost().derivatives()[0];
    const VectorXd w_times_H = math::ExtractGradient(scratch.gradient());
    const double d2cost_expected = w_times_H.dot(math::ExtractValue(w));

    // Verify pre-computed terms are correct.
    const AutoDiffXd a = search_data.a;
    const AutoDiffXd b = search_data.b;
    const AutoDiffXd c = search_data.c;
    const double momentum_cost =
        (a * alpha * alpha / 2.0 + b * alpha + c).value();
    EXPECT_NEAR(momentum_cost, momentum_cost_expected,
                8 * kEpsilon * abs(momentum_cost_expected));

    AutoDiffXd dcost, d2cost;
    const AutoDiffXd cost =
        model.CalcCostAlongLine(alpha, data, search_data, &dcost, &d2cost);
    EXPECT_NEAR(cost.value(), cost_expected, 8 * kEpsilon * abs(cost_expected));
    EXPECT_NEAR(dcost.value(), dcost_expected,
                8 * kEpsilon * abs(dcost_expected));
    EXPECT_NEAR(d2cost.value(), d2cost_expected,
                8 * kEpsilon * abs(d2cost_expected));
  }
}

/* Checks that updating the time step produces the same result as creating a new
model from scratch. */
GTEST_TEST(IcfModel, UpdateTimeStep) {
  IcfModel<double> model_original;
  // TODO(vincekurtz): run this test with constraints once they land.
  MakeUnconstrainedModel(&model_original, false, 0.02);
  model_original.SetSparsityPattern();
  EXPECT_EQ(model_original.num_cliques(), 3);
  EXPECT_EQ(model_original.num_velocities(), 18);
  EXPECT_EQ(model_original.time_step(), 0.02);

  const double new_time_step = 0.003;

  // Create a second model from scratch with the new time step.
  IcfModel<double> model_new;
  MakeUnconstrainedModel(&model_new, false, new_time_step);
  model_new.SetSparsityPattern();
  EXPECT_EQ(model_new.num_cliques(), 3);
  EXPECT_EQ(model_new.num_velocities(), 18);
  EXPECT_EQ(model_new.time_step(), new_time_step);

  // Now update the time step of the original model.
  EXPECT_NE(model_original.time_step(), new_time_step);
  model_original.UpdateTimeStep(new_time_step);
  EXPECT_EQ(model_original.time_step(), new_time_step);

  // Allocate data.
  IcfData<double> data_original, data_new;
  model_original.ResizeData(&data_original);
  model_new.ResizeData(&data_new);

  // Compute data for an arbitrary velocity.
  const int nv = model_original.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model_original.CalcData(v, &data_original);
  model_new.CalcData(v, &data_new);

  // Cost and gradient should be the same.
  EXPECT_NEAR(data_original.cost(), data_new.cost(), 8 * kEpsilon);
  EXPECT_TRUE(CompareMatrices(data_original.gradient(), data_new.gradient(),
                              8 * kEpsilon, MatrixCompareType::relative));

  // Hessians should be the same.
  auto hessian_original = model_original.MakeHessian(data_original);
  auto hessian_new = model_new.MakeHessian(data_new);
  MatrixXd H_original = hessian_original->MakeDenseMatrix();
  MatrixXd H_new = hessian_new->MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(H_original, H_new, 8 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Verifies that params() and ReleaseParams() use the same address. */
GTEST_TEST(IcfModel, ParamsAccessors) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);

  const IcfParameters<double>* params1 = &model.params();
  std::unique_ptr<IcfParameters<double>> params2 = model.ReleaseParameters();
  EXPECT_EQ(params1, params2.get());
}

// Note: the functions below are separated out to prepare for landing the
// unconstrained model first.

/* Adds some dummy contact constraints to the model. */
template <typename T>
void AddContactConstraints(IcfModel<T>* model) {
  // Add contact patches.
  const T dissipation = 50.0;
  const T stiffness = 1.0e6;
  const T friction = 0.5;

  PatchConstraintsPool<T>& patches = model->patch_constraints_pool();
  // Resize to hold three patches with one contact pair each.
  // N.B. num_pairs_per_patch is allocated on the heap.
  const std::vector<int> num_pairs_per_patch = {1, 1, 1};
  patches.Resize(num_pairs_per_patch);

  // first patch
  {
    const Vector3<T> p_AB_W(0.1, 0.0, 0.0);
    patches.SetPatch(0 /* patch index */, 1 /* body A */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.SetPair(0 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Single clique patch.
  {
    const Vector3<T> p_AB_W(0.0, 0.05, 0.0);  // A = World.
    patches.SetPatch(1 /* patch index */, 0 /* World */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(0.0, 1.0, 0.0);
    const Vector3<T> p_BC_W(0.0, -0.05, 0.0);
    const T fn0 = 1.5;
    patches.SetPair(1 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // Third patch
  {
    const Vector3<T> p_AB_W(-0.1, 0.0, 0.0);
    patches.SetPatch(2 /* patch index */, 3 /* World */, 2 /* body B */,
                     dissipation, friction, friction, p_AB_W);

    const Vector3<T> nhat_AB_W(-1.0, 0.0, 0.0);
    const Vector3<T> p_BC_W = -0.5 * p_AB_W;
    const T fn0 = 1.5;
    patches.SetPair(2 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);
  }

  // N.B. SetSparsityPattern performs several heap allocations.
  model->SetSparsityPattern();
}

/* Create a more complete model with basic contact constraints. */
template <typename T>
void MakeModel(IcfModel<T>* model, bool single_clique = false) {
  MakeUnconstrainedModel(model, single_clique);
  AddContactConstraints(model);
}

GTEST_TEST(IcfModel, GainConstraint) {
  IcfModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  IcfData<AutoDiffXd> data;
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
  gain_constraints.Set(0, 0, K0, u0, e0);

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
  gain_constraints.Set(1, 2, K2, u2, e2);

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

  const GainConstraintsDataPool<AutoDiffXd>& gains_data =
      data.gain_constraints_data();
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

  const VectorXd cost_gradient = gains_data.cost().derivatives();

  const VectorX<AutoDiffXd> gamma0 = gains_data.gamma(0);
  EXPECT_EQ(gamma0.size(), nv0);
  const VectorXd gamma0_value = math::ExtractValue(gamma0);
  EXPECT_TRUE(CompareMatrices(gamma0_value, gamma0_expected, kEpsilon,
                              MatrixCompareType::relative));

  // Gradient of the cost.
  const VectorXd minus_cost0_gradient = -cost_gradient.segment<6>(0);
  EXPECT_TRUE(CompareMatrices(gamma0_value, minus_cost0_gradient, kEpsilon,
                              MatrixCompareType::relative));

  // Verify gradients for gain on clique 2.
  const VectorX<AutoDiffXd> gamma2 = gains_data.gamma(1 /* constraint index */);
  EXPECT_EQ(gamma2.size(), nv2);
  const VectorXd gamma2_value = math::ExtractValue(gamma2);
  const VectorXd minus_cost2_gradient =
      -cost_gradient.segment<6>(12 /* first velocity index */);
  EXPECT_TRUE(CompareMatrices(gamma2_value, minus_cost2_gradient, kEpsilon,
                              MatrixCompareType::relative));

  // Verify accumulated total cost and gradients.
  const VectorXd total_cost_derivatives = data.cost().derivatives();
  const VectorXd total_gradient_value = math::ExtractValue(data.gradient());

  EXPECT_TRUE(CompareMatrices(total_gradient_value, total_cost_derivatives,
                              2 * kEpsilon, MatrixCompareType::relative));

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives,
                              10 * kEpsilon, MatrixCompareType::relative));
}

GTEST_TEST(IcfModel, LimitConstraint) {
  IcfModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());
  EXPECT_EQ(data.num_limits(), 0);

  // At this point there should be no gain constraints.
  EXPECT_EQ(model.num_limit_constraints(), 0);

  // Add limit constraints.
  auto& limits = model.limit_constraints_pool();

  // Cliques 0 and 2 will have limits. We'll resize the constraint pool
  // accordingly before adding the limit constraints.
  std::vector<int> limited_clique_sizes = {model.clique_size(0),
                                           model.clique_size(2)};
  std::vector<int> constraint_to_clique = {0, 2};
  limits.Resize(limited_clique_sizes, constraint_to_clique);

  EXPECT_EQ(model.num_limit_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 5);

  const int nv = model.num_velocities();
  VectorX<AutoDiffXd> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  // Limits on clique 0.
  limits.Set(0 /* constraint */, 0 /* clique */, 4 /* dof */, q0(2), -1.5,
             -1.0);  // Above.
  limits.Set(0 /* constraint */, 0 /* clique */, 3 /* dof */, q0(3), -1.0,
             1.0);  // Within.

  EXPECT_EQ(model.num_limit_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 5);

  // Limits on clique 2.
  limits.Set(1 /* constraint */, 2 /* clique */, 0 /* dof */, q0(12), 0.5,
             1.5);  // Below.
  limits.Set(1 /* constraint */, 2 /* clique */, 2 /* dof */, q0(14), -1.0,
             0.5);  // Above.
  limits.Set(1 /* constraint */, 2 /* clique */, 5 /* dof */, q0(17), 0.5,
             1.5);  // Within.

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

  const LimitConstraintsDataPool<AutoDiffXd>& limits_data =
      data.limit_constraints_data();
  EXPECT_EQ(limits_data.num_constraints(), 2);

  const VectorXd cost_gradient = limits_data.cost().derivatives();

  // Impulses on constraint 0, clique 0.
  const VectorX<AutoDiffXd> gamma_lower0 = limits_data.gamma_lower(0);
  const VectorX<AutoDiffXd> gamma_upper0 = limits_data.gamma_upper(0);
  // tau = Jᵀ⋅γ, and J = [1; -1] for limit constraints.
  const VectorX<AutoDiffXd> tau0 = gamma_lower0 - gamma_upper0;

  // Impulses on constraint 1, clique 2.
  const VectorX<AutoDiffXd> gamma_lower1 = limits_data.gamma_lower(1);
  const VectorX<AutoDiffXd> gamma_upper1 = limits_data.gamma_upper(1);
  // tau = Jᵀ⋅γ, and J = [1; -1] for limit constraints.
  const VectorX<AutoDiffXd> tau1 = gamma_lower1 - gamma_upper1;

  // Assemble all clique contributions into tau by hand.
  VectorX<AutoDiffXd> tau = VectorX<AutoDiffXd>::Zero(nv);
  tau.segment<6>(0) = tau0;
  tau.segment<6>(12) = tau1;

  const VectorXd tau_value = math::ExtractValue(tau);
  EXPECT_TRUE(CompareMatrices(cost_gradient, -tau_value, kEpsilon,
                              MatrixCompareType::relative));

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives,
                              10 * kEpsilon, MatrixCompareType::relative));
}

GTEST_TEST(IcfModel, CouplerConstraint) {
  IcfModel<AutoDiffXd> model;
  MakeModel(&model, false /* multiple cliques */);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), model.num_constraints());
  EXPECT_EQ(data.num_limits(), 0);

  // At this point there should be no coupler constraints.
  EXPECT_EQ(model.num_coupler_constraints(), 0);

  // Add coupler constraints.
  auto& couplers = model.coupler_constraints_pool();
  couplers.Resize(1 /* one constraint */);

  const int nv = model.num_velocities();
  VectorX<AutoDiffXd> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);

  // Coupler on clique 1.
  const auto q0_c1 = model.clique_segment(1, q0);
  const double rho1 = 2.5;
  const double offset1 = 0.1;
  couplers.Set(0 /* constraint index */, 1 /* clique */, 1 /* i */, 3 /* j */,
               q0_c1(1), q0_c1(3), rho1, offset1);
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

  // Compute regularization.
  const double beta =
      0.1;  // Keep in sync with hard-coded value in the implementation.
  const double m1 = 2.3;                     // "mass" for clique 1.
  const double w1 = (1 + rho1 * rho1) / m1;  // Delassus for clique 1.
  const double m1_eff = 1.0 / w1;            // "Effective" mass for clique 1.
  const double omega_near_rigid =
      2 * M_PI / (beta * dt);  // period_nr = beta * dt, by definition.
  const double k1 = m1_eff * omega_near_rigid * omega_near_rigid;
  const double tau1 = beta / M_PI * dt;

  const CouplerConstraintsDataPool<AutoDiffXd>& couplers_data =
      data.coupler_constraints_data();
  EXPECT_EQ(couplers_data.num_constraints(), 1);

  const VectorXd cost_gradient = couplers_data.cost().derivatives();

  // Expected impulse.
  const AutoDiffXd g0 = q_c1(1) - rho1 * q_c1(3) - offset1;
  const AutoDiffXd gdot0 = v_c1(1) - rho1 * v_c1(3);
  const AutoDiffXd gamma0 = -dt * k1 * (g0 + tau1 * gdot0);
  VectorXd tau_expected = VectorXd::Zero(nv);
  tau_expected(6 + 1) = gamma0.value();
  tau_expected(6 + 3) = -rho1 * gamma0.value();
  EXPECT_TRUE(CompareMatrices(-cost_gradient, tau_expected, 10 * kEpsilon,
                              MatrixCompareType::relative));

  const double gamma = couplers_data.gamma(0).value();
  EXPECT_NEAR(gamma, gamma0.value(), std::abs(gamma) * kEpsilon);

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives,
                              10 * kEpsilon, MatrixCompareType::relative));

  // CalcCostAlongLine()
  // Allocate search direction.
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

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
