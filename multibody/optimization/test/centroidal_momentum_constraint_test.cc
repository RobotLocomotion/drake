#include "drake/multibody/optimization/centroidal_momentum_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
template <typename T>
VectorX<T> EvalConstraint(
    const MultibodyPlant<T>* plant,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    systems::Context<T>* plant_context, bool angular_only,
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v,
    const Eigen::Ref<const VectorX<T>>& momentum_val) {
  Vector3<T> p_WC;
  SpatialMomentum<T> h_WC;
  if (model_instances.has_value()) {
    p_WC = plant->CalcCenterOfMassPositionInWorld(*plant_context,
                                                  model_instances.value());
    h_WC = plant->CalcSpatialMomentumInWorldAboutPoint(
        *plant_context, model_instances.value(), p_WC);
  } else {
    p_WC = plant->CalcCenterOfMassPositionInWorld(*plant_context);
    h_WC = plant->CalcSpatialMomentumInWorldAboutPoint(*plant_context, p_WC);
  }
  if (angular_only) {
    return h_WC.rotational() - momentum_val;
  } else {
    return h_WC.get_coeffs() - momentum_val;
  }
}

template <typename T>
void CentroidalMomentumConstraintTester(
    const MultibodyPlant<T>* plant,
    const MultibodyPlant<AutoDiffXd>* plant_autodiff,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    systems::Context<T>* plant_context,
    systems::Context<AutoDiffXd>* plant_context_autodiff, bool angular_only,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& momentum_val) {
  CentroidalMomentumConstraint dut(plant, model_instances, plant_context,
                                   angular_only);
  const int momentum_dim = angular_only ? 3 : 6;
  DRAKE_DEMAND(momentum_val.rows() == momentum_dim);
  EXPECT_EQ(dut.num_vars(),
            plant->num_positions() + plant->num_velocities() + momentum_dim);
  EXPECT_EQ(dut.num_constraints(), momentum_dim);
  EXPECT_TRUE(
      CompareMatrices(dut.lower_bound(), VectorX<double>::Zero(momentum_dim)));
  EXPECT_TRUE(
      CompareMatrices(dut.upper_bound(), VectorX<double>::Zero(momentum_dim)));
  Eigen::VectorXd x;
  dut.ComposeVariable(q, v, momentum_val, &x);
  Eigen::VectorXd y;
  // First check Eval with a vector of doubles.
  dut.Eval(x, &y);
  if constexpr (std::is_same_v<T, double>) {
    const VectorX<double> y_expected =
        EvalConstraint<double>(plant, model_instances, plant_context,
                               angular_only, q, v, momentum_val);
    EXPECT_TRUE(CompareMatrices(y, y_expected));
  } else {
    const VectorX<AutoDiffXd> y_expected = EvalConstraint<AutoDiffXd>(
        plant, model_instances, plant_context, angular_only,
        q.cast<AutoDiffXd>(), v.cast<AutoDiffXd>(),
        momentum_val.cast<AutoDiffXd>());
    EXPECT_TRUE(CompareMatrices(y, math::ExtractValue(y_expected)));
  }

  // Now check Eval with AutoDiffXd, where the gradient in x is identity.
  AutoDiffVecXd x_autodiff = math::InitializeAutoDiff(x);
  AutoDiffVecXd y_autodiff;
  dut.Eval(x_autodiff, &y_autodiff);
  VectorX<AutoDiffXd> y_autodiff_expected = EvalConstraint<AutoDiffXd>(
      plant_autodiff, model_instances, plant_context_autodiff, angular_only,
      x_autodiff.head(plant->num_positions()),
      x_autodiff.segment(plant->num_positions(), plant->num_velocities()),
      x_autodiff.tail(momentum_dim));
  const double tol{1E-14};
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

  // Now check Eval with AutoDiffXd, where the gradient in x is not identity.
  Eigen::MatrixXd x_grad(x.rows(), 2);
  for (int i = 0; i < x_grad.rows(); ++i) {
    x_grad(i, 0) = 0.5 * i - 1;
    x_grad(i, 1) = 0.9 * i + 0.5;
  }
  x_autodiff = math::InitializeAutoDiff(x, x_grad);
  dut.Eval(x_autodiff, &y_autodiff);
  y_autodiff_expected = EvalConstraint<AutoDiffXd>(
      plant_autodiff, model_instances, plant_context_autodiff, angular_only,
      x_autodiff.head(plant->num_positions()),
      x_autodiff.segment(plant->num_positions(), plant->num_velocities()),
      x_autodiff.tail(momentum_dim));
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);
}

GTEST_TEST(CentroidalMomentumConstraintTest, TestDualIiwas) {
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf");

  auto plant =
      ConstructIiwaPlant(iiwa_path, 0.1 /* time step */, 2 /* num_iiwa */);
  auto plant_context = plant->CreateDefaultContext();
  auto plant_autodiff = systems::System<double>::ToAutoDiffXd(*plant);
  auto plant_context_autodiff = plant_autodiff->CreateDefaultContext();
  // Test constraint constructed with MBP<AutoDiffXd>
  Eigen::VectorXd q(14);
  q << 0.1, 0.2, 0.4, 0.3, 0.5, -0.2, 0.6, 0.2, 0.5, 0.1, -1.2, 0.4, 0.3, 0.7;
  Eigen::VectorXd v(14);
  v << 0.2, -0.4, 0.5, 1.2, 0.6, 0.1, 0.8, -0.5, -2.1, 0.5, 0.8, 0.4, 0.9, -1.2;
  Vector6<double> h_WC;
  h_WC << 0.5, -1.2, 2.1, 3.2, 0.5, 2.3;

  // First test with empty model_instances.
  for (bool angular_only : {true, false}) {
    Eigen::VectorXd momentum_val;
    if (angular_only) {
      momentum_val = h_WC.head<3>();
    } else {
      momentum_val = h_WC;
    }
    CentroidalMomentumConstraintTester(
        plant_autodiff.get(), plant_autodiff.get(),
        std::nullopt /* model_instances */, plant_context_autodiff.get(),
        plant_context_autodiff.get(), angular_only, q, v, momentum_val);
    // Now test with non-empty model_instances
    std::vector<ModelInstanceIndex> model_instances{
        {ModelInstanceIndex{plant->GetModelInstanceByName("iiwa0")}}};
    CentroidalMomentumConstraintTester(
        plant_autodiff.get(), plant_autodiff.get(), model_instances,
        plant_context_autodiff.get(), plant_context_autodiff.get(),
        angular_only, q, v, momentum_val);
  }
}
}  // namespace multibody
}  // namespace drake
