#include "drake/multibody/optimization/centroidal_momentum_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
template <typename T>
Vector6<T> EvalConstraint(
    const MultibodyPlant<T>* plant,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    systems::Context<T>* plant_context, const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v,
    const Eigen::Ref<const Vector6<T>>& h_WC_val) {
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
  return h_WC.get_coeffs() - h_WC_val;
}

template <typename T>
void CentroidalMomentumConstraintTester(
    const MultibodyPlant<T>* plant,
    const MultibodyPlant<AutoDiffXd>* plant_autodiff,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    systems::Context<T>* plant_context,
    systems::Context<AutoDiffXd>* plant_context_autodiff,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Vector6<double>>& h_WC) {
  CentroidalMomentumConstraint dut(plant, model_instances, plant_context);
  EXPECT_EQ(dut.num_vars(),
            plant->num_positions() + plant->num_velocities() + 6);
  EXPECT_EQ(dut.num_constraints(), 6);
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), Vector6<double>::Zero()));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), Vector6<double>::Zero()));
  Eigen::VectorXd x;
  dut.ComposeVariable(q, v, h_WC, &x);
  Eigen::VectorXd y;
  // First check Eval with a vector of doubles.
  dut.Eval(x, &y);
  if constexpr (std::is_same_v<T, double>) {
    const Vector6<double> y_expected = EvalConstraint<double>(
        plant, model_instances, plant_context, q, v, h_WC);
    EXPECT_TRUE(CompareMatrices(y, y_expected));
  } else {
    const Vector6<AutoDiffXd> y_expected = EvalConstraint<AutoDiffXd>(
        plant, model_instances, plant_context, q.cast<AutoDiffXd>(),
        v.cast<AutoDiffXd>(), h_WC.cast<AutoDiffXd>());
    EXPECT_TRUE(CompareMatrices(y, math::autoDiffToValueMatrix(y_expected)));
  }

  // Now check Eval with AutoDiffXd, where the gradient in x is identity.
  AutoDiffVecXd x_autodiff = math::initializeAutoDiff(x);
  AutoDiffVecXd y_autodiff;
  dut.Eval(x_autodiff, &y_autodiff);
  Vector6<AutoDiffXd> y_autodiff_expected = EvalConstraint<AutoDiffXd>(
      plant_autodiff, model_instances, plant_context_autodiff,
      x_autodiff.head(plant->num_positions()),
      x_autodiff.segment(plant->num_positions(), plant->num_velocities()),
      x_autodiff.tail<6>());
  const double tol{1E-14};
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

  // Now check Eval with AutoDiffXd, where the gradient in x is not identity.
  Eigen::MatrixXd x_grad(x.rows(), 2);
  for (int i = 0; i < x_grad.rows(); ++i) {
    x_grad(i, 0) = 0.5 * i - 1;
    x_grad(i, 1) = 0.9 * i + 0.5;
  }
  x_autodiff = math::initializeAutoDiffGivenGradientMatrix(x, x_grad);
  dut.Eval(x_autodiff, &y_autodiff);
  y_autodiff_expected = EvalConstraint<AutoDiffXd>(
      plant_autodiff, model_instances, plant_context_autodiff,
      x_autodiff.head(plant->num_positions()),
      x_autodiff.segment(plant->num_positions(), plant->num_velocities()),
      x_autodiff.tail<6>());
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
  CentroidalMomentumConstraintTester(plant_autodiff.get(), plant_autodiff.get(),
                                     std::nullopt /* model_instances */,
                                     plant_context_autodiff.get(),
                                     plant_context_autodiff.get(), q, v, h_WC);

  // Now test with non-empty model_instances
  std::vector<ModelInstanceIndex> model_instances{
      {ModelInstanceIndex{plant->GetModelInstanceByName("iiwa0")}}};
  CentroidalMomentumConstraintTester(
      plant_autodiff.get(), plant_autodiff.get(), model_instances,
      plant_context_autodiff.get(), plant_context_autodiff.get(), q, v, h_WC);
}
}  // namespace multibody
}  // namespace drake
