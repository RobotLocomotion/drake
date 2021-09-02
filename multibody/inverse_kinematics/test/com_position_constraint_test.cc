#include "drake/multibody/inverse_kinematics/com_position_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalComPositionConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyPlant<AutoDiffXd>& plant,
    const std::vector<ModelInstanceIndex>& model_instances,
    const Frame<AutoDiffXd>& expressed_frame,
    const Eigen::Ref<const Vector3<AutoDiffXd>>& p_EC) {
  const Vector3<AutoDiffXd> p_WC =
      plant.CalcCenterOfMassPositionInWorld(context, model_instances);
  const math::RigidTransform<AutoDiffXd> X_EW = plant.CalcRelativeTransform(
      context, expressed_frame, plant.world_frame());
  return X_EW * p_WC - p_EC;
}

template <typename T>
void TestComPositionConstraint(
    const MultibodyPlant<T>* const plant,
    const MultibodyPlant<AutoDiffXd>* const plant_autodiff,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    const FrameIndex& expressed_frame_index, systems::Context<T>* plant_context,
    systems::Context<AutoDiffXd>* plant_context_autodiff,
    const Eigen::Ref<const Eigen::VectorXd>& q_val,
    const Eigen::Ref<const Eigen::VectorXd>& p_EC_val) {
  ComPositionConstraint constraint(plant, model_instances,
                                   plant->get_frame(expressed_frame_index),
                                   plant_context);
  EXPECT_EQ(constraint.num_vars(), plant->num_positions() + 3);
  EXPECT_EQ(constraint.num_constraints(), 3);
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector3d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(), Eigen::Vector3d::Zero()));

  // Now check if Eval function computes the right result.
  Eigen::VectorXd x;
  constraint.ComposeVariable<double>(q_val, p_EC_val, &x);

  Eigen::VectorXd y;
  constraint.Eval(x, &y);

  plant->SetPositions(plant_context, q_val);
  Vector3<T> p_WC;
  if (model_instances.has_value()) {
    p_WC = plant->CalcCenterOfMassPositionInWorld(*plant_context,
                                                  model_instances.value());
  } else {
    p_WC = plant->CalcCenterOfMassPositionInWorld(*plant_context);
  }
  const Vector3<T> y_expected =
      plant->CalcRelativeTransform(*plant_context,
                                   plant->get_frame(expressed_frame_index),
                                   plant->world_frame()) *
          p_WC -
      p_EC_val;
  const double tol{1E-12};
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_TRUE(CompareMatrices(y, y_expected, tol));
  } else {
    EXPECT_TRUE(CompareMatrices(y, math::ExtractValue(y_expected), tol));
  }

  AutoDiffVecXd x_autodiff = math::InitializeAutoDiff(x);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);
  plant_autodiff->GetMutablePositions(plant_context_autodiff) =
      x_autodiff.head(plant->num_positions());
  std::vector<ModelInstanceIndex> model_instances_val;
  if (model_instances.has_value()) {
    model_instances_val = model_instances.value();
  } else {
    for (ModelInstanceIndex model_instance_index{1};
         model_instance_index < plant->num_model_instances();
         ++model_instance_index) {
      model_instances_val.push_back(model_instance_index);
    }
  }
  Vector3<AutoDiffXd> y_autodiff_expected = EvalComPositionConstraintAutoDiff(
      *plant_context_autodiff, *plant_autodiff, model_instances_val,
      plant_autodiff->get_frame(expressed_frame_index), x_autodiff.tail<3>());
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

  // Test with non-identity gradient for x_autodiff.
  Eigen::MatrixXd x_grad(constraint.num_vars(), 2);
  for (int i = 0; i < constraint.num_vars(); ++i) {
    x_grad(i, 0) = 0.1 * i + 1;
    x_grad(i, 1) = -0.2 * i - 0.5;
  }
  x_autodiff = math::InitializeAutoDiff(x, x_grad);
  plant_autodiff->GetMutablePositions(plant_context_autodiff) =
      x_autodiff.head(plant->num_positions());
  constraint.Eval(x_autodiff, &y_autodiff);
  y_autodiff_expected = EvalComPositionConstraintAutoDiff(
      *plant_context_autodiff, *plant_autodiff, model_instances_val,
      plant_autodiff->get_frame(expressed_frame_index), x_autodiff.tail<3>());
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);
}

TEST_F(IiwaKinematicConstraintTest,
       ComPositionConstraint_empty_model_instance) {
  const Frame<double>& link2_frame = plant_->GetFrameByName("iiwa_link_2");
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.4, 0.3, 0.5, -0.2, 0.6;
  Eigen::Vector3d p_EC(0.9, 1.2, 0.8);
  // Test constraint constructed with MBP<double>
  TestComPositionConstraint(plant_, plant_autodiff_.get(), std::nullopt,
                            link2_frame.index(), plant_context_,
                            plant_context_autodiff_.get(), q, p_EC);
  // Test constraint constructed with MBP<AutoDiffXd>
  TestComPositionConstraint(plant_autodiff_.get(), plant_autodiff_.get(),
                            std::nullopt, link2_frame.index(),
                            plant_context_autodiff_.get(),
                            plant_context_autodiff_.get(), q, p_EC);
}

GTEST_TEST(DualIiwaTest, ComPositionConstraint_model_instance) {
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
  Eigen::Vector3d p_EC(0.9, 1.2, 0.8);
  std::vector<ModelInstanceIndex> model_instances{
      {plant->GetModelInstanceByName("iiwa0")}};
  TestComPositionConstraint(
      plant_autodiff.get(), plant_autodiff.get(), model_instances,
      plant_autodiff->world_frame().index(), plant_context_autodiff.get(),
      plant_context_autodiff.get(), q, p_EC);
  // TODO(hongkai.dai): Add test with non-empty model_instances and constraint
  // constructed from MBP<double> when #14916 is resolved.
}
}  // namespace
}  // namespace multibody
}  // namespace drake
