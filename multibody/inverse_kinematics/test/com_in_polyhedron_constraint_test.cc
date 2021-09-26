#include "drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalComInPolyhedronConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyPlant<AutoDiffXd>& plant,
    const std::vector<ModelInstanceIndex>& model_instances,
    const Frame<AutoDiffXd>& expressed_frame,
    const Eigen::Ref<const Eigen::MatrixX3d>& A) {
  // Compute the CoM position p_EC by looping through each body.
  AutoDiffXd total_mass{0};
  Vector3<AutoDiffXd> aggregated_mass_times_com;
  aggregated_mass_times_com.setZero();
  for (auto model_instance : model_instances) {
    for (auto body_index : plant.GetBodyIndices(model_instance)) {
      if (body_index != 0) {
        const math::RigidTransform<AutoDiffXd> X_EB =
            plant.CalcRelativeTransform(
                context, expressed_frame,
                plant.get_body(body_index).body_frame());
        const Vector3<AutoDiffXd> p_BBcm =
            plant.get_body(body_index).CalcCenterOfMassInBodyFrame(context);
        const Vector3<AutoDiffXd> p_EBcm = X_EB * p_BBcm;
        const AutoDiffXd body_mass =
            plant.get_body(body_index).get_mass(context);
        total_mass += body_mass;
        aggregated_mass_times_com += body_mass * p_EBcm;
      }
    }
  }
  const Vector3<AutoDiffXd> p_EC = aggregated_mass_times_com / total_mass;
  return A * p_EC;
}

template <typename T>
void TestComInPolyhedronConstraint(
    const MultibodyPlant<T>* const plant,
    const MultibodyPlant<AutoDiffXd>* const plant_autodiff,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    const FrameIndex& expressed_frame_index, systems::Context<T>* plant_context,
    systems::Context<AutoDiffXd>* plant_context_autodiff,
    const Eigen::Ref<const Eigen::MatrixX3d>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const Eigen::VectorXd>& q_val) {
  ComInPolyhedronConstraint constraint(plant, model_instances,
                                       plant->get_frame(expressed_frame_index),
                                       A, lb, ub, plant_context);
  EXPECT_EQ(constraint.num_vars(), plant->num_positions());
  EXPECT_EQ(constraint.num_constraints(), A.rows());
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), lb));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), ub));

  // Now check if Eval function computes the right result.

  Eigen::VectorXd y;
  constraint.Eval(q_val, &y);

  AutoDiffVecXd q_autodiff = math::InitializeAutoDiff(q_val);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  // Make sure the double version and the autodiff version compute the same
  // value.
  const double tol{1E-14};
  EXPECT_TRUE(CompareMatrices(y, math::ExtractValue(y_autodiff), tol));
  // Now compare the autodiff computed from Eval versus the autodiff computed
  // from EvalComInPolyhedronConstraintAutoDiff.
  plant_autodiff->GetMutablePositions(plant_context_autodiff) = q_autodiff;
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

  VectorX<AutoDiffXd> y_autodiff_expected =
      EvalComInPolyhedronConstraintAutoDiff(
          *plant_context_autodiff, *plant_autodiff, model_instances_val,
          plant_autodiff->get_frame(expressed_frame_index), A);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

  // Test with non-identity gradient for x_autodiff.
  Eigen::MatrixXd q_grad(constraint.num_vars(), 2);
  for (int i = 0; i < constraint.num_vars(); ++i) {
    q_grad(i, 0) = 0.1 * i + 1;
    q_grad(i, 1) = -0.2 * i - 0.5;
  }
  q_autodiff = math::InitializeAutoDiff(q_val, q_grad);
  plant_autodiff->GetMutablePositions(plant_context_autodiff) = q_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalComInPolyhedronConstraintAutoDiff(
      *plant_context_autodiff, *plant_autodiff, model_instances_val,
      plant_autodiff->get_frame(expressed_frame_index), A);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);
}

TEST_F(IiwaKinematicConstraintTest,
       ComInPolyhedronConstraintEmptyModelInstance) {
  const Frame<double>& link2_frame = plant_->GetFrameByName("iiwa_link_2");
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.4, 0.3, 0.5, -0.2, 0.6;
  Eigen::Matrix<double, 4, 3> A;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = 0.1 * i + 0.5 * j;
    }
  }
  const Eigen::Vector4d lb(0.1, 0.2, -0.4, -1.);
  const Eigen::Vector4d ub(0.5, 0.9, 1.5, 1.3);

  // Test constraint constructed with MBP<double>
  TestComInPolyhedronConstraint(plant_, plant_autodiff_.get(), std::nullopt,
                                link2_frame.index(), plant_context_,
                                plant_context_autodiff_.get(), A, lb, ub, q);
  // Test constraint constructed with MBP<AutoDiffXd>
  TestComInPolyhedronConstraint(plant_autodiff_.get(), plant_autodiff_.get(),
                                std::nullopt, link2_frame.index(),
                                plant_context_autodiff_.get(),
                                plant_context_autodiff_.get(), A, lb, ub, q);
}

GTEST_TEST(DualIiwaTest, ComInPolyhedronConstraintModelInstance) {
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
  Eigen::Matrix<double, 4, 3> A;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = 0.1 * i + 0.5 * j;
    }
  }
  const Eigen::Vector4d lb(0.1, 0.2, -0.4, -1.);
  const Eigen::Vector4d ub(0.5, 0.9, 1.5, 1.3);
  std::vector<ModelInstanceIndex> model_instances{
      {plant->GetModelInstanceByName("iiwa0")}};
  TestComInPolyhedronConstraint(
      plant_autodiff.get(), plant_autodiff.get(), model_instances,
      plant_autodiff->world_frame().index(), plant_context_autodiff.get(),
      plant_context_autodiff.get(), A, lb, ub, q);
  // TODO(hongkai.dai): Add test with non-empty model_instances and constraint
  // constructed from MBP<double> when #14916 is resolved.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ComInPolyhedronConstraint(plant.get(), model_instances,
                                plant->world_frame(), A, lb, ub,
                                plant_context.get()),
      std::invalid_argument, ".* model_instances has to be .*");
  // Test model_instances being an empty vector.
  model_instances.clear();
  DRAKE_EXPECT_THROWS_MESSAGE(
      ComInPolyhedronConstraint(plant_autodiff.get(), model_instances,
                                plant_autodiff->world_frame(), A, lb, ub,
                                plant_context_autodiff.get()),
      std::invalid_argument, ".* model_instances is an empty vector.");
}
}  // namespace
}  // namespace multibody
}  // namespace drake
