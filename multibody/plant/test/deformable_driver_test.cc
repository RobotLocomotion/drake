#include "drake/multibody/plant/deformable_driver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::VolumeMesh;
using drake::math::RigidTransformd;
using drake::multibody::fem::FemState;
using drake::systems::Context;
using std::make_unique;
using std::move;

namespace drake {
namespace multibody {
namespace internal {

class DeformableDriverTest : public ::testing::Test {
 protected:
  static constexpr double kDt = 0.01;

  void SetUp() override {
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    constexpr double kRezHint = 0.5;
    body_id_ = RegisterSphere(deformable_model.get(), kRezHint);
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));
    // N.B. Currently the manager only supports SAP.
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);
    plant_->Finalize();
    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(move(contact_manager));
    driver_ = CompliantContactManagerTester::deformable_driver(*manager_);
    context_ = plant_->CreateDefaultContext();
  }

  /* Forwarding calls to private member functions in DeformableDriver with the
   same name.
   @{ */
  const FemState<double>& EvalFemState(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return driver_->EvalFemState(context, index);
  }

  const FemState<double>& EvalFreeMotionFemState(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return driver_->EvalFreeMotionFemState(context, index);
  }

  const FemState<double>& EvalNextFemState(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return driver_->EvalNextFemState(context, index);
  }
  /* @} */

  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_{nullptr};
  DeformableBodyId body_id_;

 private:
  DeformableBodyId RegisterSphere(DeformableModel<double>* model,
                                  double resolution_hint) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(1), "sphere");
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(1e6);
    DeformableBodyId body_id = model->RegisterDeformableBody(
        move(geometry), body_config, resolution_hint);
    return body_id;
  }
};

namespace {

/* Verifies that a DeformableDriver has been successfully created. */
TEST_F(DeformableDriverTest, Constructor) { ASSERT_NE(driver_, nullptr); }

TEST_F(DeformableDriverTest, ScalarConversion) {
  EXPECT_FALSE(driver_->is_cloneable_to_double());
  EXPECT_FALSE(driver_->is_cloneable_to_autodiff());
  EXPECT_FALSE(driver_->is_cloneable_to_symbolic());
}

TEST_F(DeformableDriverTest, FemState) {
  const int num_dofs = model_->GetFemModel(body_id_).num_dofs();
  const auto q = 2.0 * VectorX<double>::Ones(num_dofs);
  const auto v = VectorX<double>::Ones(num_dofs);
  const auto a = VectorX<double>::Zero(num_dofs);
  VectorX<double> state_value(3 * num_dofs);
  state_value << q, v, a;
  const systems::DiscreteStateIndex state_index =
      model_->GetDiscreteStateIndex(body_id_);
  context_->SetDiscreteState(state_index, state_value);
  const FemState<double>& fem_state =
      EvalFemState(*context_, DeformableBodyIndex(0));
  EXPECT_EQ(fem_state.GetPositions(), q);
  EXPECT_EQ(fem_state.GetVelocities(), v);
  EXPECT_EQ(fem_state.GetAccelerations(), a);
}

TEST_F(DeformableDriverTest, FreeMotionFemState) {
  const VectorX<double> q = model_->GetReferencePositions(body_id_);
  const int num_dofs = q.size();
  const VectorX<double> v = VectorX<double>::Zero(num_dofs);
  const VectorX<double> a = VectorX<double>::Zero(num_dofs);
  const FemState<double>& free_motion_fem_state =
      EvalFreeMotionFemState(*context_, DeformableBodyIndex(0));
  const Vector3<double> kGravity(0, 0, -9.81);
  VectorX<double> next_a = a;
  for (int dof = 0; dof < num_dofs; dof += 3) {
    next_a.segment<3>(dof) += kGravity;
  }
  // We use the clear-box knowledge that we use midpoint rule for time
  // integration.
  const VectorX<double> next_v = v + next_a * kDt;
  const VectorX<double> next_q = q + 0.5 * (v + next_v) * kDt;
  // Tolerance has unit of velocity.
  const double kTol = 1e-4;
  EXPECT_TRUE(CompareMatrices(free_motion_fem_state.GetPositions(), next_q,
                              kTol * kDt));
  EXPECT_TRUE(
      CompareMatrices(free_motion_fem_state.GetVelocities(), next_v, kTol));
  EXPECT_TRUE(CompareMatrices(free_motion_fem_state.GetAccelerations(), next_a,
                              kTol / kDt));
}

TEST_F(DeformableDriverTest, NextFemState) {
  const FemState<double>& free_motion_fem_state =
      EvalFreeMotionFemState(*context_, DeformableBodyIndex(0));
  const FemState<double>& next_fem_state =
      EvalNextFemState(*context_, DeformableBodyIndex(0));
  /* The next state should be the same as the free motion state in the absence
   * of contact or constraints. */
  EXPECT_EQ(free_motion_fem_state.GetPositions(),
            next_fem_state.GetPositions());
  EXPECT_EQ(free_motion_fem_state.GetVelocities(),
            next_fem_state.GetVelocities());
  EXPECT_EQ(free_motion_fem_state.GetAccelerations(),
            next_fem_state.GetAccelerations());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
