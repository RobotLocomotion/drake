#include "drake/multibody/plant/deformable_driver.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
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

namespace drake {
namespace multibody {
namespace internal {

class DeformableDriverTest : public ::testing::Test {
 protected:
  static constexpr double kDt = 0.01;

  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    constexpr double kRezHint = 0.5;
    body_id_ = RegisterSphere(&deformable_model, kRezHint);
    model_ = &deformable_model;
    const RigidBody<double>& body = plant_->AddRigidBody(
        "rigid_body", SpatialInertia<double>::SolidSphereWithMass(1.0, 1.0));
    // N.B. Deformables are only supported with the SAP solver.
    // Thus for testing we choose one arbitrary contact approximation that uses
    // the SAP solver.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);
    plant_->Finalize();
    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(contact_manager));
    driver_ = manager_->deformable_driver();
    DRAKE_DEMAND(driver_ != nullptr);
    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
    // Lock the rigid body to test locking support in the presence of deformable
    // DoFs.
    body.Lock(plant_context_);
  }

  /* Forwarding calls to private member functions in DeformableDriver with the
   same name.
   @{ */
  const FemState<double>& EvalFemState(const systems::Context<double>& context,
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

  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
  DeformableBodyId body_id_;

 private:
  DeformableBodyId RegisterSphere(DeformableModel<double>* model,
                                  double resolution_hint) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(1), "sphere");
    geometry::ProximityProperties props;
    geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                                 &props);
    geometry->set_proximity_properties(std::move(props));
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(1e6);
    DeformableBodyId body_id = model->RegisterDeformableBody(
        std::move(geometry), body_config, resolution_hint);
    return body_id;
  }
};

namespace {

/* Verifies that a DeformableDriver has been successfully created. */
TEST_F(DeformableDriverTest, Constructor) {
  ASSERT_NE(driver_, nullptr);
}

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
  plant_context_->SetDiscreteState(state_index, state_value);
  const FemState<double>& fem_state =
      EvalFemState(*plant_context_, DeformableBodyIndex(0));
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
      EvalFreeMotionFemState(*plant_context_, DeformableBodyIndex(0));
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

TEST_F(DeformableDriverTest, FreeMotionFemStateDisabled) {
  model_->Disable(body_id_, plant_context_);

  const VectorX<double> q = model_->GetReferencePositions(body_id_);
  const int num_dofs = q.size();
  const FemState<double>& free_motion_fem_state =
      EvalFreeMotionFemState(*plant_context_, DeformableBodyIndex(0));
  // Free motion velocity/acceleration should be exactly zero for disabled
  // bodies. Position should remain unchanged.
  const VectorX<double> next_a = VectorX<double>::Zero(num_dofs);
  const VectorX<double> next_v = VectorX<double>::Zero(num_dofs);
  const VectorX<double> next_q = q;
  EXPECT_TRUE(CompareMatrices(free_motion_fem_state.GetPositions(), next_q));
  EXPECT_TRUE(CompareMatrices(free_motion_fem_state.GetVelocities(), next_v));
  EXPECT_TRUE(
      CompareMatrices(free_motion_fem_state.GetAccelerations(), next_a));
}

TEST_F(DeformableDriverTest, NextFemState) {
  const FemState<double>& free_motion_fem_state =
      EvalFreeMotionFemState(*plant_context_, DeformableBodyIndex(0));
  const FemState<double>& next_fem_state =
      EvalNextFemState(*plant_context_, DeformableBodyIndex(0));
  /* The next state should be the same as the free motion state in the absence
   of contact or constraints. */
  EXPECT_EQ(free_motion_fem_state.GetPositions(),
            next_fem_state.GetPositions());
  EXPECT_EQ(free_motion_fem_state.GetVelocities(),
            next_fem_state.GetVelocities());
  EXPECT_EQ(free_motion_fem_state.GetAccelerations(),
            next_fem_state.GetAccelerations());
}

/* Verifies that the discrete states are updated to match the FEM states. */
TEST_F(DeformableDriverTest, CalcDiscreteStates) {
  systems::Simulator<double> simulator(*diagram_, std::move(diagram_context_));
  simulator.Initialize();
  simulator.AdvanceTo(3 * kDt);
  const systems::DiscreteStateIndex state_index =
      model_->GetDiscreteStateIndex(body_id_);
  const VectorX<double>& discrete_state =
      plant_context_->get_discrete_state(state_index).value();
  const int num_dofs = model_->GetFemModel(body_id_).num_dofs();
  const FemState<double>& fem_state =
      EvalFemState(*plant_context_, DeformableBodyIndex(0));
  EXPECT_EQ(discrete_state.head(num_dofs), fem_state.GetPositions());
  EXPECT_EQ(discrete_state.segment(num_dofs, num_dofs),
            fem_state.GetVelocities());
  EXPECT_EQ(discrete_state.tail(num_dofs), fem_state.GetAccelerations());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
