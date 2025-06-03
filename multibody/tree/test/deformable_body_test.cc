#include "drake/multibody/tree/deformable_body.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/force_density_field.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Matrix3X;
using Eigen::Vector3d;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::Sphere;
using math::RigidTransformd;
using systems::Context;
using systems::DiagramBuilder;

/* Builds a plant with exactly one deformable body, then finalizes and
 retrieves the DeformableBody for direct testing. Most of the DeformableBody
 APIs are already tested through the DeformableModel tests (e.g. adding
 constraints, boundary conditions, enabling/disabling a body, etc), and here we
 only test the APIs not fully covered in DeformableModel tests. */
class DeformableBodyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    MultibodyPlantConfig plant_config;
    plant_config.time_step = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlant(plant_config, &builder_);

    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();

    ModelInstanceIndex model_instance = plant_->AddModelInstance("deformable");

    auto sphere = std::make_unique<GeometryInstance>(
        RigidTransformd::Identity(), std::make_unique<Sphere>(1.0),
        "test_sphere");
    sphere->set_proximity_properties({});
    body_id_ = deformable_model.RegisterDeformableBody(
        std::move(sphere), model_instance, default_body_config_,
        /*resolution_hint=*/0.5);

    plant_->Finalize();
    diagram_ = builder_.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());

    body_index_ = deformable_model.GetBodyIndex(body_id_);
    body_ = &deformable_model.GetBody(body_id_);
    mutable_body_ = &deformable_model.GetMutableBody(body_id_);
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_{nullptr};

  const fem::DeformableBodyConfig<double> default_body_config_{};
  DeformableBodyId body_id_;
  DeformableBodyIndex body_index_;
  const DeformableBody<double>* body_{nullptr};
  DeformableBody<double>* mutable_body_{nullptr};
};

TEST_F(DeformableBodyTest, Accessors) {
  /* index(), body_id(), name(), scoped_name(), geometry_id() */
  EXPECT_EQ(body_->index(), body_index_);
  EXPECT_EQ(body_->body_id(), body_id_);
  EXPECT_EQ(body_->name(), "test_sphere");
  EXPECT_EQ(body_->scoped_name().get_full(), "deformable::test_sphere");
  EXPECT_TRUE(scene_graph_->model_inspector().IsDeformableGeometry(
      body_->geometry_id()));
  /* config() matches what we registered */
  EXPECT_EQ(body_->config().youngs_modulus(),
            default_body_config_.youngs_modulus());
  EXPECT_EQ(body_->config().mass_density(),
            default_body_config_.mass_density());
  /* FEM model accessor. */
  const fem::FemModel<double>& fem_model = body_->fem_model();
  EXPECT_EQ(fem_model.num_dofs(), body_->num_dofs());
  /* External forces. */
  const std::vector<const ForceDensityFieldBase<double>*>& external_forces =
      body_->external_forces();
  ASSERT_EQ(external_forces.size(), 1);  // gravity is the only one.
  const ForceDensityFieldBase<double>* gravity_force = external_forces[0];
  const GravityForceField<double>* gravity_force_field =
      dynamic_cast<const GravityForceField<double>*>(gravity_force);
  ASSERT_NE(gravity_force_field, nullptr);
  EXPECT_EQ(gravity_force_field->EvaluateAt(*plant_context_, Vector3d(1, 2, 3)),
            Vector3d(0, 0, -9.81) * default_body_config_.mass_density());

  /* Discrete state index. */
  const systems::DiscreteStateIndex discrete_state_index =
      body_->discrete_state_index();
  EXPECT_EQ(plant_context_->get_discrete_state(discrete_state_index).size(),
            3 * body_->num_dofs());  // positions + velocities + accelerations.
  /* is_enabled_parameter_index() */
  const systems::AbstractParameterIndex is_enabled_index =
      body_->is_enabled_parameter_index();
  EXPECT_EQ(
      plant_context_->get_parameters().template get_abstract_parameter<bool>(
          is_enabled_index),
      true);
}

TEST_F(DeformableBodyTest, NumDofsAndReferencePositions) {
  const int num_dofs = body_->num_dofs();
  EXPECT_GT(num_dofs, 0);
  const VectorX<double>& q_ref = body_->reference_positions();
  EXPECT_EQ(q_ref.size(), num_dofs);
  /* Ensure that GetPositions() initially returns the same reference positions.
   */
  Matrix3X<double> q = body_->GetPositions(*plant_context_);
  Eigen::Map<const Eigen::VectorXd> q_flat(q.data(), q.size());
  EXPECT_TRUE(CompareMatrices(q_flat, q_ref));
}

TEST_F(DeformableBodyTest, SetGetPositions) {
  const int n = body_->num_dofs();
  Matrix3X<double> q(3, n / 3);
  for (int i = 0; i < q.cols(); ++i) {
    q.col(i) = Vector3d(0.1 * i, -0.2 * i, 0.3 * i);
  }
  body_->SetPositions(plant_context_, q);
  EXPECT_TRUE(CompareMatrices(body_->GetPositions(*plant_context_), q));
}

TEST_F(DeformableBodyTest, Parallelism) {
  mutable_body_->set_parallelism(Parallelism(4));
  EXPECT_EQ(body_->fem_model().parallelism().num_threads(), 4);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
