#include <gtest/gtest.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::internal::DeformableContact;
using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::PartialPermutation;
using drake::systems::Context;
using drake::systems::DiscreteStateIndex;
using Eigen::VectorXd;
using std::make_unique;
using std::move;

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// CompliantContactManager for testing purposes.
class CompliantContactManagerTester {
 public:
  static const DeformableDriver<double>* deformable_driver(
      const CompliantContactManager<double>& manager) {
    return manager.deformable_driver_.get();
  }
};

class DeformableDriverContactTest : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr double kDt = 0.01;
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    body_id0_ =
        RegisterDeformableOctahedron(deformable_model.get(), "deformable0");
    body_id1_ =
        RegisterDeformableOctahedron(deformable_model.get(), "deformable1");
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);
    /* Register a rigid collision geometry intersecting with the bottom half of
     the deformable octahedrons. */
    geometry::ProximityProperties proximity_prop;
    geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                                 &proximity_prop);
    // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
    //  as the trigger for contact with deformable bodies.
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    RigidTransformd X_WG(Vector3<double>(0, 0, -0.75));
    rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_WG, geometry::Box(1, 1, 1),
        "rigid_collision_geometry", proximity_prop);
    plant_->Finalize();

    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(move(contact_manager));
    driver_ = CompliantContactManagerTester::deformable_driver(*manager_);

    builder.Connect(model_->vertex_positions_port(),
                    scene_graph_->get_source_configuration_port(
                        plant_->get_source_id().value()));
    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
  }

  /* Forwarding calls to functions in DeformableDriver with the same name.
   @{ */
  const DeformableContact<double>& EvalDeformableContact(
      const Context<double>& context) const {
    return driver_->EvalDeformableContact(context);
  }

  const PartialPermutation& EvalDofPermutation(
      const Context<double>& context, DeformableBodyIndex index) const {
    return driver_->EvalDofPermutation(context, index);
  }

  const VectorXd& EvalParticipatingVelocities(
      const Context<double>& context) const {
    return driver_->EvalParticipatingVelocities(context);
  }
  /* @} */

  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_;
  DeformableBodyId body_id0_;
  DeformableBodyId body_id1_;
  GeometryId rigid_geometry_id_;

 private:
  DeformableBodyId RegisterDeformableOctahedron(DeformableModel<double>* model,
                                                std::string name) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(1.0), std::move(name));
    geometry->set_proximity_properties(geometry::ProximityProperties());
    const fem::DeformableBodyConfig<double> default_body_config;
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId body_id = model->RegisterDeformableBody(
        move(geometry), default_body_config, kRezHint);
    return body_id;
  }
};

namespace {

TEST_F(DeformableDriverContactTest, EvalDeformableContact) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const DeformableContact<double>& contact =
      EvalDeformableContact(plant_context);
  ASSERT_EQ(contact.contact_surfaces().size(), 2);
  /* id_B should refer to the rigid geometry. */
  EXPECT_EQ(contact.contact_surfaces()[0].id_B(), rigid_geometry_id_);
  EXPECT_EQ(contact.contact_surfaces()[1].id_B(), rigid_geometry_id_);

  /* All but the top vertex in each octahedron should participate in contact. */
  GeometryId geometry_id0 = model_->GetGeometryId(body_id0_);
  GeometryId geometry_id1 = model_->GetGeometryId(body_id1_);
  EXPECT_EQ(
      contact.contact_participation(geometry_id0).num_vertices_in_contact(), 6);
  EXPECT_EQ(
      contact.contact_participation(geometry_id1).num_vertices_in_contact(), 6);
}

TEST_F(DeformableDriverContactTest, EvalDofPermutation) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const PartialPermutation& result =
      EvalDofPermutation(plant_context, DeformableBodyIndex(0));
  /* Here we use our knowledge that Drake's coarsest sphere mesh generation is
   indexed such that the top vertex is indexed 5. (Vertex 0-4 are on the
   equator, vertex 5 is the north pole, and vertex 6 is the south pole) */
  const std::vector<int> expected_permutation = {{0,  1,  2,  3,  4,  5,  6,
                                                  7,  8,  9,  10, 11, 12, 13,
                                                  14, -1, -1, -1, 15, 16, 17}};
  EXPECT_EQ(result.permutation(), expected_permutation);
}

/* Tests EvalParticipatingVelocities as well as
 EvalParticipatingVelocityMultiplexer. */
TEST_F(DeformableDriverContactTest, EvalParticipatingVelocities) {
  /* Set states for both bodies so that they have different velocities. */
  const VectorXd& q = model_->GetReferencePositions(body_id0_);
  const int num_dofs = q.size();
  const auto v0 = VectorXd::Zero(num_dofs);
  const auto v1 = VectorXd::Ones(num_dofs);
  const auto a = VectorXd::Zero(num_dofs);
  VectorXd state0_value(3 * num_dofs);
  state0_value << q, v0, a; /* position, velocity, acceleration. */
  VectorXd state1_value(3 * num_dofs);
  state1_value << q, v1, a; /* position, velocity, acceleration. */
  const DiscreteStateIndex state0_index =
      model_->GetDiscreteStateIndex(body_id0_);
  const DiscreteStateIndex state1_index =
      model_->GetDiscreteStateIndex(body_id1_);
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context_.get());
  plant_context.SetDiscreteState(state0_index, state0_value);
  plant_context.SetDiscreteState(state1_index, state1_value);

  const int num_participating_vertices = 6;
  const int num_participating_dofs_per_body = num_participating_vertices * 3;
  /* Verify that the participating velocities are multiplexed correctly. */
  VectorXd expected_participating_velocity(2 * num_participating_dofs_per_body);
  expected_participating_velocity
      << VectorXd::Zero(num_participating_dofs_per_body),
      VectorXd::Ones(num_participating_dofs_per_body);
  EXPECT_EQ(EvalParticipatingVelocities(plant_context),
            expected_participating_velocity);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
