#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::internal::DeformableContact;
using drake::geometry::internal::DeformableContactSurface;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::systems::Context;
using drake::systems::DiscreteStateIndex;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;

namespace drake {
namespace multibody {
namespace internal {

/* Friend class used to provide access to a selection of private functions in
 CompliantContactManager for testing purposes. */
class CompliantContactManagerTester {
 public:
  static const DeformableDriver<double>* deformable_driver(
      const CompliantContactManager<double>& manager) {
    return manager.deformable_driver_.get();
  }
};

/* Test fixure to test DeformableDriver::AppendContactKinematics.
 In particular, this fixture sets up a deformable octahedron centered at world
 origin with 8 elements, 7 vertices, and 21 dofs. A rigid rectangle is added so
 that its top face intersects the bottom half of the deformable octahedron. The
 rigid rectangle can be configured to be dynamic or static to provide coverage
 for the cases with one and two Jacobian blocks. We compare rotation matrix from
 world to the contact frame and contact velocities to expected values. */
class DeformableDriverContactKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() {}

  /* Sets up the scene described in the class documentation. The rigid body is
   dynamic if `dynamic_rigid_body` is true. Otherwise, a collision geometry
   bound to the world body with the same shape is added. The pose/configuration
   of the bodies are set so that the contact normal in an arbitrarily chosen F
   frame is (0, 0, 1). Velocities of the body are set so that the contact
   velocity in the contact frame is (0, 0, 1). */
  void MakeScene(bool dynamic_rigid_body) {
    systems::DiagramBuilder<double> builder;
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    deformable_body_id_ =
        RegisterDeformableOctahedron(deformable_model.get(), "deformable");
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));

    /* Define proximity properties for all rigid geometries. */
    geometry::ProximityProperties rigid_proximity_props;
    geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                                 &rigid_proximity_props);
    // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
    //  as the trigger for contact with deformable bodies.
    rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                      geometry::internal::kRezHint, 1.0);

    const RigidTransformd X_FB(Vector3<double>(0, 0, -0.75));
    const RigidTransformd X_WB = X_WF_ * X_FB;
    if (dynamic_rigid_body) {
      /* Register a dynamic rigid body intersecting with the bottom half of the
       deformable octahedron. */
      const RigidBody<double>& rigid_body =
          plant_->AddRigidBody("rigid_body", SpatialInertia<double>());
      rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
          rigid_body, X_WB, geometry::Box(10, 10, 1),
          "dynamic_collision_geometry", rigid_proximity_props);
      rigid_body_index_ = rigid_body.index();
    } else {
      /* Register a rigid static collision geometry intersecting with the bottom
       half of the deformable octahedron. */
      rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
          plant_->world_body(), X_WB, geometry::Box(10, 10, 1),
          "static_collision_geometry", rigid_proximity_props);
    }

    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);
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

    /* Set up the deformable body velocity and the rigid body velocity so that
     the contact velocities in the contact frame are (0, 0, 1). */
    if (dynamic_rigid_body) {
      SetVelocity(deformable_body_id_, Vector3d(0, 0, -0.5));
      Context<double>& mutable_plant_context =
          plant_->GetMyMutableContextFromRoot(context_.get());
      plant_->SetFreeBodySpatialVelocity(
          &mutable_plant_context, plant_->get_body(rigid_body_index_),
          SpatialVelocity<double>(Vector3d(0, 0, 0), Vector3d(0, 0, 0.5)));
    } else {
      SetVelocity(deformable_body_id_, Vector3d(0, 0, -1.0));
    }
  }

  /* Verifies contact kinematics data are as expected.
   @param[in] dynamic_rigid_body  True if a rigid body is registered in the
   setup (instead of a static body with a collision geometry). */
  void ValidateContactKinematics(bool dynamic_rigid_body) {
    /* Each discrete contact pair should create a contact kinematics pair. */
    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(*context_);
    std::vector<ContactPairKinematics<double>> contact_kinematics;
    driver_->AppendContactKinematics(plant_context, &contact_kinematics);
    const int num_contact_points = GetNumContactPoints(plant_context);
    ASSERT_EQ(contact_kinematics.size(), num_contact_points);

    /* The contact surfaces are parallel to the xy-plane in frame F and the
     contact normal points from the deformable body into the rigid body. */
    const Vector3d nhat_F(0, 0, -1);
    const Vector3d nhat_W = X_WF_.rotation() * nhat_F;
    constexpr int kZAxis = 2;
    const math::RotationMatrixd expected_R_WC =
        math::RotationMatrixd::MakeFromOneUnitVector(nhat_W, kZAxis);
    const Vector3d expected_vc(0, 0, -1);
    for (int i = 0; i < num_contact_points; ++i) {
      const ContactPairKinematics<double>& contact_kinematic =
          contact_kinematics[i];
      EXPECT_LT(contact_kinematic.phi, 0.0);
      EXPECT_TRUE(contact_kinematic.R_WC.IsNearlyEqualTo(
          expected_R_WC, std::numeric_limits<double>::epsilon()));
      if (dynamic_rigid_body) {
        ASSERT_EQ(contact_kinematic.jacobian.size(), 2);
        const Matrix3X<double> J0 = contact_kinematic.jacobian[0].J;
        const VectorXd v0 = driver_->EvalParticipatingVelocities(plant_context);
        ASSERT_EQ(v0.size(), J0.cols());
        const Matrix3X<double> J1 = contact_kinematic.jacobian[1].J;
        Vector6<double> v1;
        v1 << 0, 0, 0, 0, 0, 0.5;
        ASSERT_EQ(v1.size(), J1.cols());
        EXPECT_TRUE(CompareMatrices(J0 * v0 + J1 * v1, expected_vc));
      } else {
        ASSERT_EQ(contact_kinematic.jacobian.size(), 1);
        const Matrix3X<double> J0 = contact_kinematic.jacobian[0].J;
        const VectorXd v0 = driver_->EvalParticipatingVelocities(plant_context);
        ASSERT_EQ(v0.size(), J0.cols());
        EXPECT_TRUE(CompareMatrices(J0 * v0, expected_vc));
      }
    }
  }

  const math::RigidTransformd X_WF_{RollPitchYawd(1, 2, 3), Vector3d(4, 5, 6)};
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_;
  DeformableBodyId deformable_body_id_;
  BodyIndex rigid_body_index_;
  GeometryId rigid_geometry_id_;

 private:
  DeformableBodyId RegisterDeformableOctahedron(DeformableModel<double>* model,
                                                std::string name) {
    auto geometry = make_unique<GeometryInstance>(
        X_WF_, make_unique<Sphere>(1.0), move(name));
    geometry::ProximityProperties props;
    geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                                 &props);
    geometry->set_proximity_properties(move(props));
    fem::DeformableBodyConfig<double> body_config;
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId body_id =
        model->RegisterDeformableBody(move(geometry), body_config, kRezHint);
    return body_id;
  }

  /* Sets all vertices of the deformable body to have the given velocity in
   world frame. */
  void SetVelocity(DeformableBodyId id, const Vector3d& v_WB) {
    Context<double>& plant_context =
        plant_->GetMyMutableContextFromRoot(context_.get());
    const DiscreteStateIndex state_index = model_->GetDiscreteStateIndex(id);
    VectorXd state = plant_context.get_discrete_state(state_index).value();
    DRAKE_DEMAND(state.size() % 3 == 0);  // q, v, a all the same length.
    const int num_dofs = state.size() / 3;
    DRAKE_DEMAND(num_dofs % 3 == 0);  // Each vertex needs a 3-vector.
    const int num_vertices = num_dofs / 3;
    VectorXd velocities(num_dofs);
    for (int i = 0; i < num_vertices; ++i) {
      velocities.segment<3>(3 * i) = v_WB;
    }
    state.segment(num_dofs, num_dofs) = velocities;
    plant_context.SetDiscreteState(state_index, state);
  }

  /* Gets the total number of contact points between deformable bodies and rigid
   bodies. */
  int GetNumContactPoints(const Context<double>& plant_context) const {
    const DeformableContact<double>& contact_data =
        driver_->EvalDeformableContact(plant_context);
    int num_contact_points = 0;
    for (const DeformableContactSurface<double>& surface :
         contact_data.contact_surfaces()) {
      num_contact_points += surface.num_contact_points();
    }
    return num_contact_points;
  }
};

namespace {

TEST_F(DeformableDriverContactKinematicsTest,
       AppendContactKinematicsStaticRigid) {
  MakeScene(false);
  ValidateContactKinematics(false);
}

TEST_F(DeformableDriverContactKinematicsTest,
       AppendContactKinematicsDynamicRigid) {
  MakeScene(true);
  ValidateContactKinematics(true);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
