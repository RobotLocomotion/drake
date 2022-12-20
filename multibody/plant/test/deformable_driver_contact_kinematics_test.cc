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
using drake::multibody::contact_solvers::internal::PartialPermutation;
using drake::systems::Context;
using drake::systems::DiscreteStateIndex;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
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

/* Registers a deformable octrahedron with the given `name` and pose in the
 world to the given `model`.
 The octahedron looks like this in its geometry frame, F.
                  +Fz   -Fx
                   |   /
                   v5 v3
                   | /
                   |/
   -Fy---v4------v0+------v2---+ Fy
                  /| Fo
                 / |
               v1  v6
               /   |
             +Fx   |
                  -Fz
*/
DeformableBodyId RegisterDeformableOctahedron(DeformableModel<double>* model,
                                              std::string name,
                                              const RigidTransformd& X_WF) {
  auto geometry =
      make_unique<GeometryInstance>(X_WF, make_unique<Sphere>(1.0), move(name));
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

/* Test fixture to test DeformableDriver::AppendContactKinematics.
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
   of the bodies are set so that the contact normals are all equal to -Fz, for
   an arbitrarily chosen frame F. Velocities of the body are set so that the
   contact velocity in the C frame is (0, 0, -1). */
  void MakeScene(bool dynamic_rigid_body) {
    systems::DiagramBuilder<double> builder;
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    deformable_body_id_ = RegisterDeformableOctahedron(deformable_model.get(),
                                                       "deformable", X_WF_);
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

    const RigidTransformd X_FR(Vector3<double>(0, 0, -0.75));
    const RigidTransformd X_WR = X_WF_ * X_FR;
    if (dynamic_rigid_body) {
      /* Register a dynamic rigid body intersecting with the bottom half of the
       deformable octahedron. The rigid body is below the deformable body along
       Fz and the contact normal is defined to point into the rigid body (in the
       -Fz direction), so Cz = -Fz. */
      const RigidBody<double>& rigid_body =
          plant_->AddRigidBody("rigid_body", SpatialInertia<double>());
      const RigidTransformd& X_BR = X_WR;  // because the pose of the rigid body
                                           // in world, X_WB, is identity.
      rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
          rigid_body, X_BR, geometry::Box(10, 10, 1),
          "dynamic_collision_geometry", rigid_proximity_props);
      rigid_body_index_ = rigid_body.index();
    } else {
      /* Register the same rigid geometry, but static instead of dynamic. */
      rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
          plant_->world_body(), X_WR, geometry::Box(10, 10, 1),
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

    /* We define the contact velocity as the velocity of the Rp, the point on
     the rigid body coincident with the contact point, relative to Dp, the point
     on the deformable body coincident with the contact point. In monogram
     notation, this is v_DpRp. We set up the deformable body velocity and the
     rigid body velocity so that the v_DpRp_F at each contact points is
     conveniently (0, 0, 1). Because we set up the geometries such that Cz = -Fz
     at all contact points, we can expect the contact velocities in the C
     frames, v_DpRp_C, are (0, 0, -1). */
    if (dynamic_rigid_body) {
      /* The rigid body can move, so we'll achieve the desired relative velocity
       by splitting it evenly between the rigid and deformable bodies. */
      const Vector3d v_WD_F = Vector3d(0, 0, -0.5);
      const Vector3d v_WD = X_WF_.rotation() * v_WD_F;
      SetVelocity(deformable_body_id_, v_WD);
      Context<double>& mutable_plant_context =
          plant_->GetMyMutableContextFromRoot(context_.get());
      /* Only the deformable geometry can move; it gets the full desired
       relative velocity. */
      const Vector3d v_WR_F = Vector3d(0, 0, 0.5);
      const SpatialVelocity<double> V_WR(Vector3d::Zero(),
                                         X_WF_.rotation() * v_WR_F);
      plant_->SetFreeBodySpatialVelocity(
          &mutable_plant_context, plant_->get_body(rigid_body_index_), V_WR);
    } else {
      /* Set the velocity of the deformable body so that it's (0, 0, -1.0) in
       the F frame. */
      const Vector3d v_WD_F = Vector3d(0, 0, -1.0);
      SetVelocity(deformable_body_id_, X_WF_.rotation() * v_WD_F);
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
    /* Velocities of the participating deformable dofs. */
    const VectorXd v0 = driver_->EvalParticipatingVelocities(plant_context);
    const Vector3d expected_v_DpRp_C(0, 0, -1);
    for (int i = 0; i < num_contact_points; ++i) {
      const ContactPairKinematics<double>& contact_kinematic =
          contact_kinematics[i];
      EXPECT_LT(contact_kinematic.phi, 0.0);
      EXPECT_TRUE(contact_kinematic.R_WC.IsNearlyEqualTo(expected_R_WC, 1e-12));
      if (dynamic_rigid_body) {
        ASSERT_EQ(contact_kinematic.jacobian.size(), 2);
        const Matrix3X<double> J0 = contact_kinematic.jacobian[0].J;
        ASSERT_EQ(v0.size(), J0.cols());
        const Matrix3X<double> J1 = contact_kinematic.jacobian[1].J;
        const Vector3d v_WR_F(0, 0, 0.5);
        const Vector3d v_WR = X_WF_.rotation() * v_WR_F;
        const Vector3d w_WR(0, 0, 0);
        Vector6<double> v1;
        v1 << w_WR, v_WR;
        ASSERT_EQ(v1.size(), J1.cols());
        EXPECT_TRUE(
            CompareMatrices(J0 * v0 + J1 * v1, expected_v_DpRp_C, 1e-14));
      } else {
        ASSERT_EQ(contact_kinematic.jacobian.size(), 1);
        const Matrix3X<double> J0 = contact_kinematic.jacobian[0].J;
        ASSERT_EQ(v0.size(), J0.cols());
        EXPECT_TRUE(CompareMatrices(J0 * v0, expected_v_DpRp_C, 1e-14));
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

class DeformableDriverTest {
 public:
  static const PartialPermutation& EvalVertexPermutation(
      const DeformableDriver<double>& driver, const Context<double>& context,
      GeometryId id) {
    return driver.EvalVertexPermutation(context, id);
  }
};

namespace {
/* Verify that jacobian columns corresponding to dofs under zero dirichlet
 boundary conditions are zeroed out. */
GTEST_TEST(DeformableDriverContactKinematicsWithBcTest,
           ContactJacobianWithBoundaryCondition) {
  systems::DiagramBuilder<double> builder;
  constexpr double kDt = 0.01;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, kDt);
  auto deformable_model = make_unique<DeformableModel<double>>(&plant);
  DeformableBodyId body_id = RegisterDeformableOctahedron(
      deformable_model.get(), "deformable", RigidTransformd::Identity());
  DeformableModel<double>* model = deformable_model.get();
  /* Put the bottom vertex under bc. */
  model->SetWallBoundaryCondition(body_id, Vector3d(0, 0, -0.5),
                                  Vector3d(0, 0, 1));
  plant.AddPhysicalModel(move(deformable_model));

  /* Define proximity properties for all rigid geometries. */
  geometry::ProximityProperties rigid_proximity_props;
  geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                               &rigid_proximity_props);
  // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
  //  as the trigger for contact with deformable bodies.
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint, 1.0);
  /* Shift the rigid body so that only the bottom half of the octahedron is in
   contact. */
  const RigidTransformd X_WR(Vector3<double>(0, 0, -0.75));
  /* Register a static rigid geometry in contact with the bottom half of the
   octahedron. */
  plant.RegisterCollisionGeometry(
      plant.world_body(), X_WR, geometry::Box(10, 10, 1),
      "static_collision_geometry", rigid_proximity_props);

  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  plant.Finalize();
  auto contact_manager = make_unique<CompliantContactManager<double>>();
  const CompliantContactManager<double>* manager = contact_manager.get();
  plant.SetDiscreteUpdateManager(move(contact_manager));
  const DeformableDriver<double>* driver =
      CompliantContactManagerTester::deformable_driver(*manager);

  builder.Connect(
      model->vertex_positions_port(),
      scene_graph.get_source_configuration_port(plant.get_source_id().value()));
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const Context<double>& plant_context = plant.GetMyContextFromRoot(*context);
  std::vector<ContactPairKinematics<double>> contact_kinematics;
  driver->AppendContactKinematics(plant_context, &contact_kinematics);
  /* The set of contact points is not empty. */
  EXPECT_GT(contact_kinematics.size(), 0);
  const PartialPermutation& vertex_permutation =
      DeformableDriverTest::EvalVertexPermutation(
          *driver, plant_context, model->GetGeometryId(body_id));
  /* We know that the octahedron created as the coarsest sphere indexes the
   bottom vertex as 6 (see RegisterDeformableOctahedron). */
  const int bottom_vertex_permuted_index = vertex_permutation.permuted_index(6);
  for (int i = 0; i < static_cast<int>(contact_kinematics.size()); ++i) {
    const ContactPairKinematics<double>& contact_kinematic =
        contact_kinematics[i];
    /* The first jacobian entry corresponds to the contribution of the
     deformable clique. */
    const Matrix3X<double>& J = contact_kinematic.jacobian[0].J;
    /* The jacobian isn't completely zero. */
    EXPECT_GT(J.norm(), 0.01);
    /* But the columns corresponding to the vertex under bc are set to zero. */
    EXPECT_TRUE(CompareMatrices(
        J.middleCols<3>(3 * bottom_vertex_permuted_index), Matrix3d::Zero()));
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
