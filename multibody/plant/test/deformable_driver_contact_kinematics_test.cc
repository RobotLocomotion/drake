#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::internal::ContactParticipation;
using drake::geometry::internal::DeformableContact;
using drake::geometry::internal::DeformableContactSurface;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::contact_solvers::internal::FixedConstraintKinematics;
using drake::multibody::contact_solvers::internal::PartialPermutation;
using drake::systems::Context;
using drake::systems::DiscreteStateIndex;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace multibody {
namespace internal {

constexpr double kTolerance = 1e-14;
constexpr double kMu = 1.0;

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
  auto geometry = make_unique<GeometryInstance>(X_WF, make_unique<Sphere>(1.0),
                                                std::move(name));
  geometry::ProximityProperties props;
  geometry::AddContactMaterial({}, {}, CoulombFriction<double>(kMu, kMu),
                               &props);
  geometry->set_proximity_properties(std::move(props));
  fem::DeformableBodyConfig<double> body_config;
  /* Make the resolution hint large enough so that we get an octahedron. */
  constexpr double kRezHint = 10.0;
  DeformableBodyId body_id =
      model->RegisterDeformableBody(std::move(geometry), body_config, kRezHint);
  return body_id;
}

/* Test fixture to test DeformableDriver::AppendDiscreteContactPairs and
 DeformableDriver::AppendDeformableRigidFixedConstraintKinematics.
 In particular, this fixture sets up environments where the contact/constraint
 information can easily be computed by hand. We then compare the result computed
 by DeformableDriver and the hand-calculated result to confirm correctness. */
class DeformableDriverContactKinematicsTest
    : public ::testing::TestWithParam<bool> {
 protected:
  void SetUp() {}

  /* Sets up a deformable octahedron centered at world origin with 8 elements, 7
  vertices, and 21 dofs. To test contact kinematics, we add a rigid box so
  that its top face intersects the bottom half of the deformable octahedron. The
  rigid rectangle can be configured to be dynamic or static to provide coverage
  for the cases with one and two Jacobian blocks. We compare rotation matrix
  from world to the contact frame and contact velocities to expected values. The
  pose/configuration of the bodies are set so that the contact normals are all
  equal to -Fz, for an arbitrarily chosen frame F. Velocities of the body are
  set so that the contact velocity in the C frame is (0, 0, -1).

  Similarly, to test fixed constraint kinematics, we add fixed constraint
  between the rigid body and the vertex of the deformable body inside the rigid
  geometry. We then compare the constraint velocities to expected values.

  This setup provides coverage for deformable vs. rigid contact as well as
  fixed constraints between deformable bodies and rigid bodies.

  @param[in] dynamic_rigid_body
  The rigid body is dynamic if `dynamic_rigid_body` is true. Otherwise, a
  collision geometry bound to the world body with the same shape is added. */
  void MakeDeformableRigidScene(bool dynamic_rigid_body) {
    systems::DiagramBuilder<double> builder;
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    deformable_body_id_ =
        RegisterDeformableOctahedron(&deformable_model, "deformable", X_WF_);
    model_ = &plant_->deformable_model();

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
    const RigidTransformd& X_BR = X_WR;  // because the pose of the rigid body
                                         // in world, X_WB, is identity.
    if (dynamic_rigid_body) {
      /* Register a dynamic rigid body intersecting with the bottom half of the
       deformable octahedron. The rigid body is below the deformable body along
       Fz and the contact normal is defined to point into the rigid body (in the
       -Fz direction), so Cz = -Fz. */
      const RigidBody<double>& rigid_body =
          plant_->AddRigidBody("rigid_body", SpatialInertia<double>::NaN());
      rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
          rigid_body, X_BR, geometry::Box(10, 10, 1),
          "dynamic_collision_geometry", rigid_proximity_props);
      rigid_body_index_ = rigid_body.index();
    } else {
      /* Register the same rigid geometry, but static instead of dynamic. */
      rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
          plant_->world_body(), X_WR, geometry::Box(10, 10, 1),
          "static_collision_geometry", rigid_proximity_props);
      rigid_body_index_ = plant_->world_body().index();
    }

    /* Add fixed constraint. */
    if (dynamic_rigid_body) {
      /* The pose of the deformable body in the rigid body's frame, X_BF, is
       equal to X_WF because X_WB is identity. */
      const RigidTransformd X_BF = X_WF_;
      deformable_model.AddFixedConstraint(deformable_body_id_,
                                          plant_->get_body(rigid_body_index_),
                                          X_BF, geometry::Box(10, 10, 1), X_BR);
    } else {
      deformable_model.AddFixedConstraint(deformable_body_id_,
                                          plant_->world_body(), X_WF_,
                                          geometry::Box(10, 10, 1), X_BR);
    }

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

  /* Sets up a deformable octahedron centered at world origin with 8 elements, 7
  vertices, and 21 dofs. To test contact kinematics for deformable vs.
  deformable contact, we add another deformable octahedron so that that its top
  half intersects the bottom half of the first deformable octahedron. We compare
  rotation matrix from world to the contact frame and contact velocities to
  expected values. The configuration of the deformable bodies are set so that
  the contact normals are all equal to -Fz, for an arbitrarily chosen frame F.
  Velocities of the body are set so that the contact velocity in the C frame is
  (0, 0, -1). */
  void MakeDeformableDeformableScene() {
    systems::DiagramBuilder<double> builder;
    constexpr double kDt = 0.01;
    // N.B. Deformables are only supported with the SAP solver. Thus for testing
    // we choose one arbitrary contact approximation that uses the SAP solver.
    MultibodyPlantConfig config{.time_step = kDt,
                                .discrete_contact_approximation = "sap"};
    std::tie(plant_, scene_graph_) = AddMultibodyPlant(config, &builder);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    deformable_body_id_ =
        RegisterDeformableOctahedron(&deformable_model, "deformable", X_WF_);
    /* Sets up a second deformable body so that its top half intersects the
     bottom half of the first deformable body. */
    deformable_body_id2_ = RegisterDeformableOctahedron(
        &deformable_model, "deformable2",
        X_WF_ * RigidTransformd(Vector3d(0, 0, -1.25)));
    model_ = &plant_->deformable_model();

    plant_->Finalize();
    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(contact_manager));
    driver_ = manager_->deformable_driver();
    DRAKE_DEMAND(driver_ != nullptr);

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();

    /* Set the velocity of the first deformable body so that it's (0, 0, -0.5)
     when expressed in the F frame. */
    const Vector3d v_WD1_F = Vector3d(0, 0, -0.5);
    SetVelocity(deformable_body_id_, X_WF_.rotation() * v_WD1_F);
    /* Set the velocity of the second deformable body so that it's (0, 0, 0.5)
     when expressed in the F frame. */
    const Vector3d v_WD2_F = Vector3d(0, 0, 0.5);
    SetVelocity(deformable_body_id2_, X_WF_.rotation() * v_WD2_F);
  }

  /* Verifies contact kinematics data in the deformable vs. deformable contact
   scene are as expected. */
  void ValidateDeformableDeformableContactPairs() {
    /* Each discrete contact pair should create a contact kinematics pair. */
    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(*context_);
    DiscreteContactData<DiscreteContactPair<double>> contact_pairs;
    driver_->AppendDiscreteContactPairs(plant_context, &contact_pairs);
    const int num_contact_points = GetNumContactPoints(plant_context);
    ASSERT_EQ(contact_pairs.size(), num_contact_points);
    const DeformableContactSurface<double>& contact_surface =
        driver_->EvalDeformableContact(plant_context).contact_surfaces()[0];

    /* The contact surfaces are parallel to the xy-plane in frame F and the
     contact normal points from the geometry A into geometry B. We know that
     in deformable vs. deformable contact, the geometry with the smaller
     GeometryId is always treated as geometry A. Pairing that with the fact that
     the deformable body registered first has the smaller GeometryId, we know
     that the normal is point from the first deformable body to the second
     deformable body. */
    const Vector3d nhat_AB_F(0, 0, -1);
    const Vector3d nhat_AB_W = X_WF_.rotation() * nhat_AB_F;
    constexpr int kZAxis = 2;
    const math::RotationMatrixd expected_R_WC =
        math::RotationMatrixd::MakeFromOneUnitVector(nhat_AB_W, kZAxis);
    /* Velocities of the participating deformable dofs. */
    const VectorXd v = driver_->EvalParticipatingVelocities(plant_context);
    const Vector3d expected_v_D1D2_C(0, 0, -1);
    for (int i = 0; i < num_contact_points; ++i) {
      const DiscreteContactPair<double>& contact_pair = contact_pairs[i];
      /* Test Jacobian by verifying J*v = vc. */
      ASSERT_EQ(contact_pair.jacobian.size(), 2);
      const Matrix3X<double> J0 = contact_pair.jacobian[0].J.MakeDenseMatrix();
      const Matrix3X<double> J1 = contact_pair.jacobian[1].J.MakeDenseMatrix();
      Matrix3X<double> J(3, J0.cols() + J1.cols());
      J << J0, J1;
      ASSERT_EQ(v.size(), J.cols());
      EXPECT_TRUE(CompareMatrices(J * v, expected_v_D1D2_C, kTolerance));
      /* Test object index and geometry Id. */
      const DeformableBodyIndex body_index =
          model_->GetBodyIndex(deformable_body_id_);
      const int object_A = body_index + plant_->num_bodies();
      EXPECT_EQ(contact_pair.object_A, object_A);
      const DeformableBodyIndex body_index2 =
          model_->GetBodyIndex(deformable_body_id2_);
      const int object_B = body_index2 + plant_->num_bodies();
      EXPECT_EQ(contact_pair.object_B, object_B);
      EXPECT_EQ(contact_pair.id_A, model_->GetGeometryId(deformable_body_id_));
      EXPECT_EQ(contact_pair.id_B, model_->GetGeometryId(deformable_body_id2_));
      /* Test normal and rotation matrix. */
      EXPECT_TRUE(
          CompareMatrices(contact_pair.nhat_BA_W, -nhat_AB_W, kTolerance));
      EXPECT_TRUE(contact_pair.R_WC.IsNearlyEqualTo(expected_R_WC, kTolerance));
      /* Test contact point position in world frame and its position relative to
       the bodies in contact. */
      const Vector3d& expected_p_WC = contact_surface.contact_points_W()[i];
      const Vector3d expected_p_ApC_W =
          expected_p_WC - contact_surface.contact_mesh_W().centroid();
      /* For both deformable geomtries, the relative-to point are both the
       centroid of the contact mesh. */
      const Vector3d& expected_p_BqC_W = expected_p_ApC_W;
      EXPECT_EQ(contact_pair.p_WC, expected_p_WC);
      EXPECT_EQ(contact_pair.p_ApC_W, expected_p_ApC_W);
      EXPECT_EQ(contact_pair.p_BqC_W, expected_p_BqC_W);
      /* Test penetration distances are copied over. Here we simply confirm that
       the the sign is correct in the copy. The correctness of the sign distance
       computation is tested in the geometry module. */
      EXPECT_LT(contact_pair.phi0, 0.0);
      /* Test contact parameters (stiffness, damping, dissipation time scale and
       friction coefficient). */
      const double expected_k = 1e8 * contact_surface.contact_mesh_W().area(i) /
                                2.0;  // see implementation notes for why this
                                      // is the expected stiffness.
      EXPECT_DOUBLE_EQ(contact_pair.stiffness, expected_k);
      EXPECT_EQ(contact_pair.damping, 0.0);
      EXPECT_EQ(contact_pair.dissipation_time_scale, 0.02 /* 2 * dt*/);
      EXPECT_EQ(contact_pair.friction_coefficient, kMu);
      /* Test normal force and velocity. */
      EXPECT_NEAR(contact_pair.vn0, expected_v_D1D2_C(2), kTolerance);
      EXPECT_DOUBLE_EQ(contact_pair.fn0, -expected_k * contact_pair.phi0);
      /* Test surface index and face index*/
      EXPECT_EQ(contact_pair.surface_index, 0);
      EXPECT_EQ(contact_pair.face_index, i);
    }
  }

  /* Verifies contact kinematics data in the deformable vs. rigid contact scene
   are as expected.
   @param[in] dynamic_rigid_body  True if a rigid body is registered in the
   setup (instead of a static body with a collision geometry). */
  void ValidateDeformableRigidContactKinematics(bool dynamic_rigid_body) {
    /* Each discrete contact pair should create a contact kinematics pair. */
    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(*context_);
    DiscreteContactData<DiscreteContactPair<double>> contact_pairs;
    driver_->AppendDiscreteContactPairs(plant_context, &contact_pairs);
    const int num_contact_points = GetNumContactPoints(plant_context);
    ASSERT_EQ(contact_pairs.size(), num_contact_points);

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
      const DiscreteContactPair<double>& contact_pair = contact_pairs[i];
      EXPECT_LT(contact_pair.phi0, 0.0);
      EXPECT_TRUE(contact_pair.R_WC.IsNearlyEqualTo(expected_R_WC, 1e-12));
      if (dynamic_rigid_body) {
        ASSERT_EQ(contact_pair.jacobian.size(), 2);
        const Matrix3X<double> J0 =
            contact_pair.jacobian[0].J.MakeDenseMatrix();
        ASSERT_EQ(v0.size(), J0.cols());
        const Matrix3X<double> J1 =
            contact_pair.jacobian[1].J.MakeDenseMatrix();
        const Vector3d v_WR_F(0, 0, 0.5);
        const Vector3d v_WR = X_WF_.rotation() * v_WR_F;
        const Vector3d w_WR(0, 0, 0);
        Vector6<double> v1;
        v1 << w_WR, v_WR;
        ASSERT_EQ(v1.size(), J1.cols());
        EXPECT_TRUE(
            CompareMatrices(J0 * v0 + J1 * v1, expected_v_DpRp_C, kTolerance));
        EXPECT_NEAR(contact_pair.vn0, expected_v_DpRp_C(2), kTolerance);
      } else {
        ASSERT_EQ(contact_pair.jacobian.size(), 1);
        const Matrix3X<double> J0 =
            contact_pair.jacobian[0].J.MakeDenseMatrix();
        ASSERT_EQ(v0.size(), J0.cols());
        EXPECT_TRUE(CompareMatrices(J0 * v0, expected_v_DpRp_C, kTolerance));
        EXPECT_NEAR(contact_pair.vn0, expected_v_DpRp_C(2), kTolerance);
      }

      // Object A is always the deformable body and B is the rigid body.
      EXPECT_EQ(BodyIndex(contact_pair.object_B), rigid_body_index_);
      const DeformableBodyIndex body_index =
          model_->GetBodyIndex(deformable_body_id_);
      // We expect deformable bodies to follow after rigid bodies.
      const int object_A = body_index + plant_->num_bodies();
      EXPECT_EQ(contact_pair.object_A, object_A);
    }
  }

  /* Verifies fixed constraint kinematics data are as expected.
   @param[in] dynamic_rigid_body  True if a rigid body is registered in the
   setup (instead of a static body with a collision geometry). */
  void ValidateFixedConstraintKinematics(bool dynamic_rigid_body) {
    /* Each discrete contact pair should create a contact kinematics pair. */
    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(*context_);
    std::vector<FixedConstraintKinematics<double>> constraint_kinematics;
    driver_->AppendDeformableRigidFixedConstraintKinematics(
        plant_context, &constraint_kinematics);
    /* Only the bottom vertex of the deformable body in its reference frame is
     constrained. */
    ASSERT_EQ(constraint_kinematics.size(), 1);
    /* Velocities of the participating deformable dofs. */
    const VectorXd v0 = driver_->EvalParticipatingVelocities(plant_context);
    /* The expected constraint velocity, v_DpRq_W, is the velocity of the
     incident point on the rigid body relative to the deformable vertex
     expressed in the world frame. We know that v_DpRq_F is (0, 0, 1) from the
     setup of the scene. */
    const Vector3d v_DpRq_F(0, 0, 1);
    const Vector3d expected_vc = X_WF_.rotation() * v_DpRq_F;
    const FixedConstraintKinematics<double>& constraint_kinematic =
        constraint_kinematics[0];
    if (dynamic_rigid_body) {
      ASSERT_EQ(constraint_kinematic.J.num_cliques(), 2);
      const Matrix3X<double> J0 =
          constraint_kinematic.J.clique_jacobian(0).MakeDenseMatrix();
      ASSERT_EQ(v0.size(), J0.cols());
      const Matrix3X<double> J1 =
          constraint_kinematic.J.clique_jacobian(1).MakeDenseMatrix();
      const Vector3d v_WR_F(0, 0, 0.5);
      const Vector3d v_WR = X_WF_.rotation() * v_WR_F;
      const Vector3d w_WR(0, 0, 0);
      Vector6<double> v1;
      v1 << w_WR, v_WR;
      ASSERT_EQ(v1.size(), J1.cols());
      EXPECT_TRUE(CompareMatrices(J0 * v0 + J1 * v1, expected_vc, kTolerance));
    } else {
      ASSERT_EQ(constraint_kinematic.J.num_cliques(), 1);
      const Matrix3X<double> J0 =
          constraint_kinematic.J.clique_jacobian(0).MakeDenseMatrix();
      ASSERT_EQ(v0.size(), J0.cols());
      EXPECT_TRUE(CompareMatrices(J0 * v0, expected_vc, kTolerance));
    }

    ASSERT_EQ(constraint_kinematic.p_PQs_W.size(), 3);
    EXPECT_EQ(constraint_kinematic.p_PQs_W, Vector3d::Zero());
  }

  void ValidateConstraintParticipation() {
    /* Verifies that all vertices on the bottom side of the deformable
     octahedron participates in constraints. That is, all vertices, with the
     exception of v5, are participating in constraint. */
    const DeformableBodyIndex body_index =
        model_->GetBodyIndex(deformable_body_id_);
    const ContactParticipation& participation =
        driver_->EvalConstraintParticipation(
            plant_->GetMyContextFromRoot(*context_), body_index);
    EXPECT_EQ(participation.num_vertices_in_contact(), 6);
    /*
     |   Original       |   Permuted       |   Participating   |
     |   vertex index   |   vertex index   |   in contact      |
     | :--------------: | :--------------: | :---------------: |
     |        0         |        0         |       yes         |
     |        1         |        1         |       yes         |
     |        2         |        2         |       yes         |
     |        3         |        3         |       yes         |
     |        4         |        4         |       yes         |
     |        5         |        6         |       no          |
     |        6         |        5         |       yes         |
    */
    EXPECT_THAT(participation.CalcVertexPermutation().permutation(),
                testing::ElementsAre(0, 1, 2, 3, 4, 6, 5));
    if (deformable_body_id2_.is_valid()) {
      /* Verifies that all vertices on the top side of the deformable
       octahedron participates in constraints. That is, all vertices, with the
       exception of v6, are participating in constraint. */
      const DeformableBodyIndex body_index2 =
          model_->GetBodyIndex(deformable_body_id2_);
      const ContactParticipation& participation2 =
          driver_->EvalConstraintParticipation(
              plant_->GetMyContextFromRoot(*context_), body_index2);
      EXPECT_EQ(participation.num_vertices_in_contact(), 6);
      /*
       |   Original       |   Permuted       |   Participating   |
       |   vertex index   |   vertex index   |   in contact      |
       | :--------------: | :--------------: | :---------------: |
       |        0         |        0         |       yes         |
       |        1         |        1         |       yes         |
       |        2         |        2         |       yes         |
       |        3         |        3         |       yes         |
       |        4         |        4         |       yes         |
       |        5         |        5         |       yes         |
       |        6         |        6         |       no          |
      */
      EXPECT_THAT(participation2.CalcVertexPermutation().permutation(),
                  testing::ElementsAre(0, 1, 2, 3, 4, 5, 6));
    }
  }

  const math::RigidTransformd X_WF_{RollPitchYawd(1, 2, 3), Vector3d(4, 5, 6)};
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  const DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_;
  DeformableBodyId deformable_body_id_;
  /* A second deformable body is added in the deformable vs. deformable contact
   scene. This id is invalid in the deformable vs. rigid scene. */
  DeformableBodyId deformable_body_id2_;
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

/* Tests deformable vs. rigid contact as well as fixed constraints between
 deformable and rigid bodies. */
TEST_P(DeformableDriverContactKinematicsTest,
       DeformableRigidConstraintKinematics) {
  const bool dynamic = GetParam();
  MakeDeformableRigidScene(dynamic);
  ValidateDeformableRigidContactKinematics(dynamic);
  ValidateFixedConstraintKinematics(dynamic);
  ValidateConstraintParticipation();
}

INSTANTIATE_TEST_SUITE_P(All, DeformableDriverContactKinematicsTest,
                         ::testing::Values(true, false));

/* Tests deformable vs. deformable contact. */
TEST_F(DeformableDriverContactKinematicsTest,
       DeformableDeformableContactKinematics) {
  MakeDeformableDeformableScene();
  ValidateDeformableDeformableContactPairs();
  ValidateConstraintParticipation();
}

/* Tests that disabled deformable bodies do not participate in contact. */
TEST_F(DeformableDriverContactKinematicsTest,
       DisabledDeformableDeformableContactKinematics) {
  /* Build a scene with two deformables in contact, but disabled. */
  MakeDeformableDeformableScene();
  model_->Disable(deformable_body_id_, context_.get());
  model_->Disable(deformable_body_id2_, context_.get());
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);

  /* Validate that placeholder contact participation entries exist for the
   disabled bodies. */
  for (const auto& deformable_id :
       {deformable_body_id_, deformable_body_id2_}) {
    const DeformableBodyIndex body_index = model_->GetBodyIndex(deformable_id);
    const ContactParticipation& participation =
        driver_->EvalConstraintParticipation(plant_context, body_index);
    EXPECT_EQ(participation.num_vertices(),
              model_->GetFemModel(deformable_id).num_nodes());
    EXPECT_EQ(participation.num_vertices_in_contact(), 0);
  }

  /* Validate that the disabled bodies are not under fixed constraints. */
  std::vector<FixedConstraintKinematics<double>> constraint_kinematics;
  driver_->AppendDeformableRigidFixedConstraintKinematics(
      plant_context, &constraint_kinematics);
  EXPECT_EQ(constraint_kinematics.size(), 0);
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
  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();
  DeformableBodyId body_id = RegisterDeformableOctahedron(
      &deformable_model, "deformable", RigidTransformd::Identity());
  /* Put the bottom vertex under bc. */
  deformable_model.SetWallBoundaryCondition(body_id, Vector3d(0, 0, -0.5),
                                            Vector3d(0, 0, 1));

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

  // N.B. Deformables are only supported with the SAP solver.
  // Thus for testing we choose one arbitrary contact approximation that uses
  // the SAP solver.
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  plant.Finalize();
  auto contact_manager = make_unique<CompliantContactManager<double>>();
  const CompliantContactManager<double>* manager = contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(contact_manager));
  const DeformableDriver<double>* driver = manager->deformable_driver();
  ASSERT_NE(driver, nullptr);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const Context<double>& plant_context = plant.GetMyContextFromRoot(*context);
  DiscreteContactData<DiscreteContactPair<double>> contact_pairs;
  driver->AppendDiscreteContactPairs(plant_context, &contact_pairs);
  /* The set of contact points is not empty. */
  EXPECT_GT(contact_pairs.size(), 0);
  const PartialPermutation& vertex_permutation =
      DeformableDriverTest::EvalVertexPermutation(
          *driver, plant_context, deformable_model.GetGeometryId(body_id));
  /* We know that the octahedron created as the coarsest sphere indexes the
   bottom vertex as 6 (see RegisterDeformableOctahedron). */
  const int bottom_vertex_permuted_index = vertex_permutation.permuted_index(6);
  for (int i = 0; i < static_cast<int>(contact_pairs.size()); ++i) {
    const DiscreteContactPair<double>& contact_pair = contact_pairs[i];
    /* The first jacobian entry corresponds to the contribution of the
     deformable clique. */
    const Matrix3X<double>& J = contact_pair.jacobian[0].J.MakeDenseMatrix();
    /* The jacobian isn't completely zero. */
    EXPECT_GT(J.norm(), 0.01);
    /* But the columns corresponding to the vertex under bc are set to zero. */
    EXPECT_TRUE(CompareMatrices(
        J.middleCols<3>(3 * bottom_vertex_permuted_index), Matrix3d::Zero()));
  }
}

/* Tests that DeformableDriver::EvalConstraintParticipation gives the correct
 result when no contact is present. */
GTEST_TEST(DeformableDriverConstraintParticipation, ConstraintWithoutContact) {
  systems::DiagramBuilder<double> builder;
  constexpr double kDt = 0.01;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, kDt);
  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();
  DeformableBodyId body_id = RegisterDeformableOctahedron(
      &deformable_model, "deformable", RigidTransformd::Identity());
  /* Use a large box geometry to ensure that all deformable vertices are placed
   under fixed constraints. */
  deformable_model.AddFixedConstraint(
      body_id, plant.world_body(), RigidTransformd::Identity(),
      geometry::Box(10, 10, 10), RigidTransformd::Identity());
  // N.B. Deformables are only supported with the SAP solver.
  // Thus for testing we choose one arbitrary contact approximation that uses
  // the SAP solver.
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  plant.Finalize();
  auto contact_manager = make_unique<CompliantContactManager<double>>();
  const CompliantContactManager<double>* manager = contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(contact_manager));
  const DeformableDriver<double>* driver = manager->deformable_driver();
  ASSERT_NE(driver, nullptr);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const Context<double>& plant_context = plant.GetMyContextFromRoot(*context);
  const ContactParticipation& participation =
      driver->EvalConstraintParticipation(plant_context,
                                          DeformableBodyIndex(0));
  /* We know that the deformable octahedron has exactly 7 vertices, and they
   should all participate in constraints. */
  EXPECT_EQ(participation.num_vertices_in_contact(), 7);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
