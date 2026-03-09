#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::Box;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
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
constexpr double kDissipationTimeScale = 0.123;
constexpr double kHcDamping = 12.3;

/* Register a deformable body with the given geometry instance and world pose
 into the given `model`. */
DeformableBodyId RegisterDeformableBody(
    DeformableModel<double>* model, std::unique_ptr<GeometryInstance> instance,
    const RigidTransformd& X_WF) {
  geometry::ProximityProperties props;
  geometry::AddContactMaterial({}, {}, CoulombFriction<double>(kMu, kMu),
                               &props);
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kHcDissipation, kHcDamping);
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kRelaxationTime, kDissipationTimeScale);
  instance->set_proximity_properties(std::move(props));
  fem::DeformableBodyConfig<double> body_config;
  /* Make the resolution hint large enough so that we get an octahedron. */
  constexpr double kRezHint = 10.0;
  DeformableBodyId body_id =
      model->RegisterDeformableBody(std::move(instance), body_config, kRezHint);
  return body_id;
}

/* Registers a deformable box with the given `name` and pose in the
 world to the given `model`.
 The box in its reference configuration is an axis-aligned unit cube centered at
 the origin of the F frame. */
DeformableBodyId RegisterDeformableBox(DeformableModel<double>* model,
                                       std::string name,
                                       const RigidTransformd& X_WF) {
  const std::string box =
      FindResourceOrThrow("drake/multibody/plant/test/box.vtk");
  auto box_mesh = std::make_unique<Mesh>(box, 1.0);
  auto box_instance = std::make_unique<GeometryInstance>(
      X_WF, std::move(box_mesh), std::move(name));
  return RegisterDeformableBody(model, std::move(box_instance), X_WF);
}

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
  return RegisterDeformableBody(model, std::move(geometry), X_WF);
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

  /* Sets up a unit deformable cube centered at the origin of frame F. To test
   the contact kinematics between deformable and rigid bodies, we set up two
   rigid geometries, one attached to a dynamic rigid body and the other attached
   to world, but both in contact with the deformable geometry. */
  void MakeDeformableRigidScene() {
    systems::DiagramBuilder<double> builder;
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    deformable_body_id_ =
        RegisterDeformableBox(&deformable_model, "deformable", X_WF_);
    model_ = &plant_->deformable_model();

    /* Add a dynamic rigid body with the same pose as the deformable body. */
    const RigidBody<double>& rigid_body =
        plant_->AddRigidBody("rigid_body", SpatialInertia<double>::NaN());
    plant_->SetDefaultFloatingBaseBodyPose(rigid_body, X_WF_);
    rigid_body_index_ = rigid_body.index();

    /* Register the collision geometry so that it intersects with the top of
     the deformable box (25% of the rigid box is inside the deformable box). One
     for the dynamic rigid body, one for the world. */
    ProximityProperties rigid_proximity_props;
    plant_->RegisterCollisionGeometry(
        rigid_body, RigidTransformd(Vector3d(0, 0, 0.525)), Box(0.1, 0.1, 0.1),
        "dynamic", rigid_proximity_props);
    const RigidTransformd X_WG = X_WF_ * RigidTransformd(Vector3d(0, 0, 0.525));
    plant_->RegisterCollisionGeometry(plant_->world_body(), X_WG,
                                      Box(0.1, 0.1, 0.1), "fixed",
                                      rigid_proximity_props);

    plant_->Finalize();
    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(contact_manager));
    driver_ = manager_->deformable_driver();
    DRAKE_DEMAND(driver_ != nullptr);

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();

    /* Given the geometries in contact opposing velocities. */
    const Vector3d v_FD = Vector3d(0, 0, 1);
    const Vector3d v_WD = X_WF_.rotation() * v_FD;
    SetVelocity(deformable_body_id_, v_WD);
    Context<double>& mutable_plant_context =
        plant_->GetMyMutableContextFromRoot(context_.get());
    /* The velocity of the rigid body. */
    const Vector3d v_FR = Vector3d(0, 0, -1);
    const SpatialVelocity<double> V_WR(Vector3d::Zero(),
                                       X_WF_.rotation() * v_FR);
    plant_->SetFreeBodySpatialVelocity(
        &mutable_plant_context, plant_->get_body(rigid_body_index_), V_WR);
  }

  /* Sets up a deformable octahedron centered at the origin of frame F with 8
   elements, 7 vertices, and 21 dofs. To test fixed constraint kinematics, we
   constrain the top vertex to a dynamic rigid body and the bottom vertex to the
   world (to provide coverage for both the case with one and two Jacobian
   blocks). */
  void MakeFixedConstraintScene() {
    systems::DiagramBuilder<double> builder;
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    deformable_body_id_ =
        RegisterDeformableOctahedron(&deformable_model, "deformable", X_WF_);
    model_ = &plant_->deformable_model();

    /* Add a dynamic rigid body with the same pose as the deformable body. */
    const RigidBody<double>& rigid_body =
        plant_->AddRigidBody("rigid_body", SpatialInertia<double>::NaN());
    plant_->SetDefaultFloatingBaseBodyPose(rigid_body, X_WF_);
    rigid_body_index_ = rigid_body.index();

    /* Add a fixed constraint between the rigid body and the deformable body.
     Set up the geometry's pose in the rigid body so that only the top vertex of
     the deformable octahedron in its reference frame is fixed. */
    const RigidTransformd X_BG(Vector3d(0, 0, 1));
    deformable_model.AddFixedConstraint(
        deformable_body_id_, plant_->get_body(rigid_body_index_),
        RigidTransformd::Identity(), Box(10, 10, 1), X_BG);
    /* Add a fixed constraint between deformable body and the world. Fix only
     the bottom vertex of the deformable octahedron in its reference frame.*/
    const RigidTransformd X_FG(Vector3d(0, 0, -1));
    deformable_model.AddFixedConstraint(deformable_body_id_,
                                        plant_->world_body(), X_WF_,
                                        Box(10, 10, 1), X_WF_ * X_FG);
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
    /* The rigid body can move, so we'll achieve the desired relative velocity
     by splitting it evenly between the rigid and deformable bodies. */
    const Vector3d v_FD = Vector3d(0, 0, -1);
    const Vector3d v_WD = X_WF_.rotation() * v_FD;
    SetVelocity(deformable_body_id_, v_WD);
    Context<double>& mutable_plant_context =
        plant_->GetMyMutableContextFromRoot(context_.get());
    /* The velocity of the rigid body. */
    const Vector3d v_FR = Vector3d(0, 0, -2);
    const SpatialVelocity<double> V_WR(Vector3d::Zero(),
                                       X_WF_.rotation() * v_FR);
    plant_->SetFreeBodySpatialVelocity(
        &mutable_plant_context, plant_->get_body(rigid_body_index_), V_WR);
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
    MultibodyPlantConfig config{.time_step = kDt};
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
   scene are as expected.
   @param[in] expect_nonzero_velocity  True if the participating deformable dof
   velocities should be verified against their expected values from the scene
   construction, or false if they are expected to be zero.
   */
  void ValidateDeformableDeformableContactKinematics(
      bool expect_nonzero_velocity) {
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
    const Vector3d expected_v_D1D2_C =
        expect_nonzero_velocity ? Vector3d(0, 0, -1) : Vector3d::Zero();
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
       friction coefficient).
       See implementation notes for why this is the expected stiffness. */
      const double expected_k = 1e8 * contact_surface.contact_mesh_W().area(i);
      EXPECT_DOUBLE_EQ(contact_pair.stiffness, expected_k);
      EXPECT_EQ(contact_pair.damping, kHcDamping);
      EXPECT_EQ(contact_pair.dissipation_time_scale,
                2.0 * kDissipationTimeScale);
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
   are as expected. */
  void ValidateDeformableRigidContactKinematics() {
    /* Each discrete contact pair should create a contact kinematics pair. */
    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(*context_);
    DiscreteContactData<DiscreteContactPair<double>> contact_pairs;
    driver_->AppendDiscreteContactPairs(plant_context, &contact_pairs);
    const int num_contact_points = GetNumContactPoints(plant_context);
    ASSERT_EQ(contact_pairs.size(), num_contact_points);

    /* The contact surfaces are parallel to the xy-plane in frame F and the
     contact normal points from the deformable body into the rigid body. */
    const Vector3d nhat_F(0, 0, 1);
    const Vector3d nhat_W = X_WF_.rotation() * nhat_F;
    constexpr int kZAxis = 2;
    const math::RotationMatrixd expected_R_WC =
        math::RotationMatrixd::MakeFromOneUnitVector(nhat_W, kZAxis);
    /* Velocities of the participating deformable dofs. */
    const VectorXd v_deformable =
        driver_->EvalParticipatingVelocities(plant_context);
    for (int i = 0; i < num_contact_points; ++i) {
      const DiscreteContactPair<double>& contact_pair = contact_pairs[i];
      EXPECT_LT(contact_pair.phi0, 0.0);
      EXPECT_TRUE(contact_pair.R_WC.IsNearlyEqualTo(expected_R_WC, 1e-12));
      bool dynamic_rigid_body = contact_pair.jacobian.size() > 1;
      if (dynamic_rigid_body) {
        ASSERT_EQ(contact_pair.jacobian.size(), 2);
        const Matrix3X<double> J0 =
            contact_pair.jacobian[0].J.MakeDenseMatrix();
        ASSERT_EQ(v_deformable.size(), J0.cols());
        const Matrix3X<double> J1 =
            contact_pair.jacobian[1].J.MakeDenseMatrix();

        VectorX<double> v_rigid = plant_->GetVelocities(plant_context);
        ASSERT_EQ(v_rigid.size(), J1.cols());
        const Vector3d expected_vc(0, 0, -2);
        EXPECT_TRUE(CompareMatrices(J0 * v_deformable + J1 * v_rigid,
                                    expected_vc, kTolerance));
        EXPECT_NEAR(contact_pair.vn0, expected_vc(2), kTolerance);
        /* Object B is the rigid body. */
        EXPECT_EQ(BodyIndex(contact_pair.object_B), rigid_body_index_);
      } else {
        ASSERT_EQ(contact_pair.jacobian.size(), 1);
        const Matrix3X<double> J0 =
            contact_pair.jacobian[0].J.MakeDenseMatrix();
        ASSERT_EQ(v_deformable.size(), J0.cols());
        const Vector3d expected_vc(0, 0, -1);
        EXPECT_TRUE(
            CompareMatrices(J0 * v_deformable, expected_vc, kTolerance));
        EXPECT_NEAR(contact_pair.vn0, expected_vc(2), kTolerance);
        /* Object B is the rigid body (world in this case). */
        EXPECT_EQ(BodyIndex(contact_pair.object_B), world_index());
      }

      /* Object A is always the deformable body. */
      const DeformableBodyIndex body_index =
          model_->GetBodyIndex(deformable_body_id_);
      /* We expect deformable bodies to follow after rigid bodies. */
      const int object_A = body_index + plant_->num_bodies();
      EXPECT_EQ(contact_pair.object_A, object_A);
    }
  }

  /* Verifies fixed constraint kinematics data are as expected. */
  void ValidateFixedConstraintKinematics() {
    /* Each discrete contact pair should create a contact kinematics pair. */
    const Context<double>& plant_context =
        plant_->GetMyContextFromRoot(*context_);
    std::vector<FixedConstraintKinematics<double>> constraint_kinematics;
    driver_->AppendDeformableRigidFixedConstraintKinematics(
        plant_context, &constraint_kinematics);
    /* Only the bottom and the top vertex of the deformable body in its
     reference frame are constrained. */
    ASSERT_EQ(constraint_kinematics.size(), 2);
    /* Velocities of the participating deformable dofs (the generalized
     velocities). */
    const VectorXd v_deformable =
        driver_->EvalParticipatingVelocities(plant_context);
    const FixedConstraintKinematics<double>& constraint_kinematic0 =
        constraint_kinematics[0];
    const FixedConstraintKinematics<double>& constraint_kinematic1 =
        constraint_kinematics[1];
    /* The first constraint is with the dynamic rigid body so there are two
     cliques in the Jacobian. */
    ASSERT_EQ(constraint_kinematic0.J.num_cliques(), 2);
    const Matrix3X<double> J00 =
        constraint_kinematic0.J.clique_jacobian(0).MakeDenseMatrix();
    const Matrix3X<double> J01 =
        constraint_kinematic0.J.clique_jacobian(1).MakeDenseMatrix();
    const int num_deformable_dofs = J00.cols();
    ASSERT_EQ(v_deformable.size(), num_deformable_dofs);
    /* The generalized velocity of the rigid dofs. */
    VectorX<double> v_rigid = plant_->GetVelocities(plant_context);

    const int num_rigid_dofs = v_rigid.size();
    ASSERT_EQ(J01.cols(), num_rigid_dofs);
    const int num_total_dofs = num_deformable_dofs + num_rigid_dofs;
    /* The generalized v of the entire system. */
    VectorX<double> v(num_total_dofs);
    v << v_deformable, v_rigid;

    /* The second constraint is with the world so there is only one clique in
     the Jacobian. */
    ASSERT_EQ(constraint_kinematic1.J.num_cliques(), 1);
    const Matrix3X<double> J10 =
        constraint_kinematic1.J.clique_jacobian(0).MakeDenseMatrix();
    ASSERT_EQ(num_deformable_dofs, J10.cols());
    MatrixX<double> J = MatrixX<double>::Zero(6, num_total_dofs);
    J.topLeftCorner(3, num_deformable_dofs) = J00;
    J.topRightCorner(3, num_rigid_dofs) = J01;
    J.bottomLeftCorner(3, num_deformable_dofs) = J10;
    VectorXd vc = J * v;
    ASSERT_EQ(vc.size(), 6);
    /* The constraint velocity in the F frame between the dynamic rigid body and
     the deformable body is v_FR - v_FD = (0, 0, -2) - (0, 0, -1) = (0, 0, -1).
    */
    const Vector3d expected_vc0 = X_WF_.rotation() * Vector3d(0, 0, -1);
    EXPECT_TRUE(CompareMatrices(vc.head<3>(), expected_vc0, kTolerance));
    /* The constraint velocity in the F frame between the world and the
     deformable body is -v_FD = (0, 0, 1). */
    const Vector3d expected_vc1 = X_WF_.rotation() * Vector3d(0, 0, 1);
    EXPECT_TRUE(CompareMatrices(vc.tail<3>(), expected_vc1, kTolerance));
  }

  void ValidateConstraintParticipation(
      DeformableBodyId body_id, int num_vertices_in_contact,
      const std::vector<int>& expected_permutation) {
    const DeformableBodyIndex body_index = model_->GetBodyIndex(body_id);
    const ContactParticipation& participation =
        driver_->EvalConstraintParticipation(
            plant_->GetMyContextFromRoot(*context_), body_index);
    EXPECT_EQ(participation.num_vertices_in_contact(), num_vertices_in_contact);
    PartialPermutation full_permutation =
        participation.CalcPartialPermutation().vertex();
    full_permutation.ExtendToFullPermutation();
    EXPECT_EQ(full_permutation.permutation(), expected_permutation);
  }

  const RigidTransformd X_WF_{RollPitchYawd(1, 2, 3), Vector3d(4, 5, 6)};
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

/* Tests contact constraints between deformable and rigid bodies. */
TEST_F(DeformableDriverContactKinematicsTest,
       DeformableRigidContactKinematics) {
  MakeDeformableRigidScene();
  ValidateDeformableRigidContactKinematics();
  /* For the box mesh, v0-v3 are on the bottom face (z < 0) and v4-v7 are on the
     top face (z > 0).

     |   Original       |   Permuted       |   Participating   |
     |   vertex index   |   vertex index   |   in contact      |
     | :--------------: | :--------------: | :---------------: |
     |        0         |        4         |       no          |
     |        1         |        5         |       no          |
     |        2         |        6         |       no          |
     |        3         |        7         |       no          |
     |        4         |        0         |       yes         |
     |        5         |        1         |       yes         |
     |        6         |        2         |       yes         |
     |        7         |        3         |       yes         |
  */
  ValidateConstraintParticipation(deformable_body_id_, 4,
                                  std::vector<int>{4, 5, 6, 7, 0, 1, 2, 3});
}

/* Tests fixed constraints between deformable and rigid bodies. */
TEST_F(DeformableDriverContactKinematicsTest, FixedConstraintKinematics) {
  MakeFixedConstraintScene();
  ValidateFixedConstraintKinematics();
  /* The only vertices participating in the fixed constraint are the top and
   the bottom vertices (v5 and v6).
     |   Original       |   Permuted       |   Participating   |
     |   vertex index   |   vertex index   |   in contact      |
     | :--------------: | :--------------: | :---------------: |
     |        0         |        2         |       no          |
     |        1         |        3         |       no          |
     |        2         |        4         |       no          |
     |        3         |        5         |       no          |
     |        4         |        6         |       no          |
     |        5         |        0         |       yes         |
     |        6         |        1         |       yes         |
  */
  ValidateConstraintParticipation(deformable_body_id_, 2,
                                  std::vector<int>{2, 3, 4, 5, 6, 0, 1});
}

/* Tests deformable vs. deformable contact. */
TEST_F(DeformableDriverContactKinematicsTest,
       DeformableDeformableContactKinematics) {
  MakeDeformableDeformableScene();
  ValidateDeformableDeformableContactKinematics(true);
  /* For the deformable body on top (with `deformable_body_id_`), all vertices
     except the one at the top (v5) are participating in contact.

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
  ValidateConstraintParticipation(deformable_body_id_, 6,
                                  std::vector<int>{0, 1, 2, 3, 4, 6, 5});
  /* For the deformable body at the bottom (with `deformable_body_id2_`), all
     vertices except the one at the bottom (v6) are participating in contact.

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
  ValidateConstraintParticipation(deformable_body_id2_, 6,
                                  std::vector<int>{0, 1, 2, 3, 4, 5, 6});
}

/* Tests that disabled deformable bodies do not participate in contact. */
TEST_F(DeformableDriverContactKinematicsTest,
       DisabledDeformableDeformableContactKinematics) {
  /* Build a scene with two deformables in contact, but disabled. */
  MakeDeformableDeformableScene();
  Context<double>& mutable_plant_context =
      plant_->GetMyMutableContextFromRoot(context_.get());
  model_->Disable(deformable_body_id_, &mutable_plant_context);
  model_->Disable(deformable_body_id2_, &mutable_plant_context);
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

  /* Validate that after re-enabling the bodies, they behave as if they were
   never disabled except that their participating vertex velocities are set to
   zero. */
  model_->Enable(deformable_body_id_, &mutable_plant_context);
  model_->Enable(deformable_body_id2_, &mutable_plant_context);
  ValidateDeformableDeformableContactKinematics(false);
  /* This is the same test as the deformable deformable case. Refer to the
   DeformableDeformableContactKinematics test for why these are the expected
   values. */
  ValidateConstraintParticipation(deformable_body_id_, 6,
                                  std::vector<int>{0, 1, 2, 3, 4, 6, 5});
  ValidateConstraintParticipation(deformable_body_id2_, 6,
                                  std::vector<int>{0, 1, 2, 3, 4, 5, 6});
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
  plant.RegisterCollisionGeometry(plant.world_body(), X_WR, Box(10, 10, 1),
                                  "static_collision_geometry",
                                  rigid_proximity_props);

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
      body_id, plant.world_body(), RigidTransformd::Identity(), Box(10, 10, 10),
      RigidTransformd::Identity());
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
