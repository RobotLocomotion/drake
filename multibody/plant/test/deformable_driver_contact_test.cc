#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/fem/matrix_utilities.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::internal::ContactParticipation;
using drake::geometry::internal::DeformableContact;
using drake::geometry::internal::DeformableContactSurface;
using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::PartialPermutation;
using drake::multibody::contact_solvers::internal::SchurComplement;
using drake::multibody::fem::FemModel;
using drake::multibody::fem::FemState;
using drake::systems::Context;
using drake::systems::DiscreteStateIndex;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace multibody {
namespace internal {

/* This fixture tests DeformableDriver member functions associated with the
 concept of contact. In particular, it sets up two identical, non-overlapping
 deformable octahedron bodies, each with 8 elements, 7 vertices, and 21 dofs. A
 rigid box is added so that its top face intersects the bottom half of
 one deformable octahedron and its bottom face intersects the top half of the
 other deformable octahedron. As a result, each deformable body has 6
 participating vertices (all vertices except the top/bottom vertex) and 18
 participating dofs. */
class DeformableDriverContactTest : public ::testing::Test {
 protected:
  static constexpr double kDt = 0.001;
  static constexpr double kDissipationTimeScale = 0.1;
  static constexpr double kHcDampingRigid = 12.3;
  static constexpr double kHcDampingDeformable = 45.6;

  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    /* Move the first deformable up so that the bottom half of it intersects the
     rigid box. */
    const RigidTransformd X_WD0(Vector3d(0, 0, 1.25));
    body_id0_ =
        RegisterDeformableOctahedron(X_WD0, &deformable_model, "deformable0");
    /* Move the second deformable down so that the top half of it intersects the
     rigid box. */
    const RigidTransformd X_WD1(Vector3d(0, 0, -1.25));
    body_id1_ =
        RegisterDeformableOctahedron(X_WD1, &deformable_model, "deformable1");
    model_ = &plant_->deformable_model();
    // N.B. Deformables are only supported with the SAP solver.
    // Thus for testing we choose one arbitrary contact approximation that uses
    // the SAP solver.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);
    /* Register a rigid collision geometry intersecting with the bottom half of
     the deformable octahedrons. */
    geometry::ProximityProperties proximity_prop;
    geometry::AddContactMaterial(kHcDampingRigid, {},
                                 CoulombFriction<double>(1.0, 1.0),
                                 &proximity_prop);
    // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
    //  as the trigger for contact with deformable bodies.
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(), RigidTransformd::Identity(),
        geometry::Box(10, 10, 1), "rigid_collision_geometry", proximity_prop);
    plant_->Finalize();

    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(contact_manager));
    driver_ = manager_->deformable_driver();
    DRAKE_DEMAND(driver_ != nullptr);

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

  const VectorXd& EvalParticipatingFreeMotionVelocities(
      const Context<double>& context) const {
    return driver_->EvalParticipatingFreeMotionVelocities(context);
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

  const Multiplexer<double>& EvalParticipatingVelocityMultiplexer(
      const systems::Context<double>& context) const {
    return driver_->EvalParticipatingVelocityMultiplexer(context);
  }

  MatrixXd EvalFreeMotionTangentMatrix(const systems::Context<double>& context,
                                       DeformableBodyIndex index) const {
    DeformableBodyId body_id = model_->GetBodyId(index);
    const FemModel<double>& fem_model = model_->GetFemModel(body_id);
    std::unique_ptr<contact_solvers::internal::Block3x3SparseSymmetricMatrix>
        fem_tangent_matrix = fem_model.MakeTangentMatrix();
    const FemState<double>& free_motion_state =
        EvalFreeMotionFemState(context, index);
    fem_model.CalcTangentMatrix(free_motion_state, GetIntegratorWeights(),
                                fem_tangent_matrix.get());
    return fem_tangent_matrix->MakeDenseMatrix();
  }

  const SchurComplement& EvalFreeMotionTangentMatrixSchurComplement(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return driver_->EvalFreeMotionTangentMatrixSchurComplement(context, index);
  }

  /* @} */

  /* Sets the velocity of the deformable body with the given id in the member
   context.
   @pre The size of v is compatible with the number of degree of freedom of the
   body with id. */
  void SetVelocities(DeformableBodyId id, const VectorXd& v) {
    Context<double>& plant_context =
        plant_->GetMyMutableContextFromRoot(context_.get());
    const DiscreteStateIndex state_index = model_->GetDiscreteStateIndex(id);
    VectorXd state = plant_context.get_discrete_state(state_index).value();
    DRAKE_DEMAND(state.size() % 3 == 0);
    const int num_dofs = state.size() / 3;
    DRAKE_DEMAND(num_dofs == v.size());
    state.segment(num_dofs, num_dofs) = v;
    plant_context.SetDiscreteState(state_index, state);
  }

  Vector3<double> GetIntegratorWeights() const {
    return driver_->integrator_->GetWeights();
  }

  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  const DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  std::unique_ptr<Context<double>> context_;
  DeformableBodyId body_id0_;
  DeformableBodyId body_id1_;
  GeometryId rigid_geometry_id_;

 private:
  /* Registers a deformable octahedron with 8 vertices with pose `X_WD` to the
   given `model`. */
  DeformableBodyId RegisterDeformableOctahedron(const RigidTransformd& X_WD,
                                                DeformableModel<double>* model,
                                                std::string name) {
    auto geometry = make_unique<GeometryInstance>(
        X_WD, make_unique<Sphere>(1.0), std::move(name));
    geometry::ProximityProperties props;
    geometry::AddContactMaterial(kHcDampingDeformable, {},
                                 CoulombFriction<double>(1.0, 1.0), &props);
    props.AddProperty(geometry::internal::kMaterialGroup,
                      geometry::internal::kRelaxationTime,
                      kDissipationTimeScale);
    geometry->set_proximity_properties(std::move(props));
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(1e6);
    body_config.set_poissons_ratio(0.4);
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId body_id = model->RegisterDeformableBody(
        std::move(geometry), body_config, kRezHint);
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

  /* All but the top/bottom vertex in each octahedron participate in contact. */
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
  const int num_dofs = model_->GetFemModel(body_id0_).num_dofs();
  const auto v0 = VectorXd::Zero(num_dofs);
  const auto v1 = VectorXd::Ones(num_dofs);
  SetVelocities(body_id0_, v0);
  SetVelocities(body_id1_, v1);

  const int num_participating_vertices = 6;
  const int num_participating_dofs_per_body = num_participating_vertices * 3;
  /* Verify that the participating velocities are multiplexed correctly. */
  VectorXd expected_participating_velocity(2 * num_participating_dofs_per_body);
  expected_participating_velocity
      << VectorXd::Zero(num_participating_dofs_per_body),
      VectorXd::Ones(num_participating_dofs_per_body);
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  EXPECT_EQ(EvalParticipatingVelocities(plant_context),
            expected_participating_velocity);
}

TEST_F(DeformableDriverContactTest, EvalParticipatingFreeMotionVelocities) {
  /* Set states for both bodies so that they have different velocities. */
  const int num_dofs = model_->GetFemModel(body_id0_).num_dofs();
  const auto v0 = VectorXd::LinSpaced(num_dofs, 0.0, 1.0);
  const auto v1 = VectorXd::LinSpaced(num_dofs, 1.0, 2.0);
  SetVelocities(body_id0_, v0);
  SetVelocities(body_id1_, v1);

  /* Here we rely on the correctness of EvalDofPermutation and
   EvalFreeMotionFemState which are tested separately in this fixture and in
   DeformableDriverTest respectively. */
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const VectorXd& v0_star =
      EvalFreeMotionFemState(plant_context, DeformableBodyIndex(0))
          .GetVelocities();
  const VectorXd& v1_star =
      EvalFreeMotionFemState(plant_context, DeformableBodyIndex(1))
          .GetVelocities();
  const PartialPermutation& p0 =
      EvalDofPermutation(plant_context, DeformableBodyIndex(0));
  const PartialPermutation& p1 =
      EvalDofPermutation(plant_context, DeformableBodyIndex(1));
  VectorXd participating_v0_star(p0.permuted_domain_size());
  VectorXd participating_v1_star(p1.permuted_domain_size());
  p0.Apply(v0_star, &participating_v0_star);
  p1.Apply(v1_star, &participating_v1_star);
  const int num_participating_vertices = 6;
  const int num_participating_dofs_per_body = num_participating_vertices * 3;
  VectorXd expected_participating_v_star(2 * num_participating_dofs_per_body);
  expected_participating_v_star << participating_v0_star, participating_v1_star;
  EXPECT_TRUE(
      CompareMatrices(expected_participating_v_star,
                      EvalParticipatingFreeMotionVelocities(plant_context)));
}

TEST_F(DeformableDriverContactTest,
       EvalFreeMotionTangentMatrixSchurComplement) {
  DeformableBodyIndex body_index(0);
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const MatrixXd tangent_matrix =
      EvalFreeMotionTangentMatrix(plant_context, body_index);
  /* Schematically the participating block (A), the non-participating block (D),
   and the off-diagonal block (B) look like
                           _______________________________________
                           |                       |      |      |
   Dofs 0-14 (associated   |                       |      |      |
   with vertices 0-4) are  |                       |      |      |
   participating.          |                       |      |      |
                           |                       |      |      |
                           |           A           |  Bᵀ  |   A  |
                           |                       |      |      |
                           |                       |      |      |
                           |                       |      |      |
                           |                       |      |      |
                           |_______________________|______|______|
   Dofs 15-17 (associated  |                       |      |      |
   with vertex 5) are not  |           B           |   D  |   B  |
   participating.          |_______________________|______|______|
   Dofs 18-20 (associated  |                       |      |      |
   with vertex 6) are      |           A           |  Bᵀ  |   A  |
   participating.          |_______________________|______|______|       */
  /* Matrix block for participating dofs. */
  const int num_participating_vertices = 6;
  const int num_participating_dofs = num_participating_vertices * 3;
  MatrixXd A = MatrixXd::Zero(num_participating_dofs, num_participating_dofs);
  /* Vertices 0, 1, 2, 3, 4, 6 are participating in contact. */
  A.topLeftCorner(15, 15) = tangent_matrix.topLeftCorner(15, 15);
  A.topRightCorner(15, 3) = tangent_matrix.topRightCorner(15, 3);
  A.bottomLeftCorner(3, 15) = tangent_matrix.bottomLeftCorner(3, 15);
  A.bottomRightCorner(3, 3) = tangent_matrix.bottomRightCorner(3, 3);
  /* Matrix block for non-participating dofs. */
  const int num_nonparticipating_vertices = 1;
  const int num_nonparticipating_dofs = num_nonparticipating_vertices * 3;
  MatrixXd D = tangent_matrix.block<3, 3>(15, 15);
  /* Off diagonal block. */
  MatrixXd B =
      MatrixXd::Zero(num_nonparticipating_dofs, num_participating_dofs);
  B.leftCols(15) = tangent_matrix.block<3, 15>(15, 0);
  B.rightCols(3) = tangent_matrix.block<3, 3>(15, 18);
  const MatrixXd expected_complement_matrix =
      A - B.transpose() * D.llt().solve(B);
  EXPECT_TRUE(CompareMatrices(
      expected_complement_matrix,
      EvalFreeMotionTangentMatrixSchurComplement(plant_context, body_index)
          .get_D_complement(),
      1e-10));
}

TEST_F(DeformableDriverContactTest, AppendLinearDynamicsMatrix) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  std::vector<MatrixXd> A;
  driver_->AppendLinearDynamicsMatrix(plant_context, &A);
  ASSERT_EQ(A.size(), 2);
  DeformableBodyIndex body_index0(0);
  DeformableBodyIndex body_index1(1);
  EXPECT_EQ(A[0], EvalFreeMotionTangentMatrixSchurComplement(plant_context,
                                                             body_index0)
                          .get_D_complement() *
                      plant_->time_step());
  EXPECT_EQ(A[1], EvalFreeMotionTangentMatrixSchurComplement(plant_context,
                                                             body_index1)
                          .get_D_complement() *
                      plant_->time_step());
}

TEST_F(DeformableDriverContactTest, AppendLinearDynamicsMatrixLocked) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  std::vector<MatrixXd> A;
  driver_->AppendLinearDynamicsMatrix(plant_context, &A);
  /* Confirm the bodies, when unlocked, have non-trivial linear dynamics
   matrices. */
  ASSERT_EQ(A.size(), 2);
  EXPECT_NE(A[0].size(), 0);
  EXPECT_NE(A[1].size(), 0);

  /* With body0 locked, ensure that the dynamics matrix for body0 is empty. */
  model_->Lock(body_id0_, context_.get());

  A.clear();
  driver_->AppendLinearDynamicsMatrix(plant_context, &A);
  ASSERT_EQ(A.size(), 2);
  EXPECT_EQ(A[0].rows(), 0);
  EXPECT_EQ(A[0].cols(), 0);
  /* The linear dynamics matrix for body1 remains unchanged. */
  DeformableBodyIndex body_index1(1);
  EXPECT_EQ(A[1], EvalFreeMotionTangentMatrixSchurComplement(plant_context,
                                                             body_index1)
                          .get_D_complement() *
                      plant_->time_step());

  /* With all bodies locked, ensure the linear dynamic matrices for both bodies
   are empty. */
  model_->Lock(body_id1_, context_.get());
  A.clear();
  driver_->AppendLinearDynamicsMatrix(plant_context, &A);
  ASSERT_EQ(A.size(), 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(A[i].rows(), 0);
    EXPECT_EQ(A[i].cols(), 0);
  }
}

TEST_F(DeformableDriverContactTest, AppendDiscreteContactPairs) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  DiscreteContactData<DiscreteContactPair<double>> contact_pairs;
  driver_->AppendDiscreteContactPairs(plant_context, &contact_pairs);

  const DeformableContact<double>& contact_data =
      EvalDeformableContact(plant_context);
  int num_contact_points = 0;
  for (const DeformableContactSurface<double>& surface :
       contact_data.contact_surfaces()) {
    num_contact_points += surface.num_contact_points();
  }
  EXPECT_GT(num_contact_points, 0);
  EXPECT_EQ(contact_pairs.size(), num_contact_points);
  /* tau for deformable body is set to kDissipationTimeScale and is unset for
   rigid body (which then assumes the default value, dt). */
  constexpr double expected_tau = kDissipationTimeScale + kDt;
  /* The H&C damping is set to be d = k₂/(k₁+k₂)⋅d₁ + k₁/(k₁+k₂)⋅d₂. In this
   case, the stiffness of the rigid body defaults to infinity so the damping
   value takes the mathematical limit in that expression, i.e. the damping
   value of the deformable body. */
  constexpr double expected_d = kHcDampingDeformable;

  GeometryId id0 = model_->GetGeometryId(body_id0_);
  GeometryId id1 = model_->GetGeometryId(body_id1_);

  /* The set of face indices for each contact surface should be
   {0, 1, ..., num_contact_points - 1}. */
  std::set<int> face_indices_0;
  std::set<int> face_indices_1;
  std::set<int> expected_face_indices;
  for (int i = 0; i < contact_data.contact_surfaces()[0].num_contact_points();
       ++i) {
    expected_face_indices.insert(i);
  }

  for (int i = 0; i < contact_pairs.size(); ++i) {
    const DiscreteContactPair<double>& pair = contact_pairs[i];
    EXPECT_TRUE(pair.id_A == id0 || pair.id_A == id1);
    EXPECT_EQ(pair.id_B, rigid_geometry_id_);
    /* The contact points are on the z = -0.5 and z = 0.5 planes, the top and
     bottom surfaces of the rigid box. */
    if (pair.id_A == id0) {
      EXPECT_EQ(pair.p_WC(2), 0.5);
      EXPECT_TRUE(CompareMatrices(pair.nhat_BA_W, Eigen::Vector3d(0, 0, 1)));
    } else {
      EXPECT_EQ(pair.p_WC(2), -0.5);
      EXPECT_TRUE(CompareMatrices(pair.nhat_BA_W, Eigen::Vector3d(0, 0, -1)));
    }
    EXPECT_EQ(pair.damping, expected_d);
    /* Verify that the stiffness and the normal contact force are compatible. */
    EXPECT_EQ(pair.fn0, -pair.stiffness * pair.phi0);
    /* Verify the sign of the normal contact force. It should be repulsive. */
    EXPECT_GT(pair.fn0, 0.0);
    EXPECT_EQ(pair.dissipation_time_scale, expected_tau);
    EXPECT_EQ(pair.friction_coefficient, 1.0);
    ASSERT_TRUE(pair.surface_index.has_value());
    ASSERT_TRUE(pair.face_index.has_value());
    /* There are two contact surfaces, one between the rigid box and the 0-th
     deformable octahedron, and the other between the rigid box and the 1st
     deformable octahedron. We don't know the order they come into
     `contact_pairs`, but we do know the surface index is either 0 or 1. */
    EXPECT_THAT(pair.surface_index.value(), testing::AnyOf(0, 1));
    if (pair.id_A == id0) {
      face_indices_0.insert(pair.face_index.value());
    } else {
      face_indices_1.insert(pair.face_index.value());
    }
  }
  EXPECT_EQ(face_indices_0, expected_face_indices);
  EXPECT_EQ(face_indices_1, expected_face_indices);
}

/* Test that locked deformable bodies are ignored when computing contact pairs.
 */
TEST_F(DeformableDriverContactTest, AppendDiscreteContactPairsLocked) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);

  /* Test all combinations of deformable contact pairs where zero, one, or both
   of the bodies are locked. */
  std::vector<std::vector<std::pair<DeformableBodyId, bool>>> test_cases = {
      {{body_id0_, false}, {body_id1_, false}},
      {{body_id0_, false}, {body_id1_, true}},
      {{body_id0_, true}, {body_id1_, false}},
      {{body_id0_, true}, {body_id1_, true}},
  };

  for (const auto& test_case : test_cases) {
    const DeformableContact<double>& contact_data =
        EvalDeformableContact(plant_context);

    /* Apply locking/unlocking per the test case configuration. Compute the
     number of expected contacts from the unlocked body count. */
    int num_contact_points = 0;
    for (const auto& [deformable_id, is_locked] : test_case) {
      if (is_locked) {
        model_->Lock(deformable_id, context_.get());
      } else {
        model_->Unlock(deformable_id, context_.get());
      }

      /* Compute this model's contribution to the expected contact point count,
        disregarding locked models. */
      for (const DeformableContactSurface<double>& surface :
           contact_data.contact_surfaces()) {
        const GeometryId geom_id = model_->GetGeometryId(deformable_id);
        if (surface.id_A() == geom_id || surface.id_B() == geom_id) {
          num_contact_points += is_locked ? 0 : surface.num_contact_points();
        }
      }
    }

    DiscreteContactData<DiscreteContactPair<double>> contact_pairs;
    driver_->AppendDiscreteContactPairs(plant_context, &contact_pairs);

    EXPECT_GE(num_contact_points, 0);
    EXPECT_EQ(contact_pairs.size(), num_contact_points);
  }
}

/* Verifies that the post contact velocites for deformable bodies are as
 expected. In particular, verify that A * dv = Jᵀγ is satisfied where A is the
 tangent matrix for deformable bodies evaluated at free motion state,
 dv = v - v*, and Jᵀγ is the generalized contact impulse. */
TEST_F(DeformableDriverContactTest, CalcNextFemStateWithContact) {
  /* Set states for both bodies so that they have non-trivial initial
   velocities. */
  const int num_dofs = model_->GetFemModel(body_id0_).num_dofs();
  const auto v0 = VectorXd::LinSpaced(num_dofs, 0.0, 1.0);
  SetVelocities(body_id0_, v0);

  /* Build v*. */
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const VectorXd& v_star =
      EvalFreeMotionFemState(plant_context, DeformableBodyIndex(0))
          .GetVelocities();

  /* Build v. */
  const VectorXd& v_next =
      EvalNextFemState(plant_context, DeformableBodyIndex(0)).GetVelocities();

  /* Build A, the tangent matrix of the momentum balance equation (thus the
   factor of dt). */
  const MatrixXd A =
      EvalFreeMotionTangentMatrix(plant_context, DeformableBodyIndex(0)) *
      plant_->time_step();

  /* Build tau. */
  const ContactSolverResults<double>& solver_results =
      manager_->EvalContactSolverResults(plant_context);
  const Multiplexer<double>& mux =
      EvalParticipatingVelocityMultiplexer(plant_context);
  const VectorXd participating_tau =
      mux.Demultiplex(solver_results.tau_contact, 0);
  const int num_participating_dofs = participating_tau.size();
  const int num_nonparticipating_dofs = num_dofs - num_participating_dofs;
  const VectorXd non_participating_tau =
      VectorXd::Zero(num_nonparticipating_dofs);
  VectorXd permuted_tau(num_dofs);
  permuted_tau << participating_tau, non_participating_tau;
  const DeformableContact<double>& contact_data =
      EvalDeformableContact(plant_context);
  const GeometryId geometry_id = model_->GetGeometryId(body_id0_);
  const ContactParticipation& participation =
      contact_data.contact_participation(geometry_id);
  const PartialPermutation& full_dof_permutation =
      participation.CalcDofPermutation();
  VectorXd tau(num_dofs);
  full_dof_permutation.ApplyInverse(permuted_tau, &tau);
  /* The convergence criterion for the SAP solver is
     ‖A⋅(v−v*)−Jᵀγ‖ = ‖∇ℓ‖ < εₐ + εᵣ max(‖A⋅v‖,‖jc‖),
   where ‖x‖ = ‖D⋅x‖₂, where D = diag(A)^(-1/2). Usually, the relative tolerance
   condition is triggered, that is,
     ‖A⋅(v−v*)−Jᵀγ‖ < εᵣ‖D⋅A⋅v‖₂. */
  const VectorXd D = A.diagonal().cwiseInverse().cwiseSqrt();
  const double scale = (D.asDiagonal() * (A * v_next)).norm();
  const double relative_tolerance = 1e-6;
  EXPECT_LT((D.asDiagonal() * (A * (v_next - v_star) - tau * kDt)).norm(),
            scale * relative_tolerance);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
