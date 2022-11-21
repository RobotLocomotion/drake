#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/fem/matrix_utilities.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
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
using drake::multibody::fem::FemModel;
using drake::multibody::fem::FemState;
using drake::multibody::fem::internal::PetscSymmetricBlockSparseMatrix;
using drake::multibody::fem::internal::SchurComplement;
using drake::systems::Context;
using drake::systems::DiscreteStateIndex;
using Eigen::MatrixXd;
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

/* This fixture tests DeformableDriver member functions associated with the
 concept of contact. In particular, it sets up two identical and overlapping
 deformable octahedron bodies centered at world origin, each with 8 elements,
 7 vertices, and 21 dofs. A rigid rectangle is added so that its top face
 intersects the bottom half of each deformable octahedron. As a result, each
 deformable body has 6 participating vertices (all vertices except the top
 vertex) and 18 participating dofs. */
class DeformableDriverContactTest : public ::testing::Test {
 protected:
  static constexpr double kDt = 0.001;
  static constexpr double kPointContactStiffness = 1e6;
  static constexpr double kDissipationTimeScale = 0.1;

  void SetUp() override {
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
    geometry::AddContactMaterial({}, kPointContactStiffness,
                                 CoulombFriction<double>(1.0, 1.0),
                                 &proximity_prop);
    // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
    //  as the trigger for contact with deformable bodies.
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    RigidTransformd X_WG(Vector3<double>(0, 0, -0.75));
    rigid_geometry_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_WG, geometry::Box(10, 10, 1),
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

  const PetscSymmetricBlockSparseMatrix& EvalFreeMotionTangentMatrix(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return driver_->EvalFreeMotionTangentMatrix(context, index);
  }

  const SchurComplement<double>& EvalFreeMotionTangentMatrixSchurComplement(
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
        RigidTransformd(), make_unique<Sphere>(1.0), move(name));
    geometry::ProximityProperties props;
    geometry::AddContactMaterial({}, kPointContactStiffness,
                                 CoulombFriction<double>(1.0, 1.0), &props);
    props.AddProperty(geometry::internal::kMaterialGroup,
                      geometry::internal::kRelaxationTime,
                      kDissipationTimeScale);
    geometry->set_proximity_properties(move(props));
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(1e6);
    body_config.set_poissons_ratio(0.4);
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId body_id =
        model->RegisterDeformableBody(move(geometry), body_config, kRezHint);
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
      EvalDofPermutation(plant_context, DeformableBodyIndex(0));
  VectorXd participating_v0_star(p0.permuted_domain_size());
  VectorXd participating_v1_star(p1.permuted_domain_size());
  p0.Apply(v0_star, &participating_v0_star);
  p1.Apply(v1_star, &participating_v1_star);
  const int num_participating_vertices = 6;
  const int num_participating_dofs_per_body = num_participating_vertices * 3;
  VectorXd expected_participating_v_star(2 * num_participating_dofs_per_body);
  expected_participating_v_star << participating_v0_star, participating_v1_star;
  EXPECT_EQ(expected_participating_v_star,
            EvalParticipatingFreeMotionVelocities(plant_context));
}

TEST_F(DeformableDriverContactTest, EvalFreeMotionTangentMatrix) {
  DeformableBodyIndex body_index(0);
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const FemState<double>& free_motion_state =
      EvalFreeMotionFemState(plant_context, body_index);
  DeformableBodyId body_id = model_->GetBodyId(body_index);
  const FemModel<double>& fem_model = model_->GetFemModel(body_id);
  std::unique_ptr<PetscSymmetricBlockSparseMatrix> fem_tangent_matrix =
      fem_model.MakePetscSymmetricBlockSparseTangentMatrix();
  fem_model.CalcTangentMatrix(free_motion_state, GetIntegratorWeights(),
                              fem_tangent_matrix.get());
  fem_tangent_matrix->AssembleIfNecessary();
  const MatrixXd expected_tangent_matrix =
      fem_tangent_matrix->MakeDenseMatrix() * plant_->time_step();
  const PetscSymmetricBlockSparseMatrix& calculated_tangent_matrix =
      EvalFreeMotionTangentMatrix(plant_context, body_index);
  EXPECT_TRUE(CompareMatrices(expected_tangent_matrix,
                              calculated_tangent_matrix.MakeDenseMatrix(),
                              1e-14, MatrixCompareType::relative));
}

TEST_F(DeformableDriverContactTest,
       EvalFreeMotionTangentMatrixSchurComplement) {
  DeformableBodyIndex body_index(0);
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  const PetscSymmetricBlockSparseMatrix& tangent_matrix =
      EvalFreeMotionTangentMatrix(plant_context, body_index);
  const MatrixXd dense_tangent_matrix = tangent_matrix.MakeDenseMatrix();
  /* Schematically the participating block (A), the non-participating block (D),
   and the off-diagonal block (B) look like
                           _______________________________________
                           |                       |      |      |
   Dofs 0-14 (associated   |                       |      |      |
   with vertices 0-4) are  |                       |      |      |
   participating.          |                       |      |      |
                           |                       |      |      |
                           |           A           |   B  |   A  |
                           |                       |      |      |
                           |                       |      |      |
                           |                       |      |      |
                           |                       |      |      |
                           |_______________________|______|______|
   Dofs 15-17 (associated  |                       |      |      |
   with vertex 5) are not  |                       |   D  |      |
   participating.          |_______________________|______|______|
   Dofs 18-20 (associated  |                       |      |      |
   with vertex 6) are      |           A           |   B  |   A  |
   participating.          |_______________________|______|______|       */
  /* Matrix block for participating dofs. */
  const int num_participating_vertices = 6;
  const int num_participating_dofs = num_participating_vertices * 3;
  MatrixXd A = MatrixXd::Zero(num_participating_dofs, num_participating_dofs);
  /* Vertices 0, 1, 2, 3, 4, 6 are participating in contact. */
  A.topLeftCorner(15, 15) = dense_tangent_matrix.topLeftCorner(15, 15);
  A.topRightCorner(15, 3) = dense_tangent_matrix.topRightCorner(15, 3);
  A.bottomLeftCorner(3, 15) = dense_tangent_matrix.bottomLeftCorner(3, 15);
  A.bottomRightCorner(3, 3) = dense_tangent_matrix.bottomRightCorner(3, 3);
  /* Matrix block for non-participating dofs. */
  const int num_nonparticipating_vertices = 1;
  const int num_nonparticipating_dofs = num_nonparticipating_vertices * 3;
  MatrixXd D = dense_tangent_matrix.block<3, 3>(15, 15);
  /* Off diagonal block. */
  MatrixXd B =
      MatrixXd::Zero(num_participating_dofs, num_nonparticipating_dofs);
  B.topRows(15) = dense_tangent_matrix.block<15, 3>(0, 15);
  B.bottomRows(3) = dense_tangent_matrix.block<3, 3>(18, 15);
  const MatrixXd expected_complement_matrix =
      SchurComplement<double>(A.sparseView(), B.transpose().sparseView(),
                              D.sparseView())
          .get_D_complement();
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
                      .get_D_complement());
  EXPECT_EQ(A[1], EvalFreeMotionTangentMatrixSchurComplement(plant_context,
                                                             body_index1)
                      .get_D_complement());
}

TEST_F(DeformableDriverContactTest, AppendDiscreteContactPairs) {
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(*context_);
  std::vector<DiscreteContactPair<double>> contact_pairs;
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
  constexpr double expected_k = kPointContactStiffness / 2.0;
  GeometryId id0 = model_->GetGeometryId(body_id0_);
  GeometryId id1 = model_->GetGeometryId(body_id1_);

  for (const DiscreteContactPair<double>& pair : contact_pairs) {
    EXPECT_TRUE(pair.id_A == id0 || pair.id_A == id1);
    EXPECT_EQ(pair.id_B, rigid_geometry_id_);
    /* The contact point is on the z = -0.25 plane, the top surface of the rigid
     box. */
    EXPECT_EQ(pair.p_WC(2), -0.25);
    EXPECT_TRUE(CompareMatrices(pair.nhat_BA_W, Eigen::Vector3d(0, 0, 1)));
    EXPECT_EQ(pair.stiffness, expected_k);
    EXPECT_EQ(pair.dissipation_time_scale, expected_tau);
    EXPECT_EQ(pair.friction_coefficient, 1.0);
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

  /* Build A. */
  const MatrixXd A =
      EvalFreeMotionTangentMatrix(plant_context, DeformableBodyIndex(0))
          .MakeDenseMatrix();

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
  const double relative_tolerance = 1e-6;  // The default SAP tolerance.
  EXPECT_TRUE(CompareMatrices(A * (v_next - v_star), tau * kDt,
                              scale * relative_tolerance));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
