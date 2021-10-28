#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fem {
/* Deformable body parameters.  */
constexpr double kYoungsModulus = 1e6;       // unit: N/m²
constexpr double kPoissonRatio = 0.3;        // unitless.
constexpr double kDensity = 1e3;             // unit: kg/m³
constexpr double kMassDamping = 0.001;       // unit: 1/s
constexpr double kStiffnessDamping = 0.002;  // unit: s
/* Time step (seconds). */
constexpr double kDt = 0.0001;
/* Contact parameters. Values not used by PGS solver are set to NAN to verify
 that they are indeed unused. */
constexpr double kContactStiffness = NAN;
constexpr double kContactDissipation = NAN;
const CoulombFriction kFriction{0.3, 0.2};
/* PGS solver parameters. */
/* Absolute tolerance (m/s) *and* relative tolerance (unitless). */
constexpr double kTolerance = 1e-6;
constexpr int kMaxIterations = 100;
constexpr double kRelaxationFactor = 1.0;
constexpr int kNumCubeVertices = 8;
constexpr int kNumOctahedronVertices = 7;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::GeometryId;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;
using math::RigidTransformd;
using multibody::contact_solvers::internal::BlockSparseMatrix;
using multibody::contact_solvers::internal::ContactSolverResults;
using multibody::contact_solvers::internal::ExtractNormal;
using multibody::contact_solvers::internal::ExtractTangent;
using multibody::contact_solvers::internal::PgsSolver;
using systems::Context;

/* Returns a proximity property with the given point contact stiffness,
 dissipation, and friction. */
ProximityProperties MakeProximityProperties(double stiffness,
                                            double dissipation,
                                            const CoulombFriction<double>& mu) {
  ProximityProperties proximity_properties;
  geometry::AddContactMaterial(dissipation, stiffness, mu,
                               &proximity_properties);
  return proximity_properties;
}

/* Returns a proximity property with an arbitrary set of default point contact
 stiffness, dissipation, and friction. */
ProximityProperties MakeDefaultProximityProperties() {
  return MakeProximityProperties(kContactStiffness, kContactDissipation,
                                 kFriction);
}

/* Returns the unit inertia for a unit cube. */
SpatialInertia<double> MakeSpatialInertiaForBox() {
  const UnitInertia<double> G_Rcm =
      UnitInertia<double>::SolidBox(1.0, 1.0, 1.0);
  return SpatialInertia<double>(1e3, Vector3d::Zero(), G_Rcm);
}

/* Creates a dummy DeformableBodyConfig. */
DeformableBodyConfig<double> MakeDeformableBodyConfig() {
  DeformableBodyConfig<double> config;
  config.set_youngs_modulus(kYoungsModulus);
  config.set_poisson_ratio(kPoissonRatio);
  config.set_mass_damping_coefficient(kMassDamping);
  config.set_stiffness_damping_coefficient(kStiffnessDamping);
  config.set_mass_density(kDensity);
  config.set_material_model(MaterialModel::kLinear);
  return config;
}

internal::ReferenceDeformableGeometry<double> MakeUnitCubeDeformableGeometry() {
  auto mesh = std::make_unique<VolumeMesh<double>>(
      geometry::internal::MakeBoxVolumeMesh<double>(
          geometry::Box(1.0, 1.0, 1.0), 1.0));
  std::vector<double> dummy_signed_distances(kNumCubeVertices, 0.0);
  auto mesh_field = std::make_unique<VolumeMeshFieldLinear<double, double>>(
      std::move(dummy_signed_distances), mesh.get(), false);
  return {std::move(mesh), std::move(mesh_field)};
}

// TODO(xuchenhan-tri): This test can be strengthened by adding a case where a
//  deformable object is in contact with a fixed collision geometry and no rigid
//  dofs exist.
/* Set up a scene with a deformable octahedron, a deformable cube, and a rigid
 unit cube to faciliate testing of the contact solver. */
class DeformableRigidContactSolverTest : public ::testing::Test {
 protected:
  /* Set up a scene with a deformable octahedron and one rigid cube. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    /* Add a deformable octahedron. */
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    deformable_cube_ = deformable_model->RegisterDeformableBody(
        MakeUnitCubeDeformableGeometry(), "deformable unit cube",
        MakeDeformableBodyConfig(), MakeDefaultProximityProperties());
    deformable_octahedron_ = deformable_model->RegisterDeformableBody(
        MakeOctahedronDeformableGeometry<double>(), "deformable octahedron",
        MakeDeformableBodyConfig(), MakeDefaultProximityProperties());
    plant_->AddPhysicalModel(std::move(deformable_model));
    /* Add the rigid cube. */
    rigid_index_ =
        plant_->AddRigidBody("rigid cube", MakeSpatialInertiaForBox()).index();
    rigid_geometry_index_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(rigid_index_), math::RigidTransform<double>(),
        geometry::Box(1.0, 1.0, 1.0), "rigid cube collision",
        MakeDefaultProximityProperties());
    plant_->Finalize();

    auto pgs_solver = std::make_unique<PgsSolver<double>>();
    pgs_solver->set_parameters(
        {kRelaxationFactor, kTolerance, kTolerance, kMaxIterations});
    auto owned_deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>(std::move(pgs_solver));
    deformable_rigid_manager_ = owned_deformable_rigid_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(*scene_graph_);

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
  }

  /* Calls DeformableRigidManager::EvalFreeMotionFemStateBase(). */
  const FemStateBase<double>& EvalFreeMotionFemStateBase(
      const Context<double>& context, DeformableBodyIndex index) const {
    return deformable_rigid_manager_->EvalFreeMotionFemStateBase(context,
                                                                 index);
  }

  /* Calls DeformableRigidManager::EvalTwoWayCoupledContactSolverResults(). */
  const ContactSolverResults<double>& EvalTwoWayCoupledContactSolverResults(
      const Context<double>& context) const {
    return deformable_rigid_manager_->EvalTwoWayCoupledContactSolverResults(
        context);
  }

  /* Calls DeformableRigidManager::CalcContactJacobian() and returns the result
   as a dense matrix. */
  MatrixXd CalcContactJacobian(const Context<double>& context) const {
    return deformable_rigid_manager_->CalcContactJacobian(context)
        .MakeDenseMatrix();
  }

  /* Calls DeformableRigidManager::EvalContactTangentMatrix() and returns the
   tangent matrix for contact as a dense matrix. */
  MatrixXd CalcContactTangentMatrix(const Context<double>& context) const {
    const BlockSparseMatrix<double> A =
        deformable_rigid_manager_->EvalContactTangentMatrix(context);
    return A.MakeDenseMatrix();
  }

  /* Calls DeformableRigidManager::EvalFreeMotionParticipatingVelocities() to
   evaluate the free motion velocity `v_star`. */
  const VectorX<double>& EvalFreeMotionParticipatingVelocities(
      const systems::Context<double>& context) const {
    return deformable_rigid_manager_->EvalFreeMotionParticipatingVelocities(
        context);
  }

  /* Calls DeformableRigidManager::EvalFreeMotionTangentMatrix(). */
  const Eigen::SparseMatrix<double>& EvalFreeMotionTangentMatrix(
      const Context<double>& context, DeformableBodyIndex index) const {
    return deformable_rigid_manager_->EvalFreeMotionTangentMatrix(context,
                                                                  index);
  }

  /* Calls DeformableRigidManager::EvalDeformableContactSolverResults(). */
  const ContactSolverResults<double>& EvalDeformableContactSolverResults(
      const Context<double>& context) const {
    return deformable_rigid_manager_->EvalDeformableContactSolverResults(
        context);
  }

  /* Calls DiscreteUpdateManager::CalcContactSolverResults() and returns the
   output. */
  const ContactSolverResults<double> CalcContactSolverResults(
      const Context<double>& context) const {
    ContactSolverResults<double> results;
    deformable_rigid_manager_->CalcContactSolverResults(context, &results);
    return results;
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableRigidManager<double>* deformable_rigid_manager_{nullptr};
  DeformableBodyIndex deformable_octahedron_;
  DeformableBodyIndex deformable_cube_;
  BodyIndex rigid_index_;
  GeometryId rigid_geometry_index_;
  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_{nullptr};
};

namespace {

/* Verifies that when there's no contact,
 EvalTwoWayCoupledContactSolverResults() returns an empty
 ContactSolverResults. */
TEST_F(DeformableRigidContactSolverTest, NoContact) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Move the rigid cube far away so that it doesn't intersect the deformable
   octahedron. */
  const Vector3d p_WB(0, 0, 5);
  const math::RigidTransformd X_WB(p_WB);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(rigid_index_), X_WB);

  const ContactSolverResults<double>& results =
      EvalTwoWayCoupledContactSolverResults(plant_context);
  EXPECT_EQ(results.vn.size(), 0);
  EXPECT_EQ(results.vt.size(), 0);
  EXPECT_EQ(results.fn.size(), 0);
  EXPECT_EQ(results.ft.size(), 0);
  EXPECT_EQ(results.v_next.size(), 0);
  EXPECT_EQ(results.tau_contact.size(), 0);
}

/* Verifies that when the deformable octahedron and the rigid cube are in
 contact and have opposing velocity EvalTwoWayCoupledContactSolverResults()
 returns an expected result. The test setup looks like the following in the
 yz-plane:
                  +Z
                  |
                  ●
                ⁄  | ＼  deformable
              ⁄----+---＼  octahedron/cube
            ⁄  |   |   | ＼
   -Y-----●---+---●---+---●----+Y
            \ |   |   |  ⁄
              \---|----⁄
              --\-|--⁄--
              |   ●   |
              |   |   |
              |   |   |
              ----|---- rigid cube
                  |
                 -Z                    */
TEST_F(DeformableRigidContactSolverTest, InContact) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Set the position of the rigid cube so that it is in contact with the
   deformable octahedron but not the deformable cube. */
  const Vector3d p_WB(0, 0, -1.25);
  const math::RigidTransformd X_WB(p_WB);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(rigid_index_), X_WB);
  /* Introduce some arbitrary spatial velocity for the rigid cube to make the
   contact problem interesting. */
  const SpatialVelocity<double> V_WB(Vector3d(0.1, 0.2, 0.3),
                                     Vector3d(0.7, 0.8, 0.9));
  plant_->SetFreeBodySpatialVelocity(&plant_context,
                                     plant_->get_body(rigid_index_), V_WB);

  const ContactSolverResults<double>& results =
      EvalTwoWayCoupledContactSolverResults(plant_context);
  const MatrixXd Jc = CalcContactJacobian(plant_context);
  const int nc = Jc.rows() / 3;
  const int nv = Jc.cols();
  /* Ensure that the result has the expected size. */
  ASSERT_EQ(results.v_next.size(), nv);
  ASSERT_EQ(results.tau_contact.size(), nv);
  ASSERT_EQ(results.vn.size(), nc);
  ASSERT_EQ(results.vt.size(), 2 * nc);
  ASSERT_EQ(results.fn.size(), nc);
  ASSERT_EQ(results.ft.size(), 2 * nc);

  /* Verify that `v_next` and `vn` and `vt` are consistent. */
  const VectorXd vc = Jc * results.v_next;
  VectorXd expected_vn(nc);
  VectorXd expected_vt(2 * nc);
  ExtractNormal(vc, &expected_vn);
  ExtractTangent(vc, &expected_vt);
  EXPECT_TRUE(CompareMatrices(results.vn, expected_vn,
                              4 * std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(results.vt, expected_vt,
                              4 * std::numeric_limits<double>::epsilon()));

  /* Verify that `tau_contact` and `fn` and `ft` are consistent. */
  VectorXd gamma(3 * nc);
  for (int i = 0; i < nc; ++i) {
    gamma(3 * i) = results.ft(2 * i);
    gamma(3 * i + 1) = results.ft(2 * i + 1);
    gamma(3 * i + 2) = results.fn(i);
  }
  const VectorXd expected_tau_contact = Jc.transpose() * gamma;
  EXPECT_TRUE(CompareMatrices(results.tau_contact, expected_tau_contact,
                              std::numeric_limits<double>::epsilon()));

  /* Verify that the equation A⋅Δv = Jcᵀ⋅γ is satisfied. */
  const MatrixXd A = CalcContactTangentMatrix(plant_context);
  const VectorXd v_star = EvalFreeMotionParticipatingVelocities(plant_context);
  const VectorXd dv = results.v_next - v_star;
  EXPECT_TRUE(
      CompareMatrices(A * dv, results.tau_contact,
                      A.norm() * std::numeric_limits<double>::epsilon()));

  /* Verify that the friction-cone constraint is satisfied. */
  for (int ic = 0; ic < nc; ++ic) {
    EXPECT_LE(results.ft.segment<2>(2 * ic).norm(),
              results.fn(ic) * kFriction.dynamic_friction());
  }

  /* Verify that the complementarity constraint is satisfied. */
  EXPECT_TRUE((results.vn.array() >= -kTolerance).all());
  EXPECT_TRUE((results.fn.array() >= 0).all());
  EXPECT_TRUE(CompareMatrices(results.vn.cwiseProduct(results.fn),
                              VectorXd::Zero(nc), kTolerance));
}

/* Verifies that the deformable contact solver results obtained through the
 Schur complement matches expectation. In particular,
 1. the results at the contact points (i.e. contact forces and contact
    velocities) match their rigid counterparts, and
 2. the equation A⋅Δv = Jcᵀ⋅γ is satisfied for *all* (not just participating)
    deformable dofs. */
TEST_F(DeformableRigidContactSolverTest, DeformableResults) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* The test setup looks like the following in the yz-plane:
                  +Z
                  |
                  ●
                ⁄  | ＼  deformable
              ⁄----+---＼  octahedron/cube
            ⁄  |   |   | ＼
   -Y-----●---+---●---+---●----+Y
            \ |   |   |  ⁄
              \---|----⁄
              --\-|--⁄--
              |   ●   |
              |   |   |
              |   |   |
              ----|---- rigid cube
                  |
                 -Z                    */
  /* We set the position of the rigid cube so that it is in contact with the
   deformable octahedron but not the deformable cube. By doing so, the test
   covers the post-contact deformable results for both bodies that are in
   contact and those that aren't. */
  const Vector3d p_WB(0, 0, -1.25);
  const math::RigidTransformd X_WB(p_WB);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(rigid_index_), X_WB);
  /* Set up spatial velocities for the rigid cube so that the contact solver
   does some meaningful computation. I.e. the contact constraints aren't
   satisfied in the absense of contact forces. */
  const Vector3d w_WB(0.1, 0.2, 0.3);
  const Vector3d v_WB(0.4, 0.5, 0.6);
  const SpatialVelocity<double> V_WB(w_WB, v_WB);
  plant_->SetFreeBodySpatialVelocity(&plant_context,
                                     plant_->get_body(rigid_index_), V_WB);
  const ContactSolverResults<double>& deformable_results =
      EvalDeformableContactSolverResults(plant_context);
  const ContactSolverResults<double> rigid_results =
      CalcContactSolverResults(plant_context);

  /* Verifies the first documented test. Since there are only deformable-rigid
   contact and no rigid-rigid contact, the contact forces and velocities
   should be the same for the rigid results and the deformable results. */
  EXPECT_TRUE(CompareMatrices(deformable_results.fn, rigid_results.fn));
  EXPECT_TRUE(CompareMatrices(deformable_results.ft, rigid_results.ft));
  EXPECT_TRUE(CompareMatrices(deformable_results.vn, rigid_results.vn));
  EXPECT_TRUE(CompareMatrices(deformable_results.vt, rigid_results.vt));

  /* Verifies the second documented test. */
  /* The cube is not in contact, so v = v* (dv = 0) and tau = 0. */
  const FemStateBase<double>& cube_state_star =
      EvalFreeMotionFemStateBase(plant_context, deformable_cube_);
  const VectorXd& cube_v_star = cube_state_star.qdot();
  /* N.B. Deformable dofs are in the order deformable bodies are added. In this
   case the deformable cube was added to the model before the deformable
   octahedron and thus the deformable cube's dofs come first. */
  const VectorXd& cube_v = deformable_results.v_next.head(3 * kNumCubeVertices);
  EXPECT_TRUE(CompareMatrices(cube_v, cube_v_star));
  const VectorXd cube_tau =
      deformable_results.tau_contact.head(3 * kNumCubeVertices);
  EXPECT_TRUE(CompareMatrices(cube_tau, VectorXd::Zero(3 * kNumCubeVertices)));

  /* The octahedron is in contact, and we verify that A⋅Δv = Jcᵀ⋅γ. */
  const FemStateBase<double>& octahedron_state_star =
      EvalFreeMotionFemStateBase(plant_context, deformable_octahedron_);
  const VectorXd& octahedron_v_star = octahedron_state_star.qdot();
  const VectorXd& octahedron_v =
      deformable_results.v_next.tail(3 * kNumOctahedronVertices);
  const VectorXd dv = octahedron_v - octahedron_v_star;
  const MatrixXd A(
      EvalFreeMotionTangentMatrix(plant_context, deformable_octahedron_));
  const VectorXd Adv = A * dv;
  /* This `tau` is given by Jcᵀ⋅γ. */
  const VectorXd octahedron_tau =
      deformable_results.tau_contact.tail(3 * kNumOctahedronVertices);
  /* Verify that A⋅Δv = Jcᵀ⋅γ. */
  constexpr double kEpsilon = 1e-13;
  EXPECT_TRUE(CompareMatrices(Adv, octahedron_tau, kEpsilon));
  /* Confirm that the contact problem is non-trivial by checking the contact
   impulse is non-zero. */
  EXPECT_GE(octahedron_tau.norm(), kEpsilon);
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
