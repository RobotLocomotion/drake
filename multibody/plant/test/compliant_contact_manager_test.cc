#include "drake/multibody/plant/compliant_contact_manager.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/multibody/plant/test/spheres_stack.h"
#include "drake/multibody/plant/test_utilities/rigid_body_on_compliant_ground.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::PenetrationAsPointPair;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::TriangleSurfaceMesh;
using drake::geometry::VolumeMesh;
using drake::geometry::VolumeMeshFieldLinear;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::MergeNormalAndTangent;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::internal::DiscreteContactPair;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// TODO(amcastro-tri): Implement AutoDiffXd testing.

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
// TODO(amcastro-tri): Consider how to split SapDriver tests from
// CompliantContactManager tests.
class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }

  static void PackContactSolverResults(
      const Context<double>& context, const SapDriver<double>& driver,
      const contact_solvers::internal::SapContactProblem<double>& problem,
      int num_contacts,
      const contact_solvers::internal::SapSolverResults<double>& sap_results,
      contact_solvers::internal::ContactSolverResults<double>*
          contact_results) {
    driver.PackContactSolverResults(context, problem, num_contacts, sap_results,
                                    contact_results);
  }
};

namespace {

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
constexpr auto kDiscreteContactSolverTamsi = DiscreteContactSolver::kTamsi;
#pragma GCC diagnostic push

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Tests that in SetDiscreteUpdateManager, a non-empty DeformableModel will
// cause a DeformableDriver to be instantiated in the manager.
GTEST_TEST(CompliantContactManagerTest, ExtractModelInfo) {
  CompliantContactManager<double> manager;
  EXPECT_EQ(manager.deformable_driver(), nullptr);

  systems::DiagramBuilder<double> builder;
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.01;
  // N.B. Deformables are only supported with the SAP solver.
  // Thus for testing we choose one arbitrary contact approximation that
  // uses the SAP solver.
  plant_config.discrete_contact_approximation = "sap";
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();
  auto geometry_instance = std::make_unique<geometry::GeometryInstance>(
      math::RigidTransformd::Identity(), geometry::Sphere(1.0), "sphere");
  deformable_model.RegisterDeformableBody(
      std::move(geometry_instance), fem::DeformableBodyConfig<double>{}, 10.0);
  plant.Finalize();
  auto contact_manager = std::make_unique<CompliantContactManager<double>>();
  const CompliantContactManager<double>* contact_manager_ptr =
      contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(contact_manager));
  EXPECT_NE(contact_manager_ptr->deformable_driver(), nullptr);
}

// TODO(DamrongGuoy): Simplify the test fixture somehow (need discussion
//  among the architects). Due to the existing architecture of our code,
//  our fixture is too complex for the purpose of unit tests. Ideally there
//  should be one-to-one matching between tested functions (_.h) and testing
//  functions (_test.cc) as the unit tests.

// In this fixture we set a simple model consisting of a flat ground,
// a sphere (sphere 1) on top of the ground, and another sphere (sphere 2)
// on top the first sphere. They are assigned to be rigid-hydroelastic,
// compliant-hydroelastic, or non-hydroelastic to test various cases of
// contact quantities computed by the CompliantContactManager.
class SpheresStackTest : public SpheresStack, public ::testing::Test {
 public:
  // This function makes a model with the specified sphere 1 and sphere 2
  // properties and verifies the resulting contact pairs.
  // In this model sphere 1 always interacts with the ground using the
  // hydroelastic contact model.
  // Point contact stiffness must be provided for both spheres in
  // sphere1_point_params and sphere2_point_params.
  void VerifyDiscreteContactPairs(
      const ContactParameters& sphere1_point_params,
      const ContactParameters& sphere2_point_params) {
    // This test is specific to point contact. Both spheres must have point
    // contact properties.
    DRAKE_DEMAND(sphere1_point_params.point_stiffness.has_value());
    DRAKE_DEMAND(sphere2_point_params.point_stiffness.has_value());

    ContactParameters sphere1_contact_params = sphere1_point_params;
    sphere1_contact_params.hydro_modulus = 1.0e5;
    const ContactParameters sphere2_contact_params = sphere2_point_params;

    const ContactParameters hard_hydro_contact{
        std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
    const SphereParameters sphere1_params{
        "Sphere1", 10.0 /* mass */, 0.2 /* size */, sphere1_contact_params};
    const SphereParameters sphere2_params{
        "Sphere2", 10.0 /* mass */, 0.2 /* size */, sphere2_contact_params};

    // Soft sphere/hard ground.
    MakeModel(hard_hydro_contact, sphere1_params, sphere2_params);

    const GeometryContactData<double>& geometry_contact_data =
        EvalGeometryContactData(*plant_context_);
    const std::vector<PenetrationAsPointPair<double>>& point_pairs =
        geometry_contact_data.get().point_pairs;
    const std::vector<geometry::ContactSurface<double>>& surfaces =
        geometry_contact_data.get().surfaces;
    const int num_point_pairs = point_pairs.size();
    ASSERT_EQ(surfaces.size(), 1u);
    const int num_hydro_pairs = surfaces[0].num_faces();

    // In these tests ContactParameters::relaxation_time = nullopt
    // indicates we want to build a model for which we forgot to specify the
    // relaxation time in ProximityProperties. Here we verify this is not
    // required by the manager, since the manager specifies a default value.
    if (!sphere1_point_params.relaxation_time.has_value() ||
        !sphere2_point_params.relaxation_time.has_value()) {
      EXPECT_NO_THROW(
          contact_manager_->EvalDiscreteContactPairs(*plant_context_));
      return;
    }

    // Verify that the manager throws an exception if a negative relaxation
    // times is provided.
    if (*sphere1_point_params.relaxation_time < 0 ||
        *sphere2_point_params.relaxation_time < 0) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          contact_manager_->EvalDiscreteContactPairs(*plant_context_),
          "Relaxation time must be non-negative "
          "and relaxation_time = .* was "
          "provided. For geometry .* on body .*.");
      return;
    }

    const DiscreteContactData<DiscreteContactPair<double>>& pairs =
        contact_manager_->EvalDiscreteContactPairs(*plant_context_);
    EXPECT_EQ(pairs.size(), num_point_pairs + num_hydro_pairs);

    const GeometryId sphere2_geometry =
        plant_->GetCollisionGeometriesForBody(*sphere2_)[0];

    for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
      const DiscreteContactPair<double>& point_pair = pairs[i];

      if (i < num_point_pairs) {
        // Unit tests for point contact only.
        // Here we use our knowledge that we always place point contact pairs
        // followed by hydroelastic contact pairs.
        const double phi_expected = -penetration_distance_;
        // The geometry engine computes absolute values of penetration to
        // machine epsilon (at least for sphere vs. sphere contact).
        EXPECT_NEAR(point_pair.phi0, phi_expected, kEps);

        const double k1 = *sphere1_contact_params.point_stiffness;
        const double k2 = *sphere2_contact_params.point_stiffness;
        const double stiffness_expected = (k1 * k2) / (k1 + k2);
        EXPECT_NEAR(point_pair.stiffness, stiffness_expected,
                    kEps * stiffness_expected);

        // Verify contact location.
        const double pz_WS1 =
            plant_->GetFreeBodyPose(*plant_context_, *sphere1_)
                .translation()
                .z();
        const double pz_WC = -k2 / (k1 + k2) * penetration_distance_ + pz_WS1 +
                             sphere1_params.radius;
        EXPECT_NEAR(point_pair.p_WC.z(), pz_WC, 1.0e-14);

        // Check the optional parameters are set correctly for point contact.
        // The index into `point_pairs` should match the index into `pairs`.
        EXPECT_TRUE(point_pair.point_pair_index.has_value());
        EXPECT_EQ(point_pair.point_pair_index.value(), i);
        EXPECT_FALSE(point_pair.surface_index.has_value());
        EXPECT_FALSE(point_pair.face_index.has_value());
      } else {
        // Check the optional parameters are set correctly for hydro.
        EXPECT_FALSE(point_pair.point_pair_index.has_value());
        EXPECT_TRUE(point_pair.surface_index.has_value());
        EXPECT_TRUE(point_pair.face_index.has_value());
      }

      // Unit tests for both point and hydroelastic discrete pairs.
      const int sign = point_pair.id_A == sphere2_geometry ? 1 : -1;
      const Vector3d normal_expected = sign * Vector3d::UnitZ();
      EXPECT_TRUE(CompareMatrices(point_pair.nhat_BA_W, normal_expected));

      // Verify dissipation.
      const double tau1 = *sphere1_contact_params.relaxation_time;
      const double tau2 = i == 0 ? *sphere2_contact_params.relaxation_time
                                 : *hard_hydro_contact.relaxation_time;
      const double tau_expected = tau1 + tau2;
      EXPECT_NEAR(point_pair.dissipation_time_scale, tau_expected,
                  kEps * tau_expected);

      // Verify friction.
      const double mu1 = sphere1_contact_params.friction_coefficient;
      const double mu2 = i == 0 ? sphere2_contact_params.friction_coefficient
                                : hard_hydro_contact.friction_coefficient;
      const double mu_expected = 2.0 * (mu1 * mu2) / (mu1 + mu2);
      EXPECT_NEAR(point_pair.friction_coefficient, mu_expected,
                  kEps * mu_expected);
    }
  }

  // Helper method to test EvalContactJacobianCache().
  // Returns the Jacobian J_AcBc_W.
  MatrixXd CalcDenseJacobianMatrixInWorldFrame(
      const DiscreteContactData<DiscreteContactPair<double>>& contact_pairs)
      const {
    const int nc = contact_pairs.size();
    const MatrixXd J_AcBc_C =
        CompliantContactManagerTester::CalcDenseJacobianMatrixInContactFrame(
            *contact_manager_, contact_pairs);
    MatrixXd J_AcBc_W(3 * nc, contact_manager_->plant().num_velocities());
    J_AcBc_W.setZero();
    for (int i = 0; i < nc; ++i) {
      const DiscreteContactPair<double>& contact_pair = contact_pairs[i];
      J_AcBc_W.middleRows<3>(3 * i) =
          contact_pair.R_WC.matrix() * J_AcBc_C.middleRows<3>(3 * i);
    }
    return J_AcBc_W;
  }

  // In the functions below we use CompliantContactManagerTester to provide
  // access to private functions for unit testing.

  const GeometryContactData<double>& EvalGeometryContactData(
      const Context<double>& context) const {
    return CompliantContactManagerTester::EvalGeometryContactData(
        *contact_manager_, context);
  }

  const SapContactProblem<double>& EvalSapContactProblem(
      const Context<double>& context) const {
    const auto& sap_driver =
        CompliantContactManagerTester::sap_driver(*contact_manager_);
    return *SapDriverTest::EvalContactProblemCache(sap_driver, *plant_context_)
                .sap_problem;
  }
};

// Unit test to verify the computation of the contact kinematics.
TEST_F(SpheresStackTest, CalcDiscreteContactPairs) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  const double radius = 0.2;  // Spheres's radii in the default setup.

  const DiscreteContactData<DiscreteContactPair<double>>& contact_pairs =
      contact_manager_->EvalDiscreteContactPairs(*plant_context_);
  const MatrixXd J_AcBc_W = CalcDenseJacobianMatrixInWorldFrame(contact_pairs);

  // Arbitrary velocity of sphere 1.
  const Vector3d v_WS1(1, 2, 3);
  const Vector3d w_WS1(4, 5, 6);
  const SpatialVelocity<double> V_WS1(w_WS1, v_WS1);

  // Arbitrary velocity of sphere 2.
  const Vector3d v_WS2(7, 8, 9);
  const Vector3d w_WS2(10, 11, 12);
  const SpatialVelocity<double> V_WS2(w_WS2, v_WS2);

  plant_->SetFreeBodySpatialVelocity(plant_context_, *sphere1_, V_WS1);
  plant_->SetFreeBodySpatialVelocity(plant_context_, *sphere2_, V_WS2);
  const VectorXd v = plant_->GetVelocities(*plant_context_);

  const GeometryId sphere1_geometry =
      plant_->GetCollisionGeometriesForBody(*sphere1_)[0];

  // Verify contact Jacobian for the point pair.
  // For this model we know the first entry corresponds to the single point pair
  // between sphere 1 and sphere 2.
  {
    // For the default setup both spheres are equally compliant and therefore
    // the contact point C lies right in the middle.
    const Vector3d p_S1C_W(0, 0, radius - penetration_distance_ / 2.0);
    const Vector3d p_S2C_W(0, 0, -(radius - penetration_distance_ / 2.0));

    // Compute expected contact point velocity.
    const Vector3d v_WS1c = V_WS1.Shift(p_S1C_W).translational();
    const Vector3d v_WS2c = V_WS2.Shift(p_S2C_W).translational();
    const Vector3d expected_v_S1cS2c_W = v_WS2c - v_WS1c;

    const int sign = contact_pairs[0].id_A == sphere1_geometry ? 1 : -1;
    const MatrixXd J_S1cS2c_W = sign * J_AcBc_W.topRows(3);
    const Vector3d v_S1cS2c_W = J_S1cS2c_W * v;
    EXPECT_TRUE(CompareMatrices(v_S1cS2c_W, expected_v_S1cS2c_W, kEps,
                                MatrixCompareType::relative));

    // Verify configuration.
    const BodyIndex body_A_index =
        contact_manager_->FindBodyByGeometryId(contact_pairs[0].id_A);
    const BodyIndex body_B_index =
        contact_manager_->FindBodyByGeometryId(contact_pairs[0].id_B);
    EXPECT_EQ(BodyIndex(contact_pairs[0].object_A), body_A_index);
    EXPECT_EQ(BodyIndex(contact_pairs[0].object_B), body_B_index);
    const Vector3d p_AoC_W =
        body_A_index == sphere1_->index() ? p_S1C_W : p_S2C_W;
    const Vector3d p_BoC_W =
        body_B_index == sphere1_->index() ? p_S1C_W : p_S2C_W;
    EXPECT_TRUE(CompareMatrices(contact_pairs[0].p_ApC_W, p_AoC_W, kEps,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(contact_pairs[0].p_BqC_W, p_BoC_W, kEps,
                                MatrixCompareType::relative));
  }

  // Verify contact Jacobian for hydroelastic pairs.
  // We know hydroelastic pairs come after point pairs.
  {
    const Vector3d p_WS1(0, 0, radius - penetration_distance_);
    for (int q = 1; q < contact_pairs.size(); ++q) {
      const Vector3d& p_WC = contact_pairs[q].p_WC;
      const Vector3d p_S1C_W = p_WC - p_WS1;
      const Vector3d expected_v_WS1c = V_WS1.Shift(p_S1C_W).translational();
      const int sign = contact_pairs[q].id_B == sphere1_geometry ? 1 : -1;
      const MatrixXd J_WS1c_W =
          sign * J_AcBc_W.block(3 * q, 0, 3, plant_->num_velocities());
      const Vector3d v_WS1c_W = J_WS1c_W * v;
      EXPECT_TRUE(CompareMatrices(v_WS1c_W, expected_v_WS1c, kEps,
                                  MatrixCompareType::relative));

      // Verify configuration.
      const BodyIndex body_A_index =
          contact_manager_->FindBodyByGeometryId(contact_pairs[q].id_A);
      const BodyIndex body_B_index =
          contact_manager_->FindBodyByGeometryId(contact_pairs[q].id_B);
      EXPECT_EQ(BodyIndex(contact_pairs[q].object_A), body_A_index);
      EXPECT_EQ(BodyIndex(contact_pairs[q].object_B), body_B_index);
      const Vector3d p_AoC_W =
          body_A_index == sphere1_->index() ? p_S1C_W : p_WC;
      const Vector3d p_BoC_W =
          body_B_index == sphere1_->index() ? p_S1C_W : p_WC;
      EXPECT_TRUE(CompareMatrices(contact_pairs[q].p_ApC_W, p_AoC_W, kEps,
                                  MatrixCompareType::relative));
      EXPECT_TRUE(CompareMatrices(contact_pairs[q].p_BqC_W, p_BoC_W, kEps,
                                  MatrixCompareType::relative));
    }
  }
}

// Unit test to verify discrete contact pairs computed by the manager for
// different combinations of compliance.
TEST_F(SpheresStackTest, VerifyDiscreteContactPairs) {
  ContactParameters soft_point_contact{1.0e3, std::nullopt, 0.01, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard sphere 1/soft sphere 2.
  VerifyDiscreteContactPairs(hard_point_contact, soft_point_contact);

  // Equally soft spheres.
  VerifyDiscreteContactPairs(soft_point_contact, soft_point_contact);

  // Soft sphere 1/hard sphere 2.
  VerifyDiscreteContactPairs(soft_point_contact, hard_point_contact);
}

TEST_F(SpheresStackTest, RelaxationTimeIsNotRequired) {
  ContactParameters soft_point_contact{
      1.0e3, std::nullopt,
      std::nullopt /* Dissipation not included in ProximityProperties */, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard sphere 1/soft sphere 2.
  VerifyDiscreteContactPairs(hard_point_contact, soft_point_contact);

  // Equally soft spheres.
  VerifyDiscreteContactPairs(soft_point_contact, soft_point_contact);

  // Soft sphere 1/hard sphere 2.
  VerifyDiscreteContactPairs(soft_point_contact, hard_point_contact);
}

TEST_F(SpheresStackTest, RelaxationTimeMustBePositive) {
  ContactParameters soft_point_contact{
      1.0e3, std::nullopt, -1.0 /* Negative dissipation timescale */, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard sphere 1/soft sphere 2.
  VerifyDiscreteContactPairs(hard_point_contact, soft_point_contact);

  // Equally soft spheres.
  VerifyDiscreteContactPairs(soft_point_contact, soft_point_contact);

  // Soft sphere 1/hard sphere 2.
  VerifyDiscreteContactPairs(soft_point_contact, hard_point_contact);
}

// Unit test to verify discrete contact pairs computed by the manager for
// rigid-compliant hydroelastic contact with point-contact fall back.
TEST_F(SpheresStackTest,
       VerifyDiscreteContactPairsFromRigidCompliantHydroelasticContact) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();

  const std::vector<PenetrationAsPointPair<double>>& point_pairs =
      EvalGeometryContactData(*plant_context_).get().point_pairs;
  const int num_point_pairs = point_pairs.size();
  EXPECT_EQ(num_point_pairs, 1);
  const DiscreteContactData<DiscreteContactPair<double>>& pairs =
      contact_manager_->EvalDiscreteContactPairs(*plant_context_);

  const std::vector<geometry::ContactSurface<double>>& surfaces =
      EvalGeometryContactData(*plant_context_).get().surfaces;
  ASSERT_EQ(surfaces.size(), 1);
  EXPECT_EQ(pairs.size(), surfaces[0].num_faces() + num_point_pairs);
}

// The purpose of this test is not to verify the correctness of the computation,
// but rather to verify that data flows correctly. That is, that
// CalcContactSolverResults() loads the expected computation into the contact
// results.
// TODO(amcastro-tri): The correctness of the computation should be moved into
// the SapDriver tests. Here we should only test that the manager invokes the
// SapDriver counterpart.
TEST_F(SpheresStackTest, DoCalcContactSolverResults) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  // N.B. We make sure both the manager and the manual invocations of the SAP
  // solver in this test both use the same set of parameters.
  SapSolverParameters params;  // Default set of parameters.
  contact_manager_->set_sap_solver_parameters(params);
  ContactSolverResults<double> contact_results;
  contact_manager_->CalcContactSolverResults(*plant_context_, &contact_results);

  // Generate contact results here locally to verify that
  // CalcContactSolverResults() loads them properly.
  const SapContactProblem<double>& sap_problem =
      EvalSapContactProblem(*plant_context_);
  const int num_contacts = sap_problem.num_constraints();  // Only contacts.
  SapSolver<double> sap;
  sap.set_parameters(params);
  SapSolverResults<double> sap_results;
  const SapSolverStatus status = sap.SolveWithGuess(
      sap_problem, plant_->GetVelocities(*plant_context_), &sap_results);
  ASSERT_EQ(status, SapSolverStatus::kSuccess);

  ContactSolverResults<double> contact_results_expected;
  const auto& sap_driver =
      CompliantContactManagerTester::sap_driver(*contact_manager_);
  SapDriverTest::PackContactSolverResults(
      *plant_context_, sap_driver, sap_problem, num_contacts, sap_results,
      &contact_results_expected);

  // Verify the expected result.
  EXPECT_TRUE(CompareMatrices(contact_results.v_next,
                              contact_results_expected.v_next, kEps,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.fn, contact_results_expected.fn,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.ft, contact_results_expected.ft,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.vn, contact_results_expected.vn,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.vt, contact_results_expected.vt,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.tau_contact,
                              contact_results_expected.tau_contact, kEps,
                              MatrixCompareType::relative));
}

// The purpose of this test is to verify that CalcDiscreteValues() loads state
// updates correctly. This can be verified even with a simple case without
// contact, for which we can manually compute the solution. The correctness of
// contact results in configurations with contact is tested elsewhere.
// In this case we setup two free falling spheres for which we can compute the
// state update by hand, under the assumption the manager is using a symplectic
// Euler scheme. This assumping might need to be updated in the future when
// other schemes are supported.
TEST_F(SpheresStackTest, DoCalcDiscreteValues) {
  SetupFreeFloatingSpheresWithNoContact();

  // Both spheres accelerate from zero velocity in a single time step with
  // gravity along the z-axis.
  const Vector3d v_WS(0., 0., -gravity_ * plant_->time_step());
  const SpatialVelocity<double> V_WS(Vector3d::Zero(), v_WS);

  // Positions at the previous time step.
  const Vector3d& p_WS10 =
      plant_->EvalBodyPoseInWorld(*plant_context_, *sphere1_).translation();
  const Vector3d& p_WS20 =
      plant_->EvalBodyPoseInWorld(*plant_context_, *sphere2_).translation();

  // The manager uses a symplectic update of the positions. For this case then
  // we know the positions are:
  const Vector3d p_WS1 = p_WS10 + plant_->time_step() * v_WS;
  const Vector3d p_WS2 = p_WS20 + plant_->time_step() * v_WS;

  // Create a new context for the next state.
  auto next_context = plant_->CreateDefaultContext();

  // In this simple setup only positions change since there is no angular
  // velocities nor external torques.
  plant_->SetFloatingBaseBodyPoseInWorldFrame(next_context.get(), *sphere1_,
                                              RigidTransformd(p_WS1));
  plant_->SetFloatingBaseBodyPoseInWorldFrame(next_context.get(), *sphere2_,
                                              RigidTransformd(p_WS2));
  plant_->SetFreeBodySpatialVelocity(next_context.get(), *sphere1_, V_WS);
  plant_->SetFreeBodySpatialVelocity(next_context.get(), *sphere2_, V_WS);

  // Obtain the expected value of the state.
  const VectorXd& x_next_expected = next_context->get_discrete_state().value();

  // Perform a discrete update, on the original context_.
  std::unique_ptr<systems::DiscreteValues<double>> updates =
      diagram_->AllocateDiscreteVariables();
  contact_manager_->CalcDiscreteValues(*plant_context_, updates.get());
  ASSERT_EQ(updates->num_groups(), 1);
  const VectorXd x_next = updates->value();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(x_next, x_next_expected, kEps,
                              MatrixCompareType::relative));
}

// This test verifies contact results in the equilibrium configuration.
TEST_P(RigidBodyOnCompliantGround, VerifyContactResultsEquilibriumPosition) {
  const double kTolerance = 1e-15;

  ContactResults<double> contact_results;
  manager_->CalcContactResults(*plant_context_, &contact_results);
  const ContactTestConfig& config = GetParam();

  // Body should be in equilibrium so we expect the contact force to oppose
  // gravity.
  const Vector3d expected_contact_force =
      -kMass_ * plant_->gravity_field().gravity_vector();

  if (config.point_contact) {
    // Test point contact.
    EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
    EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 0);

    const PointPairContactInfo<double>& contact_info =
        contact_results.point_pair_contact_info(0);

    // PointPairContactInfo expresses the contact force acting on body B of the
    // pair. Flip the sign of the contact force if `body_` is not body B.
    const double scale =
        (contact_info.bodyB_index() == body_->index() ? 1 : -1);

    EXPECT_TRUE(CompareMatrices(scale * contact_info.contact_force(),
                                expected_contact_force, kTolerance));
    EXPECT_NEAR(contact_info.contact_point().z(),
                CalcEquilibriumZPosition() - kPointContactSphereRadius_,
                kTolerance);
    EXPECT_NEAR(contact_info.slip_speed(), 0, kEps);
    EXPECT_NEAR(contact_info.separation_speed(), 0, kEps);
  } else {
    // Test hydroelastic contact.
    EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
    EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);

    const HydroelasticContactInfo<double>& contact_info =
        contact_results.hydroelastic_contact_info(0);

    // HydroelasticContactInfo expresses the contact force acting on body A of
    // the pair. Flip the sign of the contact force if `body_` is not body A.
    BodyIndex body_A_index =
        CompliantContactManagerTester::FindBodyByGeometryId(
            *manager_, contact_info.contact_surface().id_M());
    const double scale = (body_A_index == body_->index() ? 1 : -1);
    const SpatialForce<double> F_Ac_W = scale * contact_info.F_Ac_W();

    EXPECT_TRUE(F_Ac_W.IsApprox(
        SpatialForce<double>(Vector3d::Zero(), expected_contact_force),
        kTolerance));
  }
}

TEST_P(RigidBodyOnCompliantGround, VerifyContactResultsBodyInStiction) {
  const double kTolerance = 1e-5;
  const ContactTestConfig& config = GetParam();

  ApplyTangentialForceForBodyInStiction();

  // Simulate the plant for 50 time steps, long enough to reach steady state.
  Simulate(50);

  ContactResults<double> contact_results;
  manager_->CalcContactResults(*plant_context_, &contact_results);

  // Body should be in stiction so we expect the contact force to oppose both
  // gravity and the externally applied tangential force.
  const Vector3d expected_contact_force =
      -(kTangentialStictionForce_ +
        kMass_ * plant_->gravity_field().gravity_vector());

  if (config.point_contact) {
    // Test point contact.
    EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
    EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 0);

    const PointPairContactInfo<double>& contact_info =
        contact_results.point_pair_contact_info(0);

    // PointPairContactInfo reports the contact force acting on body B of the
    // pair. Flip the sign of the contact force if `body_` is not body B.
    const double scale =
        (contact_info.bodyB_index() == body_->index() ? 1 : -1);

    EXPECT_TRUE(CompareMatrices(scale * contact_info.contact_force(),
                                expected_contact_force, kTolerance));
  } else {
    // Test hydroelastic contact.
    EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
    EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);

    const HydroelasticContactInfo<double>& contact_info =
        contact_results.hydroelastic_contact_info(0);

    // HydroelasticContactInfo reports the contact force acting on body A of
    // the pair. Flip the sign of the contact force if `body_` is not body A.
    BodyIndex body_A_index =
        CompliantContactManagerTester::FindBodyByGeometryId(
            *manager_, contact_info.contact_surface().id_M());
    const double scale = (body_A_index == body_->index() ? 1 : -1);
    const SpatialForce<double> F_Ac_W = scale * contact_info.F_Ac_W();

    EXPECT_TRUE(CompareMatrices(F_Ac_W.translational(), expected_contact_force,
                                kTolerance));
    EXPECT_TRUE(
        CompareMatrices(F_Ac_W.rotational(), Vector3d::Zero(), kTolerance));
  }
}

TEST_P(RigidBodyOnCompliantGround, VerifyContactResultsBodyInSlip) {
  const ContactTestConfig& config = GetParam();

  ApplyTangentialForceForBodyInSlip();

  // Simulate the plant for 10 time steps, long enough to reach steady state.
  Simulate(10);

  ContactResults<double> contact_results;
  manager_->CalcContactResults(*plant_context_, &contact_results);

  // For this case the friction force must be on the friction cone. For TAMSI
  // accuracy of this prediction depends on the accuracy of the slip speed,
  // which in turn depends on the convergence tolerance of the solver. For SAP,
  // projections onto the friction cone are accurate to machine epsilon and
  // therefore no error is expected.
  const double kRelativeSlipTolerance =
      config.contact_solver == kDiscreteContactSolverTamsi ? 2.0e-6 : 0.0;

  if (config.point_contact) {
    // Test point contact.
    EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
    EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 0);

    const PointPairContactInfo<double>& contact_info =
        contact_results.point_pair_contact_info(0);

    // PointPairContactInfo reports the contact force acting on body B of the
    // pair. Flip the sign of the contact force if `body_` is not body B.
    const double scale =
        (contact_info.bodyB_index() == body_->index() ? 1 : -1);

    const Vector3d f_Bc_W = scale * contact_info.contact_force();

    // Verify that the contact force lies exactly on the boundary of the
    // friction cone. i.e. |fₜ| = μ|fₙ|
    const double tolerance = kRelativeSlipTolerance * kMu_ * f_Bc_W.z();
    EXPECT_NEAR(-f_Bc_W.x(), kMu_ * f_Bc_W.z(), tolerance);

    // Should be slipping but still at equilibrium penetration.
    EXPECT_GT(contact_info.slip_speed(), 0);
  } else {
    // Test hydroelastic contact.
    EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
    EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);

    const HydroelasticContactInfo<double>& contact_info =
        contact_results.hydroelastic_contact_info(0);

    // HydroelasticContactInfo reports the contact force acting on body A of
    // the pair. Flip the sign of the contact force if `body_` is not body A.
    BodyIndex body_A_index =
        CompliantContactManagerTester::FindBodyByGeometryId(
            *manager_, contact_info.contact_surface().id_M());
    const double scale = (body_A_index == body_->index() ? 1 : -1);
    const SpatialForce<double> F_Ac_W = scale * contact_info.F_Ac_W();

    // Verify that the contact force lies exactly on the boundary of the
    // friction cone. i.e. |fₜ| = μ|fₙ|
    const double tolerance =
        kRelativeSlipTolerance * kMu_ * F_Ac_W.translational().z();
    EXPECT_NEAR(-F_Ac_W.translational().x(), kMu_ * F_Ac_W.translational().z(),
                tolerance);
  }
}

// Setup test cases using point and hydroelastic contact.
std::vector<ContactTestConfig> MakeTestCases() {
  return std::vector<ContactTestConfig>{
      {.description = "HydroelasticContactOnly_SAP",
       .point_contact = false,
       // We verify that the test passes with hydroelastic only.
       .contact_model = ContactModel::kHydroelastic,
       .contact_solver = DiscreteContactSolver::kSap},
      {.description = "HydroelasticContactWithFallback_SAP",
       .point_contact = false,
       .contact_solver = DiscreteContactSolver::kSap},
      {.description = "PointContact_SAP",
       .point_contact = true,
       .contact_solver = DiscreteContactSolver::kSap},
      {.description = "HydroelasticContactWithFallback_TAMSI",
       .point_contact = false,
       .contact_solver = kDiscreteContactSolverTamsi},
      {.description = "HydroelasticContactOnly_TAMSI",
       .point_contact = false,
       // We verify that the test passes with hydroelastic only.
       .contact_model = ContactModel::kHydroelastic,
       .contact_solver = kDiscreteContactSolverTamsi},
      {.description = "PointContact_TAMSI",
       .point_contact = true,
       .contact_solver = kDiscreteContactSolverTamsi},
  };
}

INSTANTIATE_TEST_SUITE_P(CompliantContactManagerTests,
                         RigidBodyOnCompliantGround,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
