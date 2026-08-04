#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_weld_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

/* @file This file tests SapDriver's support for weld constraints.

  Constraints are only supported by the SAP solver. Therefore, to exercise the
  relevant code paths, we arbitrarily choose one contact approximation that uses
  the SAP solver. More precisely, in the unit tests below we call
  set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
  MultibodyPlant used for testing, before constraints are added. */

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::multibody::contact_solvers::internal::SapWeldConstraint;
using drake::systems::Context;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kEps = 2 * std::numeric_limits<double>::epsilon();

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }
};

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  bool bodyA_anchored{};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

constexpr double theta = 0.01 * M_PI;

// Fixture that sets a MultibodyPlant model of two bodies A and B with a
// weld constraint connecting point P on body A and point Q on body B.
class TwoBodiesTest : public ::testing::TestWithParam<TestConfig> {
 public:
  // Makes the model described in the fixture's documentation.
  // @param[in] anchor_bodyA If true, body A will be anchored to the world.
  // Otherwise body A has 6 DOFs as body B does.
  void MakeModel(bool anchor_bodyA) {
    plant_.set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    // Arbitrary inertia values only used by the driver to build a valid contact
    // problem.
    const double mass = 1.5;
    const double radius = 0.1;
    const SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(mass, radius);

    bodyA_ = &plant_.AddRigidBody("A", M_BBcm);
    bodyB_ = &plant_.AddRigidBody("B", M_BBcm);
    if (anchor_bodyA) {
      plant_.WeldFrames(plant_.world_frame(), bodyA_->body_frame());
    }

    plant_.AddWeldConstraint(*bodyA_, X_AP_, *bodyB_, X_BQ_);

    plant_.Finalize();

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single weld constraint.
    EXPECT_EQ(plant_.num_constraints(), 1);

    context_ = plant_.CreateDefaultContext();
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

 protected:
  MultibodyPlant<double> plant_{0.01};  // Discrete model.
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;

  // Parameters of the problem. Arbitrary and non-zero.
  const Vector3d p_AP_{1.0, -2.0, 3.0};
  const Vector3d p_BQ_{-5.0, 4.0, -6.0};
  const RigidTransformd X_AP_{RotationMatrixd::MakeZRotation(theta), p_AP_};
  const RigidTransformd X_BQ_{RotationMatrixd::MakeZRotation(-theta), p_BQ_};
  const Vector3d kOffset_{0.1, 0.2, 0.3};
  const RotationMatrixd kRotationOffset_{
      RollPitchYawd(Vector3d(1.0, 2.0, 3.0))};
};

// This test configures a single weld constraint in variety of ways (causing
// differing numbers of cliques and varying constraint properties). It then
// examines the newly added constraint to confirm that its instantiation
// reflects the specification.
TEST_P(TwoBodiesTest, ConfirmConstraintProperties) {
  const TestConfig& config = GetParam();

  MakeModel(config.bodyA_anchored);

  const int expected_num_velocities = config.bodyA_anchored ? 6 : 12;
  EXPECT_EQ(plant_.num_velocities(), expected_num_velocities);

  // Place Bo at known offset from Ao; this does *not* satisfy the
  // constraint. We'll observe a non-zero value when evaluating the constraint
  // function.
  // Moreover, rotate both bodies A and B an arbitrary non-identity amount.
  if (!config.bodyA_anchored) {
    plant_.SetFloatingBaseBodyPoseInWorldFrame(
        context_.get(), *bodyA_,
        RigidTransformd(kRotationOffset_, Vector3d::Zero()));
  }
  plant_.SetFloatingBaseBodyPoseInWorldFrame(
      context_.get(), *bodyB_, RigidTransformd(kRotationOffset_, kOffset_));
  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // Verify the expected number of constraints and equations for a single
  // weld constraint.
  EXPECT_EQ(problem.num_constraints(), 1);
  EXPECT_EQ(problem.num_constraint_equations(), 6);

  const auto* constraint = dynamic_cast<const SapWeldConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapWeldConstraint as expected.
  ASSERT_NE(constraint, nullptr);

  // One clique if body A is anchored, two if not.
  const int expected_num_cliques = config.bodyA_anchored ? 1 : 2;
  EXPECT_EQ(constraint->num_cliques(), expected_num_cliques);
  if (expected_num_cliques == 1) {
    EXPECT_EQ(constraint->first_clique(), 1);  // i.e., bodyB's tree
    EXPECT_THROW(constraint->second_clique(), std::exception);
  } else {
    EXPECT_EQ(constraint->first_clique(), 0);
    EXPECT_EQ(constraint->second_clique(), 1);
  }

  // Verify parameters.
  const SapHolonomicConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.num_constraint_equations(), 6);
  // bi-lateral constraint with no impulse bounds.
  EXPECT_EQ(p.impulse_lower_limits(), -kInfinity * Vector6d::Ones());
  EXPECT_EQ(p.impulse_upper_limits(), kInfinity * Vector6d::Ones());
  EXPECT_EQ(p.stiffnesses(), kInfinity * Vector6d::Ones());
  EXPECT_EQ(p.relaxation_times(), Vector6d::Zero());

  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(p.beta(), 0.1);

  // The constraint function for a weld constraint is defined as:
  //   g = (a_PQ_W, p_PQ_W)
  // a_PQ_W = θ⋅k is the axis-angle representation of of the relative
  // orientation between frames P and Q, expressed the world frame.

  const Vector6d& g = constraint->constraint_function();
  const RigidTransformd& X_WA = plant_.EvalBodyPoseInWorld(*context_, *bodyA_);
  const RigidTransformd& X_WB = plant_.EvalBodyPoseInWorld(*context_, *bodyB_);
  const RigidTransformd X_WP = X_WA * X_AP_;
  const RigidTransformd X_WQ = X_WB * X_BQ_;

  const Vector3d p_PQ_W = X_WQ.translation() - X_WP.translation();
  EXPECT_TRUE(CompareMatrices(g.template tail<3>(), p_PQ_W));

  const RotationMatrixd R_PQ =
      X_WP.rotation().InvertAndCompose(X_WQ.rotation());
  const Eigen::AngleAxis<double> a = R_PQ.ToAngleAxis();
  const Vector3d a_PQ_W = X_WP.rotation() * (a.angle() * a.axis());
  EXPECT_TRUE(CompareMatrices(g.template head<3>(), a_PQ_W));

  // Verify the constraint Jacobians (based on number of cliques).
  // We know by construction that the 6 generalized velocities for a floating
  // body, A, are laid out in the same order as V_WA. We can express the
  // Jacobian of the constraint with respect to each body's spatial velocity:
  //
  // g = (a_PQ_W, p_PQ_W)
  //
  // d(g)/dt = V_W_AmBm (as documented in the class)
  //
  // V_W_AmBm = V_WBm_N - V_WAm
  //
  //        =   (w_WB, v_WB + w_WB x p_BBm_W)
  //          - (w_WA, v_WA + w_WA x p_AAm_W)
  //
  //        =   (w_WB, v_WB - p_BBm_W x w_WB)
  //          - (w_WA, v_WA - p_AAm_W x w_WA)
  //
  //        = [       [I]    0 ]          [       [I]    0 ]
  //          [-[p_BBm_W]ₓ [I] ] ⋅ V_WB - [-[p_AAm_W]ₓ [I] ] ⋅ V_WA
  //
  //        = J_B ⋅ V_WB - J_A ⋅ V_WA
  const Vector3d p_AAm_W = constraint->kinematics().p_AP_W() +
                           0.5 * constraint->kinematics().p_PoQo_W();
  const Vector3d p_BBm_W = constraint->kinematics().p_BQ_W() -
                           0.5 * constraint->kinematics().p_PoQo_W();
  const Matrix3d I3 = Matrix3d::Identity();
  // clang-format off
  const Matrix3d p_AAm_Wx =
      (Matrix3d() <<           0, -p_AAm_W(2),  p_AAm_W(1),
                      p_AAm_W(2),           0, -p_AAm_W(0),
                     -p_AAm_W(1),  p_AAm_W(0),           0).finished();
  const Matrix3d p_BBm_Wx =
      (Matrix3d() <<           0, -p_BBm_W(2),  p_BBm_W(1),
                      p_BBm_W(2),           0, -p_BBm_W(0),
                     -p_BBm_W(1),  p_BBm_W(0),           0).finished();
  // clang-format on
  if (expected_num_cliques == 1) {
    const MatrixXd& J = constraint->first_clique_jacobian().MakeDenseMatrix();
    // clang-format off
      const MatrixXd J_expected =
        (MatrixXd(6, 6) <<         I3, Matrix3d::Zero(),
                            -p_BBm_Wx,               I3).finished();
    // clang-format on
    EXPECT_TRUE(
        CompareMatrices(J, J_expected, 8 * kEps, MatrixCompareType::relative));
  } else {
    const MatrixXd& Ja = constraint->first_clique_jacobian().MakeDenseMatrix();
    // clang-format off
    const MatrixXd Ja_expected =
      -(MatrixXd(6, 6) <<         I3, Matrix3d::Zero(),
                           -p_AAm_Wx,               I3).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(Ja, Ja_expected, 8 * kEps,
                                MatrixCompareType::relative));

    const MatrixXd& Jb = constraint->second_clique_jacobian().MakeDenseMatrix();
    // clang-format off
      const MatrixXd Jb_expected =
        (MatrixXd(6, 6) <<        I3, Matrix3d::Zero(),
                           -p_BBm_Wx,               I3).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(Jb, Jb_expected, 8 * kEps,
                                MatrixCompareType::relative));
  }
}

std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "SingleClique", .bodyA_anchored = true},
      {.description = "TwoCliques", .bodyA_anchored = false},
  };
}

INSTANTIATE_TEST_SUITE_P(SapWeldConstraintTests, TwoBodiesTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

GTEST_TEST(WeldConstraintsTests, VerifyIdMapping) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  const RigidTransformd X_AP(Vector3d(1, 2, 3));
  const RigidTransformd X_BQ(Vector3d(4, 5, 6));
  MultibodyConstraintId weld_id =
      plant.AddWeldConstraint(bodyA, X_AP, bodyB, X_BQ);
  const WeldConstraintSpec& weld_spec =
      plant.get_weld_constraint_specs(weld_id);
  EXPECT_EQ(weld_spec.id, weld_id);
  EXPECT_EQ(weld_spec.body_A, bodyA.index());
  EXPECT_EQ(weld_spec.body_B, bodyB.index());
  EXPECT_TRUE(weld_spec.X_AP.IsExactlyEqualTo(X_AP));
  EXPECT_TRUE(weld_spec.X_BQ.IsExactlyEqualTo(X_BQ));

  const std::map<MultibodyConstraintId, WeldConstraintSpec>& weld_specs =
      plant.get_weld_constraint_specs();
  ASSERT_EQ(ssize(weld_specs), 1);

  const MultibodyConstraintId weld_id_from_map = weld_specs.begin()->first;
  const WeldConstraintSpec& weld_spec_from_map = weld_specs.begin()->second;

  // Check the id in the map matches the one returned.
  EXPECT_EQ(weld_id, weld_id_from_map);

  // Check that the one spec in the map is equal to `weld_spec`.
  EXPECT_EQ(weld_spec.id, weld_spec_from_map.id);
  EXPECT_EQ(weld_spec.body_A, weld_spec_from_map.body_A);
  EXPECT_EQ(weld_spec.body_B, weld_spec_from_map.body_B);
  EXPECT_TRUE(weld_spec.X_AP.IsExactlyEqualTo(weld_spec_from_map.X_AP));
  EXPECT_TRUE(weld_spec.X_BQ.IsExactlyEqualTo(weld_spec_from_map.X_BQ));

  // Throw on id to wrong constraint specs type.
  EXPECT_THROW(plant.get_coupler_constraint_specs(weld_id), std::exception);
  EXPECT_THROW(plant.get_ball_constraint_specs(weld_id), std::exception);
}

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
GTEST_TEST(BallConstraintTests, FailOnTAMSI) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(
      DiscreteContactApproximation::kTamsi);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(plant.AddWeldConstraint(bodyA, RigidTransformd(),
                                                      bodyB, RigidTransformd()),
                              ".*TAMSI does not support weld constraints.*");
}
#pragma GCC diagnostic pop

GTEST_TEST(WeldConstraintTests, FailOnContinuous) {
  MultibodyPlant<double> plant{0.0};
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyB,
                          RigidTransformd{Vector3d{0, 0, 0}});
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalTimeDerivatives(*context),
                              ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.get_body_spatial_accelerations_output_port()
          .Eval<std::vector<SpatialAcceleration<double>>>(*context),
      ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.get_generalized_acceleration_output_port()
          .Eval<std::vector<SpatialAcceleration<double>>>(*context),
      ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.get_reaction_forces_output_port().Eval(*context),
      ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcCenterOfMassTranslationalAccelerationInWorld(*context),
      ".*continuous.*not.*support.*constraints.*");
}

GTEST_TEST(WeldConstraintTests, FailOnFinalized) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyB,
                              RigidTransformd{Vector3d{0, 0, 0}}),
      ".*Post-finalize calls to 'AddWeldConstraint\\(\\)' are not allowed.*");
}

GTEST_TEST(WeldConstraintTests, FailOnSameBody) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyA,
                              RigidTransformd{Vector3d{0, 0, 0}}),
      ".*Invalid set of parameters for constraint between bodies 'A' and "
      "'A'.*");
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
