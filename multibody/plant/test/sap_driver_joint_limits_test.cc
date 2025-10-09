#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

/* @file This file tests SapDriver's support for joint limits.

  Constraints are only supported by the SAP solver. Therefore, to exercise the
  relevant code paths, we arbitrarily choose one contact approximation that uses
  the SAP solver. More precisely, in the unit tests below we call
  set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
  MultibodyPlant used for testing, before constraints are added. */

using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::multibody::contact_solvers::internal::SapLimitConstraint;
using drake::multibody::internal::DiscreteContactPair;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace internal {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }

  static VectorXd CalcFreeMotionVelocities(const SapDriver<double>& driver,
                                           const Context<double>& context) {
    VectorXd v_star(driver.plant().num_velocities());
    driver.CalcFreeMotionVelocities(context, &v_star);
    return v_star;
  }

  static std::vector<MatrixXd> CalcLinearDynamicsMatrix(
      const SapDriver<double>& driver, const Context<double>& context) {
    std::vector<MatrixXd> A;
    driver.CalcLinearDynamicsMatrix(context, &A);
    return A;
  }
};

// Fixture to set up a Kuka iiwa arm model with a Schunk wsg gripper and welded
// to the world at the base link. This fixture is used to stress test the
// implementation of SapDriver with a model of practical relevance to robotics.
// In particular, we unit test the implementation of damping, joint limits and
// coupler constraints.
class KukaIiwaArmTests : public ::testing::Test {
 public:
  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

  // Enum used to specify how we'd like to initialize the state for limit
  // constraints unit tests.
  enum class InitializePositionAt {
    BelowLowerLimit,  // q₀ < qₗ
    // Current position is above limit, though predicted position is below.
    AboveLowerLimitThoughPredictionBelow,  // q₀+δt⋅v₀ < qₗ < q₀
    AboveUpperLimit,                       // q₀ > qᵤ
    // Current position is below limit, though predicted position is above.
    BelowUpperLimitThoughPredictionAbove,  // q₀ < qᵤ < q₀+δt⋅v₀
    WellWithinLimits,  // Both q0 and q₀ + δt⋅v₀ are in (qₗ, qᵤ)
  };

  // Sets up model of the Kuka iiwa arm with Schunk gripper and allocate context
  // resources. The model includes reflected inertias. Input ports are fixed to
  // arbitrary non-zero values.
  void SetSingleRobotModel() {
    // Only SAP supports the modeling of constraints.
    plant_.set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    // Load robot model from files.
    const std::vector<ModelInstanceIndex> models = SetUpArmModel(&plant_);
    plant_.Finalize();

    // The model has a single coupler constraint.
    EXPECT_EQ(plant_.num_constraints(), 1);

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single robot and gripper. A single coupler constraint to
    // model the gripper.
    EXPECT_EQ(plant_.num_constraints(), 1);

    context_ = plant_.CreateDefaultContext();
    SetArbitraryNonZeroActuation(plant_, models[0], models[1], context_.get());
    SetArbitraryState(plant_, context_.get());
  }

  // Sets up model of the Kuka iiwa arm with Schunk gripper. The model includes
  // reflected inertias. The gripper is modeled with a coupler constraint.
  std::vector<ModelInstanceIndex> SetUpArmModel(
      MultibodyPlant<double>* plant) const {
    std::vector<ModelInstanceIndex> models = LoadIiwaWithGripper(plant);
    AddInReflectedInertia(plant, models, kRotorInertias, kGearRatios);

    // Constrain the gripper fingers to be coupled.
    const ModelInstanceIndex gripper_model = models[1];
    const Joint<double>& left_finger_slider =
        plant->GetJointByName("left_finger_sliding_joint", gripper_model);
    const Joint<double>& right_finger_slider =
        plant->GetJointByName("right_finger_sliding_joint", gripper_model);
    // While for a typical gripper most likely the gear ratio is one and the
    // offset is zero, here we use an arbitrary set of values to verify later on
    // in the test that the manager created a constraint consistent with these
    // numbers.
    plant->AddCouplerConstraint(left_finger_slider, right_finger_slider,
                                kCouplerGearRatio, kCouplerOffset);

    return models;
  }

  // The manager solves free motion velocities using a discrete scheme with
  // implicit joint dissipation. That is, it solves the momentum balance:
  //   m(v) = (M + dt⋅D)⋅(v-v₀) - dt⋅k(x₀)
  // where k(x₀) are all the non-constraint forces such as Coriolis terms and
  // external actuation, evaluated at the previous state x₀.
  // The dynamics matrix is defined as:
  //   A = ∂m/∂v = (M + dt⋅D)
  // This method computes A, including the contribution to implicit damping.
  MatrixXd CalcLinearDynamicsMatrixIncludingImplicitDampingContribution()
      const {
    const int nv = plant_.num_velocities();
    MatrixXd A(nv, nv);
    plant_.CalcMassMatrix(*context_, &A);
    A.diagonal() +=
        plant_.time_step() * plant_.EvalJointDampingCache(*context_);
    return A;
  }

  // Initializes `context` to store arbitrary values of state that abide by the
  // given specification in `limits_specification`. This allows us to test how
  // the manager adds constraints to the problem at different configurations.
  // limits_specification are indexed by joint dofs.
  // TODO(amcastro-tri): our testing strategy using these specifications can be
  // further improved as discussed in the review of #17083, tracked in #17137.
  void SetArbitraryStateWithLimitsSpecification(
      const MultibodyPlant<double>& plant,
      const std::vector<InitializePositionAt>& limits_specification,
      Context<double>* context) {
    // Arbitrary positive slop used for positions.
    const double kPositiveDeltaQ = M_PI / 10.0;
    const double dt = plant.time_step();
    VectorXd v0(plant.num_velocities());
    VectorXd q0(plant.num_positions());

    for (JointIndex joint_index : plant.GetJointIndices()) {
      const Joint<double>& joint = plant.get_joint(joint_index);

      if (joint.num_velocities() == 1) {  // skip welds in the model.
        const int v_index = joint.velocity_start();
        const InitializePositionAt limit_spec = limits_specification[v_index];
        const double ql = joint.position_lower_limits()[0];
        const double qu = joint.position_upper_limits()[0];

        double joint_q0 = 0.;
        double joint_v0 = 0.;
        switch (limit_spec) {
          case InitializePositionAt::BelowLowerLimit: {
            joint_q0 = ql > 0 ? 0.8 * ql : 1.2 * ql;
            joint_v0 = joint_index * 2.3;  // Arbitrary.
            break;
          }
          case InitializePositionAt::AboveLowerLimitThoughPredictionBelow: {
            // We initialize a state s.t. q₀+δt⋅v₀ < qₗ < q₀.
            joint_q0 = ql + kPositiveDeltaQ;
            const double qp = ql - kPositiveDeltaQ;
            joint_v0 = (qp - joint_q0) / dt;
            break;
          }
          case InitializePositionAt::AboveUpperLimit: {
            joint_q0 = qu > 0 ? 1.2 * qu : 0.8 * qu;
            joint_v0 = joint_index * 2.3;  // Arbitrary.
            break;
          }
          case InitializePositionAt::BelowUpperLimitThoughPredictionAbove: {
            // We initialize a state s.t. q₀ < qᵤ < q₀+δt⋅v₀.
            joint_q0 = qu - kPositiveDeltaQ;
            const double qp = qu + kPositiveDeltaQ;
            joint_v0 = (qp - joint_q0) / dt;
            break;
          }
          case InitializePositionAt::WellWithinLimits: {
            joint_q0 = 0.5 * (ql + qu);  // q in (ql, qu)
            joint_v0 = 0.0;
            break;
          }
          default:
            DRAKE_UNREACHABLE();
        }

        q0(v_index) = joint_q0;
        v0(v_index) = joint_v0;
      }
    }

    plant.SetPositions(context, q0);
    plant.SetVelocities(context, v0);
  }

  // Fixes all input ports to have non-zero actuation.
  void SetArbitraryNonZeroActuation(const MultibodyPlant<double>& plant,
                                    ModelInstanceIndex arm_model,
                                    ModelInstanceIndex gripper_model,
                                    Context<double>* context) const {
    const VectorX<double> tau_arm = VectorX<double>::LinSpaced(
        plant.num_actuated_dofs(arm_model), 10.0, 1000.0);
    const VectorX<double> tau_gripper = VectorX<double>::LinSpaced(
        plant.num_actuated_dofs(gripper_model), 10.0, 1000.0);
    plant.get_actuation_input_port(arm_model).FixValue(context, tau_arm);
    plant.get_actuation_input_port(gripper_model)
        .FixValue(context, tau_gripper);
  }

 private:
  std::vector<ModelInstanceIndex> LoadIiwaWithGripper(
      MultibodyPlant<double>* plant) const {
    DRAKE_DEMAND(plant != nullptr);
    const char kArmUrl[] =
        "package://drake_models/iiwa_description/urdf/iiwa14_no_collision.urdf";
    const char kWsg50Url[] =
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    Parser parser(plant);
    parser.SetAutoRenaming(true);
    ModelInstanceIndex arm_model = parser.AddModelsFromUrl(kArmUrl).at(0);

    // Add the gripper.
    ModelInstanceIndex gripper_model = parser.AddModelsFromUrl(kWsg50Url).at(0);

    const auto& base_body = plant->GetBodyByName("base", arm_model);
    const auto& end_effector = plant->GetBodyByName("iiwa_link_7", arm_model);
    const auto& gripper_body = plant->GetBodyByName("body", gripper_model);
    plant->WeldFrames(plant->world_frame(), base_body.body_frame());
    plant->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());

    return {arm_model, gripper_model};
  }

  void AddInReflectedInertia(MultibodyPlant<double>* plant,
                             const std::vector<ModelInstanceIndex>& models,
                             const VectorX<double>& rotor_inertias,
                             const VectorX<double>& gear_ratios) const {
    DRAKE_DEMAND(plant != nullptr);
    int local_joint_index = 0;
    for (JointActuatorIndex index : plant->GetJointActuatorIndices()) {
      JointActuator<double>& joint_actuator =
          plant->get_mutable_joint_actuator(index);
      if (std::count(models.begin(), models.end(),
                     joint_actuator.model_instance()) > 0) {
        joint_actuator.set_default_rotor_inertia(
            rotor_inertias(local_joint_index));
        joint_actuator.set_default_gear_ratio(gear_ratios(local_joint_index));
        local_joint_index++;
      }
    }
  }

  // Set arbitrary state, though within joint limits.
  void SetArbitraryState(const MultibodyPlant<double>& plant,
                         Context<double>* context) {
    for (JointIndex joint_index : plant.GetJointIndices()) {
      const Joint<double>& joint = plant.get_joint(joint_index);
      // This model only has weld, prismatic, and revolute joints.
      if (joint.type_name() == RevoluteJoint<double>::kTypeName) {
        const RevoluteJoint<double>& revolute_joint =
            dynamic_cast<const RevoluteJoint<double>&>(joint);
        // Arbitrary position within position limits.
        const double ql = revolute_joint.position_lower_limit();
        const double qu = revolute_joint.position_upper_limit();
        const double w = 1. * revolute_joint.velocity_start() /
                         kNumJoints;               // Number in [0,1).
        const double q = w * ql + (1.0 - w) * qu;  // q in (ql, qu)
        revolute_joint.set_angle(context, q);
        // Arbitrary velocity.
        revolute_joint.set_angular_rate(context, 0.5 * joint_index);
        // Set damping.
        revolute_joint.SetDamping(
            context, kJointDamping(revolute_joint.velocity_start()));
      } else if (joint.type_name() == PrismaticJoint<double>::kTypeName) {
        const PrismaticJoint<double>& prismatic_joint =
            dynamic_cast<const PrismaticJoint<double>&>(joint);
        // Arbitrary position within position limits.
        const double ql = prismatic_joint.position_lower_limit();
        const double qu = prismatic_joint.position_upper_limit();
        const double w = 1. * prismatic_joint.velocity_start() /
                         kNumJoints;               // Number in [0,1).
        const double q = w * ql + (1.0 - w) * qu;  // q in (ql, qu)
        prismatic_joint.set_translation(context, q);
        // Arbitrary velocity.
        prismatic_joint.set_translation_rate(context, 0.5 * joint_index);
        // Set damping.
        prismatic_joint.SetDamping(
            context, kJointDamping(prismatic_joint.velocity_start()));
      }
    }
  }

 protected:
  const int kNumJoints = 9;
  const double kTimeStep{0.015};
  const VectorXd kRotorInertias{VectorXd::LinSpaced(kNumJoints, 0.1, 12.0)};
  const VectorXd kGearRatios{VectorXd::LinSpaced(kNumJoints, 1.5, 100.0)};
  const VectorXd kJointDamping{VectorXd::LinSpaced(kNumJoints, 0.3, 30)};
  const double kCouplerGearRatio{-1.5};
  const double kCouplerOffset{3.1};
  MultibodyPlant<double> plant_{kTimeStep};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// This test verifies that the linear dynamics matrix is properly computed
// according to A = ∂m/∂v = (M + dt⋅D).
TEST_F(KukaIiwaArmTests, CalcLinearDynamicsMatrix) {
  SetSingleRobotModel();
  const std::vector<MatrixXd> A =
      SapDriverTest::CalcLinearDynamicsMatrix(sap_driver(), *context_);
  const int nv = plant_.num_velocities();
  MatrixXd Adense = MatrixXd::Zero(nv, nv);
  const SpanningForest& forest =
      CompliantContactManagerTester::get_forest(*manager_);
  for (TreeIndex t(0); t < forest.num_trees(); ++t) {
    const SpanningForest::Tree& tree = forest.trees(t);
    const int tree_start = tree.v_start();
    const int tree_nv = tree.nv();
    Adense.block(tree_start, tree_start, tree_nv, tree_nv) = A[t];
  }
  const MatrixXd Aexpected =
      CalcLinearDynamicsMatrixIncludingImplicitDampingContribution();
  EXPECT_TRUE(
      CompareMatrices(Adense, Aexpected, kEps, MatrixCompareType::relative));
}

// This test verifies that the computation of free motion velocities v*
// correctly include the effect of damping implicitly.
TEST_F(KukaIiwaArmTests, CalcFreeMotionVelocities) {
  SetSingleRobotModel();
  const VectorXd v_star =
      SapDriverTest::CalcFreeMotionVelocities(sap_driver(), *context_);

  MultibodyForces<double> forces(plant_);
  CompliantContactManagerTester::CalcNonContactForces(*manager_, *context_,
                                                      false, &forces);
  const VectorXd zero_vdot = VectorXd::Zero(plant_.num_velocities());
  const VectorXd k0 = -plant_.CalcInverseDynamics(*context_, zero_vdot, forces);

  // A = M + dt*D
  const MatrixXd A =
      CalcLinearDynamicsMatrixIncludingImplicitDampingContribution();
  const VectorXd a = A.ldlt().solve(k0);
  const VectorXd& v0 = plant_.GetVelocities(*context_);
  const VectorXd v_star_expected = v0 + plant_.time_step() * a;

  EXPECT_TRUE(CompareMatrices(v_star, v_star_expected, 5.0 * kEps,
                              MatrixCompareType::relative));
}

// This unit test simply verifies that the manager is loading acceleration
// kinematics with the proper results. The correctness of the computations we
// rely on in this test (computation of accelerations) are tested elsewhere.
TEST_F(KukaIiwaArmTests, CalcAccelerationKinematicsCache) {
  SetSingleRobotModel();
  const VectorXd& v0 = plant_.GetVelocities(*context_);
  ContactSolverResults<double> contact_results;
  manager_->CalcContactSolverResults(*context_, &contact_results);
  const VectorXd a_expected =
      (contact_results.v_next - v0) / plant_.time_step();
  std::vector<SpatialAcceleration<double>> A_WB_expected(plant_.num_bodies());
  plant_.CalcSpatialAccelerationsFromVdot(*context_, a_expected,
                                          &A_WB_expected);

  // Verify CompliantContactManager loads the acceleration kinematics with the
  // proper results.
  AccelerationKinematicsCache<double> ac(
      CompliantContactManagerTester::get_forest(*manager_));
  manager_->CalcAccelerationKinematicsCache(*context_, &ac);
  EXPECT_TRUE(CompareMatrices(ac.get_vdot(), a_expected));
  for (BodyIndex b(0); b < plant_.num_bodies(); ++b) {
    const auto& body = plant_.get_body(b);
    EXPECT_TRUE(ac.get_A_WB(body.mobod_index()).IsApprox(A_WB_expected[b]));
  }
}

TEST_F(KukaIiwaArmTests, LimitConstraints) {
  SetSingleRobotModel();
  // Arbitrary selection of how positions and velocities are initialized.
  std::vector<InitializePositionAt> limits_specification(
      kNumJoints, InitializePositionAt::WellWithinLimits);
  limits_specification[0] = InitializePositionAt::BelowLowerLimit;
  limits_specification[1] = InitializePositionAt::AboveUpperLimit;
  limits_specification[2] =
      InitializePositionAt::AboveLowerLimitThoughPredictionBelow;
  limits_specification[3] =
      InitializePositionAt::BelowUpperLimitThoughPredictionAbove;
  limits_specification[4] = InitializePositionAt::WellWithinLimits;
  limits_specification[5] = InitializePositionAt::BelowLowerLimit;
  limits_specification[6] = InitializePositionAt::AboveUpperLimit;
  limits_specification[7] = InitializePositionAt::WellWithinLimits;
  limits_specification[8] = InitializePositionAt::WellWithinLimits;

  // Three joints are WellWithinLimits.
  const int kNumJointsWithLimits = 6;
  const int kNumConstraintEquations = 6;
  SetArbitraryStateWithLimitsSpecification(plant_, limits_specification,
                                           context_.get());

  const DiscreteContactData<DiscreteContactPair<double>>& discrete_pairs =
      manager_->EvalDiscreteContactPairs(*context_);
  const int num_contacts = discrete_pairs.size();
  // We are assuming there is no contact. Assert this.
  ASSERT_EQ(num_contacts, 0);

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // This model has no contact. We expect the number of constraints and
  // equations be consistent with limits_specification defined above.
  // Recall the model has one additional constraint to model the coupler between
  // the gripper fingers.
  EXPECT_EQ(problem.num_constraints(), kNumJointsWithLimits + 1);
  EXPECT_EQ(problem.num_constraint_equations(), kNumConstraintEquations + 1);

  // In this model we clearly have single tree, the arm with its gripper.
  const int tree_expected = 0;

  int num_constraints = 0;  // count number of constraints visited.
  // The manager adds limit constraints in the order joints are specified.
  // Therefore we verify the limit constraint for each joint.
  for (JointIndex joint_index : plant_.GetJointIndices()) {
    const Joint<double>& joint = plant_.get_joint(joint_index);
    if (joint.num_velocities() == 1) {
      const int v_index = joint.velocity_start();
      const InitializePositionAt limit_spec = limits_specification[v_index];

      if (limit_spec != InitializePositionAt::WellWithinLimits) {
        // Get limit constraint for the specific joint.
        const auto* constraint =
            dynamic_cast<const SapLimitConstraint<double>*>(
                &problem.get_constraint(num_constraints++));
        // Since the spec is not WellWithinLimits, we expect a constraint added.
        ASSERT_NE(constraint, nullptr);

        // Limit constraints always apply to a single tree in the multibody
        // forest.
        EXPECT_EQ(constraint->num_cliques(), 1);

        // There is a single tree in this model, the arm with gripper.
        EXPECT_EQ(constraint->first_clique(), tree_expected);

        EXPECT_EQ(constraint->clique_dof(), v_index);

        // Each constraints acts on the same tree (the arm+gripper) with a total
        // of kNumJoint DOFs in that tree.
        EXPECT_EQ(constraint->first_clique_jacobian().cols(), kNumJoints);

        // Verify the plant's state is consistent with the constraint's state.
        const double q0 = joint.GetOnePosition(*context_);
        EXPECT_EQ(constraint->position(), q0);

        // N.B. Default values implemented in
        // SapDriver::AddLimitConstraints(), keep these values in
        // sync.
        const SapLimitConstraint<double>::Parameters& params =
            constraint->parameters();
        EXPECT_EQ(params.stiffness(), 1.0e12);
        EXPECT_EQ(params.dissipation_time_scale(), plant_.time_step());
        EXPECT_EQ(params.beta(), 0.1);

        const bool lower_limit_expected =
            limit_spec == InitializePositionAt::BelowLowerLimit ||
            limit_spec ==
                InitializePositionAt::AboveLowerLimitThoughPredictionBelow;
        const bool upper_limit_expected =
            limit_spec == InitializePositionAt::AboveUpperLimit ||
            limit_spec ==
                InitializePositionAt::BelowUpperLimitThoughPredictionAbove;

        const int expected_num_equations =
            lower_limit_expected && upper_limit_expected ? 2 : 1;
        EXPECT_EQ(constraint->num_constraint_equations(),
                  expected_num_equations);
        EXPECT_EQ(constraint->first_clique_jacobian().rows(),
                  expected_num_equations);

        const double kInf = std::numeric_limits<double>::infinity();
        const double ql_expected =
            lower_limit_expected ? joint.position_lower_limits()[0] : -kInf;
        const double qu_expected =
            upper_limit_expected ? joint.position_upper_limits()[0] : kInf;

        EXPECT_EQ(params.lower_limit(), ql_expected);
        EXPECT_EQ(params.upper_limit(), qu_expected);
      }
    }
  }
  EXPECT_EQ(num_constraints, kNumJointsWithLimits);
}

// This test verifies that the manager properly added holonomic constraints for
// the coupler constraints specified in the MultibodyPlant model.
TEST_F(KukaIiwaArmTests, CouplerConstraints) {
  // Only SAP supports the modeling of constraints.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);

  // Load two robot models.
  std::vector<ModelInstanceIndex> arm_gripper1 = SetUpArmModel(&plant_);
  std::vector<ModelInstanceIndex> arm_gripper2 = SetUpArmModel(&plant_);

  // For testing purposes, we'll add a coupler constraint between joints in two
  // different arms.
  const Joint<double>& arm1_joint3 =
      plant_.GetJointByName("iiwa_joint_3", arm_gripper1[0]);
  const Joint<double>& arm2_joint6 =
      plant_.GetJointByName("iiwa_joint_6", arm_gripper2[0]);
  plant_.AddCouplerConstraint(arm1_joint3, arm2_joint6, kCouplerGearRatio,
                              kCouplerOffset);

  plant_.Finalize();

  // There should be three coupler constraints: one for each gripper and a third
  // one between the two arms.
  EXPECT_EQ(plant_.num_constraints(), 3);

  // Set manager using the experimental API so that we can bring it to scope.
  auto owned_contact_manager =
      std::make_unique<CompliantContactManager<double>>();
  manager_ = owned_contact_manager.get();
  plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));

  context_ = plant_.CreateDefaultContext();
  SetArbitraryNonZeroActuation(plant_, arm_gripper1[0], arm_gripper1[1],
                               context_.get());
  SetArbitraryNonZeroActuation(plant_, arm_gripper2[0], arm_gripper2[1],
                               context_.get());

  // Specify a state in which all kNumJoints are within limits so that we know
  // the contact problem has no limit constraints.
  std::vector<InitializePositionAt> limits_specification(
      2 * kNumJoints, InitializePositionAt::WellWithinLimits);
  SetArbitraryStateWithLimitsSpecification(plant_, limits_specification,
                                           context_.get());

  // We are assuming there is no contact. Assert this.
  const DiscreteContactData<DiscreteContactPair<double>>& discrete_pairs =
      manager_->EvalDiscreteContactPairs(*context_);
  const int num_contacts = discrete_pairs.size();
  ASSERT_EQ(num_contacts, 0);

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // This model has no contact and the configuration is set to be within joint
  // limits. Therefore we expect the problem to have the three couple
  // constraints we added to the model.
  EXPECT_EQ(problem.num_constraints(), 3);
  EXPECT_EQ(problem.num_constraint_equations(), 3);

  std::vector<std::pair<JointIndex, JointIndex>> coupler_joints;
  // Coupler on first robot's gripper.
  coupler_joints.push_back({
      plant_.GetJointByName("left_finger_sliding_joint", arm_gripper1[1])
          .index(),
      plant_.GetJointByName("right_finger_sliding_joint", arm_gripper1[1])
          .index(),
  });

  // Coupler on second robot's gripper.
  coupler_joints.push_back({
      plant_.GetJointByName("left_finger_sliding_joint", arm_gripper2[1])
          .index(),
      plant_.GetJointByName("right_finger_sliding_joint", arm_gripper2[1])
          .index(),
  });

  // Coupler between the two robots.
  coupler_joints.push_back({
      plant_.GetJointByName("iiwa_joint_3", arm_gripper1[0]).index(),
      plant_.GetJointByName("iiwa_joint_6", arm_gripper2[0]).index(),
  });

  // Verify each of the coupler constraints.
  for (int i = 0; i < 3; ++i) {
    const auto* constraint =
        dynamic_cast<const SapHolonomicConstraint<double>*>(
            &problem.get_constraint(i));

    // Verify it is a SapHolonomicConstraint as expected.
    ASSERT_NE(constraint, nullptr);

    // There are two cliques in this model, one for each robot arm.
    const int num_cliques = i == 2 ? 2 : 1;
    EXPECT_EQ(constraint->num_cliques(), num_cliques);
    const int first_clique = i == 1 ? 1 : 0;
    EXPECT_EQ(constraint->first_clique(), first_clique);
    if (i == 2) {
      // constraint between the two robots.
      EXPECT_EQ(constraint->second_clique(), 1);
    }

    const Joint<double>& joint0 = plant_.get_joint(coupler_joints[i].first);
    const Joint<double>& joint1 = plant_.get_joint(coupler_joints[i].second);

    // Verify the value of the constraint function.
    const double q0 = joint0.GetOnePosition(*context_);
    const double q1 = joint1.GetOnePosition(*context_);
    const Vector1d g0_expected(q0 - kCouplerGearRatio * q1 - kCouplerOffset);
    const VectorXd& g0 = constraint->constraint_function();
    EXPECT_EQ(g0, g0_expected);

    if (i < 2) {
      // For the grippers, fingers are the last two DOFs in their tree.
      const int left_index = 7;
      const int right_index = 8;
      const MatrixXd J_expected =
          (VectorXd::Unit(kNumJoints, left_index) -
           kCouplerGearRatio * VectorXd::Unit(kNumJoints, right_index))
              .transpose();
      EXPECT_EQ(constraint->first_clique_jacobian().MakeDenseMatrix(),
                J_expected);
    } else {
      // The third constraint couples the two robot arms.
      const MatrixXd J0_expected =
          VectorXd::Unit(kNumJoints, 2 /* third joint. */).transpose();
      const MatrixXd J1_expected =
          -kCouplerGearRatio *
          VectorXd::Unit(kNumJoints, 5 /* sixth joint. */).transpose();
      EXPECT_EQ(constraint->first_clique_jacobian().MakeDenseMatrix(),
                J0_expected);
      EXPECT_EQ(constraint->second_clique_jacobian().MakeDenseMatrix(),
                J1_expected);
    }

    // N.B. Default values implemented in
    // SapDriver::AddCouplerConstraints(), keep these values in sync.
    const Vector1d kInfinity =
        Vector1d::Constant(std::numeric_limits<double>::infinity());
    const SapHolonomicConstraint<double>::Parameters& params =
        constraint->parameters();
    EXPECT_EQ(params.impulse_lower_limits(), -kInfinity);
    EXPECT_EQ(params.impulse_upper_limits(), kInfinity);
    EXPECT_EQ(params.stiffnesses(), kInfinity);
    EXPECT_EQ(params.relaxation_times(), Vector1d::Zero());
    EXPECT_EQ(params.beta(), 0.1);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
