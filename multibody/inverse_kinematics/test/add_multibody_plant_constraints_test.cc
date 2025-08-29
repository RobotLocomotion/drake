#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using systems::Context;

bool SnoptSolverUnavailable() {
  return !(solvers::SnoptSolver::is_available() &&
           solvers::SnoptSolver::is_enabled());
}

class AddMultibodyPlantConstraintsTest : public ::testing::Test {
 public:
  void SetUp() override {
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);
    plant_->SetUseSampledOutputPorts(false);

    body_A_ = &plant_->AddRigidBody(
        "body_A", SpatialInertia<double>::SolidBoxWithMass(1, 1, 1, 1));
    body_B_ = &plant_->AddRigidBody(
        "body_B", SpatialInertia<double>::SolidBoxWithMass(1, 1, 1, 1));
    world_A_ = &plant_->AddJoint<RevoluteJoint>(
        "world_A", plant_->world_body(), RigidTransformd(Vector3d(-1, 0, 0)),
        *body_A_, RigidTransformd(), Vector3d::UnitZ());
    // Add and then immediately remove a joint so that the joint indices do not
    // correspond to the position indices.
    plant_->RemoveJoint(plant_->AddJoint<RevoluteJoint>(
        "temp", *body_A_, RigidTransformd(Vector3d(1, 0, 0)), *body_B_,
        RigidTransformd(), Vector3d::UnitZ()));
    A_B_ = &plant_->AddJoint<RevoluteJoint>(
        "A_B", *body_A_, RigidTransformd(Vector3d(1, 0, 0)), *body_B_,
        RigidTransformd(), Vector3d::UnitZ());
  }

  void CheckConstraints(int expected_num_constraints = 2, double tol = 1e-6) {
    if (!plant_->is_finalized()) {
      plant_->Finalize();
    }
    plant_context_ = plant_->CreateDefaultContext();

    // First demonstrate that these constraints work with MathematicalProgram,
    // and use them to find a valid initial condition.
    solvers::MathematicalProgram prog;
    auto q = prog.NewContinuousVariables(plant_->num_positions());
    VectorXd q0 = VectorXd::LinSpaced(plant_->num_positions(), 0, 1);
    prog.AddQuadraticErrorCost(
        MatrixXd::Identity(plant_->num_positions(), plant_->num_positions()),
        q0, q);

    auto constraints =
        AddMultibodyPlantConstraints(plant_, q, &prog, plant_context_.get());
    EXPECT_EQ(constraints.size(), expected_num_constraints);
    auto result = solvers::Solve(prog);
    EXPECT_TRUE(result.is_success());

    // Now step the dynamics forward and verify that the constraint stays
    // satisfied.
    plant_->SetPositions(plant_context_.get(), result.GetSolution(q));
    VectorXd qn = plant_->EvalUniquePeriodicDiscreteUpdate(*plant_context_)
                      .value()
                      .head(plant_->num_positions());

    prog.SetInitialGuessForAllVariables(qn);
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
  }

 protected:
  std::shared_ptr<MultibodyPlant<double>> plant_{
      std::make_shared<MultibodyPlant<double>>(0.1)};
  std::unique_ptr<Context<double>> plant_context_{};
  const RigidBody<double>* body_A_{nullptr};
  const RigidBody<double>* body_B_{nullptr};
  const RevoluteJoint<double>* world_A_{nullptr};
  const RevoluteJoint<double>* A_B_{nullptr};
};

TEST_F(AddMultibodyPlantConstraintsTest, CouplerConstraint) {
  plant_->AddCouplerConstraint(*world_A_, *A_B_, 2.3);
  CheckConstraints();

  // Confirm that I also could have called the method with plant_context ==
  // nullptr.
  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(plant_->num_positions());
  EXPECT_NO_THROW(AddMultibodyPlantConstraints(plant_, q, &prog));
}

TEST_F(AddMultibodyPlantConstraintsTest, DistanceConstraint) {
  plant_->AddDistanceConstraint(*body_A_, Vector3d(0.0, 1.0, 0.0), *body_B_,
                                Vector3d(0.0, 1.0, 0.0), 1.5);
  CheckConstraints();
}

TEST_F(AddMultibodyPlantConstraintsTest, BallConstraint) {
  plant_->AddBallConstraint(*body_A_, Vector3d(0.0, 2.0, 0.0), *body_B_);
  if (SnoptSolverUnavailable()) {
    // IPOPT is flakey on this test.
    return;
  }
  CheckConstraints();
}

TEST_F(AddMultibodyPlantConstraintsTest, WeldConstraint) {
  const RigidBody<double>& body_C = plant_->AddRigidBody(
      "body_C", SpatialInertia<double>::SolidBoxWithMass(1, 1, 1, 1));
  plant_->AddWeldConstraint(*body_A_, RigidTransformd(Vector3d(1, 2, 3)),
                            body_C, RigidTransformd(Vector3d(4, 5, 6)));
  if (SnoptSolverUnavailable()) {
    // IPOPT is flakey on this test.
    return;
  }
  const int expected_num_constraints =
      4;  // 1 joint limits, 1 unit quaternion, 2 for weld.
  CheckConstraints(expected_num_constraints, /* tol = */ 1e-2);
}

// Tests the interaction between joint locking and constraints, in the simple
// case where floating joints are parented on the world.
GTEST_TEST(AdditionalTests, QuaternionsAndJointLimitsAndLocks1) {
  auto plant = std::make_shared<MultibodyPlant<double>>(0);

  // Create a plant with four bodies.
  const auto& world = plant->world_body();
  const auto M = SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);
  const auto& body1 = plant->AddRigidBody("body1", M);
  const auto& body2 = plant->AddRigidBody("body2", M);
  const auto& body3 = plant->AddRigidBody("body3", M);
  const auto& body4 = plant->AddRigidBody("body4", M);

  // Attach a specific joint to each body:
  // (1) A quaternion floating joint that will not be locked.
  // (2) A quaternion floating joint that we'll lock to its initial position.
  // (3) A revolute joint that will not be locked.
  // (4) A revolute joint that we'll lock to its initial position.
  math::RigidTransform<double> I;
  Eigen::Vector3d X = Eigen::Vector3d::UnitX();
  const auto& joint1 =
      plant->AddJoint<QuaternionFloatingJoint>("joint1", world, I, body1, I);
  const auto& joint2 =
      plant->AddJoint<QuaternionFloatingJoint>("joint2", world, I, body2, I);
  const auto& joint3 =
      plant->AddJoint<RevoluteJoint>("joint3", world, I, body3, I, X);
  const auto& joint4 =
      plant->AddJoint<RevoluteJoint>("joint4", world, I, body4, I, X);
  plant->Finalize();
  auto context = plant->CreateDefaultContext();

  // Leave joint1 unlocked.

  // Lock body2's floating joint to an un-normalized initial value.
  joint2.SetQuaternion(&*context, Eigen::Quaternion<double>(0, 3.0, 0, 0));
  joint2.Lock(&*context);

  // Set limits on joint3, but do not lock it.
  dynamic_cast<RevoluteJoint<double>&>(plant->get_mutable_joint(joint3.index()))
      .set_position_limits(Vector1d{-0.5}, Vector1d{0.5});

  // Lock body4's revolute joint beyond its limit.
  dynamic_cast<RevoluteJoint<double>&>(plant->get_mutable_joint(joint4.index()))
      .set_position_limits(Vector1d{-1}, Vector1d{1});
  joint4.set_angle(&*context, 1.1);
  joint4.Lock(&*context);

  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(plant->num_positions());
  AddMultibodyPlantConstraints(plant, q, &prog, context.get());

  // The initial guess is set for the two quaternion floating joints.
  EXPECT_TRUE(CompareMatrices(
      prog.GetInitialGuess(q.segment(joint1.position_start(), 4)),
      Eigen::Vector4d(1, 0, 0, 0)));
  const Eigen::Vector4d joint2_position(0, 1, 0, 0);
  EXPECT_TRUE(CompareMatrices(
      prog.GetInitialGuess(q.segment(joint2.position_start(), 4)),
      joint2_position));

  // We only expect one bounding box constraint, which is the joint limits.
  ASSERT_EQ(prog.bounding_box_constraints().size(), 1);
  const solvers::Binding<solvers::BoundingBoxConstraint>& limits =
      prog.bounding_box_constraints().front();

  // joint 1 is unlocked, we expect a unit quaternion constraint and limits of
  // [-1, 1].
  ASSERT_EQ(prog.generic_constraints().size(), 1);
  const solvers::Binding<solvers::Constraint>& unit_quat =
      prog.generic_constraints().front();
  ASSERT_EQ(unit_quat.variables().size(), 4);
  const int j1_start = joint1.position_start();
  EXPECT_EQ(symbolic::Variables(unit_quat.variables()),
            symbolic::Variables(q.segment(j1_start, 4)));
  EXPECT_TRUE(
      CompareMatrices(limits.evaluator()->lower_bound().segment<4>(j1_start),
                      Eigen::Vector4d(-1, -1, -1, -1)));
  EXPECT_TRUE(
      CompareMatrices(limits.evaluator()->upper_bound().segment<4>(j1_start),
                      Eigen::Vector4d(1, 1, 1, 1)));

  // joint2 is locked, so we expect a limits == joint2_position.
  const int j2_start = joint2.position_start();
  EXPECT_TRUE(CompareMatrices(
      limits.evaluator()->lower_bound().segment<4>(j2_start), joint2_position));
  EXPECT_TRUE(CompareMatrices(
      limits.evaluator()->upper_bound().segment<4>(j2_start), joint2_position));

  // joint3 is unlocked, so we expect the joint limits to be enforced.
  const int j3_start = joint3.position_start();
  EXPECT_EQ(limits.evaluator()->lower_bound()[j3_start], -0.5);
  EXPECT_EQ(limits.evaluator()->upper_bound()[j3_start], +0.5);

  // joint4 is locked. Locked revolute joints obey their initial position,
  // ignoring limits.
  const int j4_start = joint4.position_start();
  EXPECT_EQ(limits.evaluator()->lower_bound()[j4_start], 1.1);
  EXPECT_EQ(limits.evaluator()->upper_bound()[j4_start], 1.1);

  // If we don't pass in the context, then we will not get the locked joints.
  solvers::MathematicalProgram prog2;
  auto q2 = prog2.NewContinuousVariables(plant->num_positions());
  AddMultibodyPlantConstraints(plant, q2, &prog2);
  EXPECT_EQ(prog2.linear_equality_constraints().size(), 0);
}

// Tests the interaction between joint locking and constraints, in the unusual
// case where floating joints are parented onto the robot instead of the world.
GTEST_TEST(AdditionalTests, QuaternionsAndJointLimitsAndLocks2) {
  auto plant = std::make_shared<MultibodyPlant<double>>(0);

  // Create a plant with two bodies.
  const auto& world = plant->world_body();
  const auto M = SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);
  const auto& robot = plant->AddRigidBody("robot", M);
  const auto& object = plant->AddRigidBody("object", M);

  // Attach the robot to the world with a revolute joint (with limits).
  // Attach the object to the robot with a floating joint.
  Eigen::Vector3d X = Eigen::Vector3d::UnitX();
  const auto& world_robot =
      plant->AddJoint<RevoluteJoint>("world_robot", world, {}, robot, {}, X);
  dynamic_cast<RevoluteJoint<double>&>(
      plant->get_mutable_joint(world_robot.index()))
      .set_position_limits(Vector1d{-0.5}, Vector1d{0.5});
  const auto& robot_object = plant->AddJoint<QuaternionFloatingJoint>(
      "robot_object", robot, {}, object, {});
  plant->Finalize();
  ASSERT_EQ(plant->num_positions(), 1 + 7);

  // Helper function (lambda) that calls AddMultibodyPlantConstraints and
  // extracts the details of what got added to the MathematicalProgram.
  //
  // Note that for convenience the output is a local variable named "detail"
  // which is overwritten by this lambda.
  struct ProgramDetail {
    VectorXd initial_guess;
    VectorXd lower_bound;
    VectorXd upper_bound;
    bool has_unit_quat_constraint{};
  };
  ProgramDetail detail;
  auto inspect_constraints = [&plant, &detail](Context<double>* context) {
    detail = {};
    solvers::MathematicalProgram prog;
    auto q = prog.NewContinuousVariables(plant->num_positions());
    prog.SetInitialGuessForAllVariables(VectorXd::Zero(q.size()));
    AddMultibodyPlantConstraints(plant, q, &prog, context);
    detail.initial_guess = prog.GetInitialGuess(q);
    // We expect exactly one bounding box constraint, which is the joint limits.
    ASSERT_EQ(prog.bounding_box_constraints().size(), 1);
    const solvers::Binding<solvers::BoundingBoxConstraint>& bounding_box =
        prog.bounding_box_constraints().front();
    ASSERT_TRUE(CheckStructuralEquality(bounding_box.variables(), q));
    detail.lower_bound = bounding_box.evaluator()->lower_bound();
    detail.upper_bound = bounding_box.evaluator()->upper_bound();
    // We expect at most one generic constraint, which is the unit quaternion.
    if (!prog.generic_constraints().empty()) {
      ASSERT_EQ(prog.generic_constraints().size(), 1);
      const solvers::Binding<solvers::Constraint>& generic =
          prog.generic_constraints().front();
      ASSERT_NO_THROW(unused(
          dynamic_cast<const UnitQuaternionConstraint&>(*generic.evaluator())));
      ASSERT_TRUE(CheckStructuralEquality(
          generic.variables(),
          q.segment<4>(plant->GetJointByName("robot_object").position_start())
              .eval()));
      detail.has_unit_quat_constraint = true;
    }
  };

  // In all of the below, our positions vector has 8 positions in this order:
  // - 1 position: revolute joint
  // - 4 positions: floating joint quaternion
  // - 3 positions: floating joint translation
  using Vector8d = Eigen::Vector<double, 8>;

  // With no context, nothing is locked.
  const double inf = std::numeric_limits<double>::infinity();
  inspect_constraints(nullptr);
  EXPECT_EQ(detail.initial_guess, Vector8d(0, 1, 0, 0, 0, 0, 0, 0));
  EXPECT_EQ(detail.lower_bound,
            Vector8d(-0.5, -1, -1, -1, -1, -inf, -inf, -inf));
  EXPECT_EQ(detail.upper_bound,
            Vector8d(+0.5, +1, +1, +1, +1, +inf, +inf, +inf));
  EXPECT_TRUE(detail.has_unit_quat_constraint);

  // With a default a context, nothing is locked.
  auto context = plant->CreateDefaultContext();
  inspect_constraints(context.get());
  EXPECT_EQ(detail.initial_guess, Vector8d(0, 1, 0, 0, 0, 0, 0, 0));
  EXPECT_EQ(detail.lower_bound,
            Vector8d(-0.5, -1, -1, -1, -1, -inf, -inf, -inf));
  EXPECT_EQ(detail.upper_bound,
            Vector8d(+0.5, +1, +1, +1, +1, +inf, +inf, +inf));
  EXPECT_TRUE(detail.has_unit_quat_constraint);

  // Lock the revolute joint.
  context = plant->CreateDefaultContext();
  world_robot.set_angle(context.get(), 1.1);
  world_robot.Lock(context.get());
  inspect_constraints(context.get());
  EXPECT_EQ(detail.initial_guess, Vector8d(1.1, 1, 0, 0, 0, 0, 0, 0));
  EXPECT_EQ(detail.lower_bound,
            Vector8d(1.1, -1, -1, -1, -1, -inf, -inf, -inf));
  EXPECT_EQ(detail.upper_bound,
            Vector8d(1.1, +1, +1, +1, +1, +inf, +inf, +inf));
  EXPECT_TRUE(detail.has_unit_quat_constraint);

  // Lock the floating joint to an un-normalized initial value.
  context = plant->CreateDefaultContext();
  robot_object.SetQuaternion(context.get(),
                             Eigen::Quaternion<double>(0, 3.0, 0, 0));
  robot_object.SetTranslation(context.get(), Vector3d(0.5, 0.25, 0.125));
  robot_object.Lock(context.get());
  inspect_constraints(context.get());
  EXPECT_EQ(detail.initial_guess, Vector8d(0, 0, 1, 0, 0, 0.5, 0.25, 0.125));
  EXPECT_EQ(detail.lower_bound, Vector8d(-0.5, 0, 1, 0, 0, 0.5, 0.25, 0.125));
  EXPECT_EQ(detail.upper_bound, Vector8d(+0.5, 0, 1, 0, 0, 0.5, 0.25, 0.125));
  EXPECT_FALSE(detail.has_unit_quat_constraint);

  // Lock both joints.
  context = plant->CreateDefaultContext();
  world_robot.set_angle(context.get(), 1.1);
  world_robot.Lock(context.get());
  robot_object.SetQuaternion(context.get(),
                             Eigen::Quaternion<double>(0, 3.0, 0, 0));
  robot_object.SetTranslation(context.get(), Vector3d(0.5, 0.25, 0.125));
  robot_object.Lock(context.get());
  inspect_constraints(context.get());
  EXPECT_EQ(detail.initial_guess, Vector8d(1.1, 0, 1, 0, 0, 0.5, 0.25, 0.125));
  EXPECT_EQ(detail.lower_bound, Vector8d(1.1, 0, 1, 0, 0, 0.5, 0.25, 0.125));
  EXPECT_EQ(detail.upper_bound, Vector8d(1.1, 0, 1, 0, 0, 0.5, 0.25, 0.125));
  EXPECT_FALSE(detail.has_unit_quat_constraint);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
