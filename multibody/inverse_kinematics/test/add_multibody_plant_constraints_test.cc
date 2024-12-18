#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"

#include <gtest/gtest.h>

#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;

class AddMultibodyPlantConstraintsTest : public ::testing::Test {
 public:
  void SetUp() override {
    plant_.set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);
    plant_.SetUseSampledOutputPorts(false);

    body_A_ = &plant_.AddRigidBody(
        "body_A", SpatialInertia<double>::SolidBoxWithMass(1, 1, 1, 1));
    body_B_ = &plant_.AddRigidBody(
        "body_B", SpatialInertia<double>::SolidBoxWithMass(1, 1, 1, 1));
    world_A_ = &plant_.AddJoint<RevoluteJoint>(
        "world_A", plant_.world_body(), RigidTransformd(Vector3d(-1, 0, 0)),
        *body_A_, RigidTransformd(), Vector3d::UnitZ());
    A_B_ = &plant_.AddJoint<RevoluteJoint>(
        "A_B", *body_A_, RigidTransformd(Vector3d(1, 0, 0)), *body_B_,
        RigidTransformd(), Vector3d::UnitZ());
  }

  void CheckConstraints(int expected_num_constraints = 1, double tol = 1e-6) {
    if (!plant_.is_finalized()) {
      plant_.Finalize();
    }
    plant_context_ = plant_.CreateDefaultContext();

    // First demonstrate that these constraints work with MathematicalProgram,
    // and use them to find a valid initial condition.
    solvers::MathematicalProgram prog;
    auto q = prog.NewContinuousVariables(plant_.num_positions());
    VectorXd q0 = VectorXd::LinSpaced(plant_.num_positions(), 0, 1);
    prog.AddQuadraticErrorCost(
        MatrixXd::Identity(plant_.num_positions(), plant_.num_positions()), q0,
        q);

    auto constraints =
        AddMultibodyPlantConstraints(plant_, q, &prog, plant_context_.get());
    EXPECT_EQ(constraints.size(), expected_num_constraints);
    auto result = solvers::Solve(prog);
    EXPECT_TRUE(result.is_success());

    // Now step the dynamics forward and verify that the constraint stays
    // satisfied.
    plant_.SetPositions(plant_context_.get(), result.GetSolution(q));
    VectorXd qn = plant_.EvalUniquePeriodicDiscreteUpdate(*plant_context_)
                      .value()
                      .head(plant_.num_positions());

    prog.SetInitialGuessForAllVariables(qn);
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

    result.set_x_val(qn);
    log()->info(
        "Infeasible constraints:\n{}",
        fmt::join(result.GetInfeasibleConstraintNames(prog, tol), "\n"));
  }

 protected:
  MultibodyPlant<double> plant_{0.1};
  std::unique_ptr<systems::Context<double>> plant_context_{};
  const RigidBody<double>* body_A_{nullptr};
  const RigidBody<double>* body_B_{nullptr};
  const RevoluteJoint<double>* world_A_{nullptr};
  const RevoluteJoint<double>* A_B_{nullptr};
};

TEST_F(AddMultibodyPlantConstraintsTest, CouplerConstraint) {
  plant_.AddCouplerConstraint(*world_A_, *A_B_, 2.3);
  CheckConstraints();
}

TEST_F(AddMultibodyPlantConstraintsTest, DistanceConstraint) {
  plant_.AddDistanceConstraint(*body_A_, Vector3d(0.0, 1.0, 0.0), *body_B_,
                               Vector3d(0.0, 1.0, 0.0), 1.5);
  CheckConstraints();
}

TEST_F(AddMultibodyPlantConstraintsTest, BallConstraint) {
  plant_.AddBallConstraint(*body_A_, Vector3d(0.0, 2.0, 0.0), *body_B_);
  CheckConstraints();
}

TEST_F(AddMultibodyPlantConstraintsTest, WeldConstraint) {
  const RigidBody<double>& body_C = plant_.AddRigidBody(
      "body_C", SpatialInertia<double>::SolidBoxWithMass(1, 1, 1, 1));
  plant_.AddWeldConstraint(*body_A_, RigidTransformd(Vector3d(1, 2, 3)), body_C,
                           RigidTransformd(Vector3d(4, 5, 6)));
  CheckConstraints(/* expected_num_constraints = */ 3, /* tol = */ 1e-2);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
