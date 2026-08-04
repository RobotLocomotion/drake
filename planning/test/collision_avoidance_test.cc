#include "drake/planning/collision_avoidance.h"

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/unimplemented_collision_checker.h"

namespace drake {
namespace planning {
namespace internal {
namespace {

using drake::multibody::BodyIndex;
using drake::multibody::default_model_instance;
using Eigen::VectorXd;
using std::unique_ptr;

double ZeroDistanceFunc(const VectorXd&, const VectorXd&) {
  return 0.0;
}

/* We simply need a diagram with a non-zero number of dofs; a single floating
 body will give us 7 -- the semantics are unimportant. */
unique_ptr<RobotDiagram<double>> MakeRobot() {
  RobotDiagramBuilder<double> builder;
  const auto model_instance = drake::multibody::default_model_instance();
  builder.plant().AddRigidBody("floater", model_instance);
  return builder.Build();
}

/* Has to match the 7 dofs in the robot. */
constexpr int kQSize = 7;

/* A minimum implementation of the CollisionChecker, but one that allows us to
 explicitly specify the result of CalcRobotClearance(). */
class DummyCollisionChecker final : public UnimplementedCollisionChecker {
 public:
  /* I'm passing garbage parameters to UnimplementedCollisionChecker. That's ok;
   the only API we're exercising is CalcRobotClearance() directly and
   DoCalcContextRobotClearance() and DoUpdateContextPositions() indirectly.
   We've overridden the latter so they don't care about the garbage. */
  DummyCollisionChecker()
      : UnimplementedCollisionChecker(
            {.model = MakeRobot(),
             .robot_model_instances = {default_model_instance()},
             .configuration_distance_function = ZeroDistanceFunc,
             .edge_step_size = 0.1},
            false /* supports_parallel */),
        data_(plant().num_positions()) {
    AllocateContexts();
  }

  /* This clearance data will define the distance/jacobian data that will be
   given back to ComputeCollisionAvoidanceDisplacement() when invoking
   CalcRobotClearance(). */
  void set_robot_clearance(RobotClearance data) { data_ = std::move(data); }

 private:
  void DoUpdateContextPositions(
      CollisionCheckerContext* model_context) const final {
    /* Invoked indirectly; but we won't do any work. */
    return;
  }

  RobotClearance DoCalcContextRobotClearance(
      const CollisionCheckerContext& model_context,
      double influence_distance) const final {
    return data_;
  }

  RobotClearance data_;
};

/* ComputeCollisionAvoidanceDisplacement() is uniquely responsible for handling:
    - Calling the right CollisionChecker API based on presence of a context
      (i.e., context provenance).
    - Empty distance measurements produces zero vector.
    - Weights are properly computed.

Each of those gets us own unit test case in the below. */

GTEST_TEST(ComputeCollisionAvoidanceDisplacementTest, ContextProvenance) {
  DummyCollisionChecker checker;
  const VectorXd q1 = (VectorXd(kQSize) << 1, 2, 3, 4, 5, 6, 7).finished();
  const VectorXd q2 = (VectorXd(kQSize) << 9, 8, 7, 6, 5, 4, 3).finished();

  /* Confirm context provenance. We'll show that the expected context has been
   written to as evidence of provenance. */

  /* Implicit context. */
  EXPECT_FALSE(CompareMatrices(
      checker.plant().GetPositions(checker.plant_context()), q1));
  EXPECT_NO_THROW(ComputeCollisionAvoidanceDisplacement(checker, q1, 0, 1));
  EXPECT_TRUE(CompareMatrices(
      checker.plant().GetPositions(checker.plant_context()), q1));

  /* Implicit context using context number. */
  const int context_number = 0;
  EXPECT_FALSE(CompareMatrices(
      checker.plant().GetPositions(checker.plant_context(context_number)), q2));
  EXPECT_NO_THROW(
      ComputeCollisionAvoidanceDisplacement(checker, q2, 0, 1, context_number));
  EXPECT_TRUE(CompareMatrices(
      checker.plant().GetPositions(checker.plant_context(context_number)), q2));

  /* Explicit context. */
  auto context = checker.MakeStandaloneModelContext();
  EXPECT_FALSE(CompareMatrices(
      checker.plant().GetPositions(context->plant_context()), q2));
  EXPECT_NO_THROW(ComputeCollisionAvoidanceDisplacement(
      checker, q2, 0, 1, std::nullopt, context.get()));
  EXPECT_TRUE(CompareMatrices(
      checker.plant().GetPositions(context->plant_context()), q2));

  /* Combining implicit context number and explicit context throws. */
  EXPECT_THROW(ComputeCollisionAvoidanceDisplacement(
                   checker, q2, 0, 1, context_number, context.get()),
               std::exception);
}

GTEST_TEST(ComputeCollisionAvoidanceDisplacementTest, NoDistanceMeasurements) {
  DummyCollisionChecker checker;
  const VectorXd q = (VectorXd(kQSize) << 1, 2, 3, 4, 5, 6, 7).finished();

  /* The checker returns an empty RobotClearance by default. This should lead to
   zero-valued Δq. We're passing in an arbitrary, non-zero, q to eliminate the
   appearance that the return value copies the input value. */
  EXPECT_TRUE(
      CompareMatrices(ComputeCollisionAvoidanceDisplacement(checker, q, 0, 1),
                      VectorXd::Zero(kQSize)));
}

GTEST_TEST(ComputeCollisionAvoidanceDisplacementTest, WeightedCombinations) {
  DummyCollisionChecker checker;
  const VectorXd q0 = VectorXd::Zero(kQSize);
  const VectorXd q1 = (VectorXd(kQSize) << 1, 2, 3, 4, 5, 6, 7).finished();
  const VectorXd q2 = (VectorXd(kQSize) << 9, 8, 7, 6, 5, 4, 3).finished();

  /* Confirm weight calculation. Using an arbitrary safety range, we'll specify
   distances around the range and confirm the Jacobians have been properly
   scaled. */
  const double kMin = -0.25;
  const double kMax = 0.75;
  const double kRange = kMax - kMin;
  const double kTol = 1e-15;

  /* We'll use a robot-env collision; ComputeCollisionAvoidanceDisplacement()
   ignores this field. So, robot-env and robot-robot measurements should be
   treated identically. */
  const auto& plant = checker.plant();
  const BodyIndex robot_body = plant.GetBodyByName("floater").index();
  const BodyIndex env_body = plant.GetBodyByName("world").index();
  const auto env_type = RobotCollisionType::kEnvironmentCollision;

  /* In the following tests, the q passed to
   ComputeCollisionAvoidanceDisplacement() doesn't matter. Only the qs in
   the collision avoidance data matter. */

  {
    /* Deeper than kMin (max_penetration), saturates to a weight of 1. */
    RobotClearance clearance(kQSize);
    clearance.Append(robot_body, env_body, env_type, kMin - 0.1,
                     q1.transpose());
    checker.set_robot_clearance(std::move(clearance));
    const VectorXd grad =
        ComputeCollisionAvoidanceDisplacement(checker, q0, kMin, kMax);
    EXPECT_TRUE(CompareMatrices(grad, q1));
  }

  {
    /* Farther than kMax (max_penetration), saturates to a weight of 0. */
    RobotClearance clearance(kQSize);
    clearance.Append(robot_body, env_body, env_type, kMax + 0.1,
                     q1.transpose());
    checker.set_robot_clearance(std::move(clearance));
    const VectorXd grad =
        ComputeCollisionAvoidanceDisplacement(checker, q0, kMin, kMax);
    EXPECT_TRUE(CompareMatrices(grad, q0));
  }

  {
    /* Pick an arbitrary weight between 0 and 1. */
    constexpr double kWeight = 1.0 / 3.0;
    RobotClearance clearance(kQSize);
    clearance.Append(robot_body, env_body, env_type, kMax - kRange * kWeight,
                     q1.transpose());
    checker.set_robot_clearance(std::move(clearance));
    const VectorXd grad =
        ComputeCollisionAvoidanceDisplacement(checker, q0, kMin, kMax);
    EXPECT_EQ(grad.size(), kQSize);
    EXPECT_TRUE(CompareMatrices(grad, kWeight * q1, kTol));
  }

  {
    /* Two arbitrary weights combine. */
    constexpr double kWeight1 = 1.0 / 3.0;
    constexpr double kWeight2 = 3.0 / 5.0;
    RobotClearance clearance(kQSize);
    clearance.Append(robot_body, env_body, env_type, kMax - kRange * kWeight1,
                     q1.transpose());
    clearance.Append(robot_body, env_body, env_type, kMax - kRange * kWeight2,
                     q2.transpose());
    checker.set_robot_clearance(std::move(clearance));
    const VectorXd grad =
        ComputeCollisionAvoidanceDisplacement(checker, q0, kMin, kMax);
    EXPECT_EQ(grad.size(), kQSize);
    EXPECT_TRUE(CompareMatrices(grad, kWeight1 * q1 + kWeight2 * q2, kTol));
  }
}

GTEST_TEST(ComputeCollisionAvoidanceDisplacementTest, Errors) {
  DummyCollisionChecker checker;
  const VectorXd q = VectorXd::Zero(kQSize);

  // The max_penetration cannot be positive.
  EXPECT_THROW(ComputeCollisionAvoidanceDisplacement(checker, q, 0.1, 1),
               std::exception);

  // The max_clearance cannot be negative.
  EXPECT_THROW(ComputeCollisionAvoidanceDisplacement(checker, q, -1, -0.1),
               std::exception);

  // They cannot be both zero.
  EXPECT_THROW(ComputeCollisionAvoidanceDisplacement(checker, q, 0, 0),
               std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace planning
}  // namespace drake
