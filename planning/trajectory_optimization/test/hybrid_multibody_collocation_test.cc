#include "drake/planning/trajectory_optimization/hybrid_multibody_collocation.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::Solve;
using trajectories::PiecewisePolynomial;

const char ball_floor_xml[] = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="floor" type="box" size="5 5 5" pos="0 0 -5" />
    <body name="ball">
      <joint type="slide" name="z" axis="0 0 1" />
      <geom name="ball" type="sphere" size="1" mass="1" />
    </body>
  </worldbody>
</mujoco>
)""";

// A ball (sphere) resting at a fixed point on the floor (box).
//
// This problem has a single mode and a trivial solution; but it does require
// contact forces.
GTEST_TEST(HybridMultibodyCollocationTest, BallOnTheFloor) {
  RobotDiagramBuilder<double> builder;
  builder.parser().AddModelsFromString(ball_floor_xml, "xml");
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->plant().GetMyContextFromRoot(*context);

  const double kTimeStep = 0.1;
  HybridMultibodyCollocation hybrid(diagram.get(), *context, kTimeStep,
                                    kTimeStep);
  EXPECT_EQ(hybrid.num_inputs(), 0);
  EXPECT_EQ(hybrid.num_states(), 2);
  EXPECT_EQ(hybrid.model_inspector().num_geometries(), 4);

  auto& prog = hybrid.prog();

  EXPECT_EQ(hybrid.GetContactPairCandidates().size(), 1);
  const int kNumTimeSteps = 2;
  auto* mode = hybrid.AddMode("ground", kNumTimeSteps,
                              hybrid.GetContactPairCandidates());
  EXPECT_EQ(mode->input().size(), 0);
  EXPECT_EQ(mode->state().size(), 2);
  EXPECT_EQ(mode->AllContactForces().size(), 3);

  // zdot = 0.
  prog.AddBoundingBoxConstraint(0, 0, mode->initial_state()[1]);
  prog.AddBoundingBoxConstraint(0, 0, mode->final_state()[1]);

  // Constant force (otherwise there is a manifold of solutions, since only the
  // average force over an interval must balance gravity).
  prog.AddLinearEqualityConstraint(mode->AllContactForces(0) ==
                                   mode->AllContactForces(1));

  // Ipopt complains about TOO_FEW_DEGREES_OF_FREEDOM for this problem.
  solvers::SnoptSolver solver;
  if (!solver.available() || !solver.enabled()) {
    return;
  }
  auto result = solver.Solve(prog);
  EXPECT_TRUE(result.is_success());

  const double kTol = 1e-3;
  // Ball position is in contact.
  EXPECT_TRUE(CompareMatrices(result.GetSolution(mode->initial_state()),
                              Vector2d(1, 0), kTol));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(mode->final_state()),
                              Vector2d(1, 0), kTol));
  // Contact forces are balancing gravity.
  const Vector3d mg = diagram->plant().CalcTotalMass(plant_context) *
                      diagram->plant().gravity_field().gravity_vector();
  for (int i = 0; i < kNumTimeSteps; ++i) {
    EXPECT_TRUE(CompareMatrices(result.GetSolution(mode->AllContactForces(i)),
                                -mg, kTol));
  }

  // Add the same contact pairs again as a second mode.
  auto* mode2 = hybrid.AddMode("ground2", kNumTimeSteps,
                               hybrid.GetContactPairCandidates());
  prog.AddLinearEqualityConstraint(mode2->AllContactForces(0) ==
                                   mode2->AllContactForces(1));
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(mode2->initial_state()),
                              Vector2d(1, 0), kTol));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(mode2->final_state()),
                              Vector2d(1, 0), kTol));
  // Contact forces are balancing gravity.
  for (int i = 0; i < kNumTimeSteps; ++i) {
    EXPECT_TRUE(CompareMatrices(result.GetSolution(mode2->AllContactForces(i)),
                                -mg, kTol));
  }
}

// A ball (sphere) drops from above the floor (box) onto the floor.
GTEST_TEST(HybridMultibodyCollocationTest, BallDrop) {
  RobotDiagramBuilder<double> builder;
  builder.parser().AddModelsFromString(ball_floor_xml, "xml");
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->plant().GetMyContextFromRoot(*context);

  const double kMinTimeStep = 0.05;
  const double kMaxTimeStep = 0.5;
  HybridMultibodyCollocation hybrid(diagram.get(), *context, kMinTimeStep,
                                    kMaxTimeStep);
  auto& prog = hybrid.prog();

  EXPECT_EQ(hybrid.GetContactPairCandidates().size(), 1);
  const int kNumTimeSteps = 5;
  auto* aerial_phase = hybrid.AddMode("aerial", kNumTimeSteps, {});
  auto* ground_phase = hybrid.AddModeWithInelasticImpact(
      "ground", kNumTimeSteps, *hybrid.GetContactPairCandidates().begin());

  // z(t0) = 1.5, zdot(t0) = 0.
  prog.AddBoundingBoxConstraint(Vector2d(1.5, 0), Vector2d(1.5, 0),
                                aerial_phase->initial_state());

  auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  const double kTol = 1e-3;
  // Ball position is in contact.
  EXPECT_NEAR(result.GetSolution(aerial_phase->final_state()[0]), 1.0, kTol);
  EXPECT_TRUE(CompareMatrices(result.GetSolution(ground_phase->initial_state()),
                              Vector2d(1, 0), kTol));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(ground_phase->final_state()),
                              Vector2d(1, 0), kTol));
  // Contact forces are balancing gravity.
  const Vector3d mg = diagram->plant().CalcTotalMass(plant_context) *
                      diagram->plant().gravity_field().gravity_vector();
  for (int i = 0; i < kNumTimeSteps; ++i) {
    EXPECT_TRUE(CompareMatrices(
        result.GetSolution(ground_phase->AllContactForces(i)), -mg, kTol));
  }
}

// A ball (sphere) drops from above the floor (box) onto the floor, then rolls
// due to a torque around theta.  This confirms that actuators work.
GTEST_TEST(HybridMultibodyCollocationTest, BallDropAndRoll) {
  const std::string xml = R"""(
<mujoco model="test">
<worldbody>
    <geom name="floor" type="box" size="5 5 5" pos="0 0 -5" />
    <body name="body">
    <joint type="slide" name="x" axis="1 0 0" />
    <joint type="slide" name="z" axis="0 0 1" />
    <joint type="hinge" name="theta" axis="0 1 0" />
    <geom name="ball1" type="sphere" size="1" mass="1"/>
    </body>
</worldbody>
<actuator>
    <motor joint="theta"/>
</actuator>
</mujoco>)""";

  RobotDiagramBuilder<double> builder;
  builder.parser().AddModelsFromString(xml, "xml");
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const double kMinTimeStep = 0.05;
  const double kMaxTimeStep = 0.5;
  HybridMultibodyCollocation hybrid(diagram.get(), *context, kMinTimeStep,
                                    kMaxTimeStep);
  auto& prog = hybrid.prog();

  EXPECT_EQ(hybrid.GetContactPairCandidates().size(), 1);
  EXPECT_EQ(hybrid.num_inputs(), 1);
  const int kNumTimeSteps = 5;
  auto* aerial_phase = hybrid.AddMode("aerial", kNumTimeSteps, {});
  auto* ground_phase = hybrid.AddModeWithInelasticImpact(
      "ground", kNumTimeSteps, *hybrid.GetContactPairCandidates().begin());

  Vector6d x0 = Vector6d::Zero();
  x0[1] = 1.5;  // z(0) = 1.5
  prog.AddBoundingBoxConstraint(x0, x0, aerial_phase->initial_state());

  auto constant_torque = std::make_shared<solvers::BoundingBoxConstraint>(
      Vector1d(1), Vector1d(1));
  aerial_phase->AddConstraintToAllKnotPoints(constant_torque,
                                             aerial_phase->input());
  ground_phase->AddConstraintToAllKnotPoints(constant_torque,
                                             ground_phase->input());
  auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Ball is rolling at the final state.
  EXPECT_GE(result.GetSolution(ground_phase->final_state()[5]), 0.1);
  // Ball has rolled a non-trivial amount.
  EXPECT_GE(result.GetSolution(ground_phase->final_state()[2]), 0.1);

  // Test the reconstruction routines.
  auto aerial_u_traj = aerial_phase->ReconstructInputTrajectory(result);
  auto aerial_x_traj = aerial_phase->ReconstructStateTrajectory(result);
  auto aerial_f_traj = aerial_phase->ReconstructContactForceTrajectory(
      result, *hybrid.GetContactPairCandidates().begin());

  EXPECT_EQ(aerial_u_traj.rows(), 1);
  EXPECT_EQ(aerial_x_traj.rows(), 6);
  EXPECT_EQ(aerial_f_traj.rows(), 3);

  auto u_traj = hybrid.ReconstructInputTrajectory(result);
  auto x_traj = hybrid.ReconstructStateTrajectory(result);
  auto f_traj = hybrid.ReconstructContactForceTrajectory(
      result, *hybrid.GetContactPairCandidates().begin());

  EXPECT_EQ(u_traj.rows(), 1);
  EXPECT_EQ(x_traj.rows(), 6);
  EXPECT_EQ(f_traj.rows(), 3);

  // z velocity at touchdown in the aerial phase is < 0.0.
  EXPECT_LE(aerial_x_traj.value(aerial_x_traj.end_time())(4), 0);
  // z velocity at post-touchdown full trajectory is 0.0 (due to the impact).
  EXPECT_NEAR(x_traj.value(aerial_x_traj.end_time() + 0.01)(4), 0, 1e-3);
}

// A ball (sphere) drops onto an inclined plane, and rolls down. (This confirms
// that rolling is indeed possible under the sticking friction implementation).
GTEST_TEST(HybridMultibodyCollocationTest, DropAndRollDownRamp) {
  const std::string xml = R"""(
<mujoco model="test">
<worldbody>
    <geom name="floor" type="box" size="50 50 50" pos="-12.94 0 -48.29"
        euler="0 15 0"/>
    <body name="body">
    <joint type="slide" name="x" axis="1 0 0" />
    <joint type="slide" name="z" axis="0 0 1" />
    <joint type="hinge" name="theta" axis="0 1 0" />
    <geom name="ball1" type="sphere" size="1" mass="1"  pos="3 0 0"/>
    </body>
</worldbody>
</mujoco>)""";

  RobotDiagramBuilder<double> builder;
  builder.parser().AddModelsFromString(xml, "xml");
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const double kMinTimeStep = 0.05;
  const double kMaxTimeStep = 0.5;
  HybridMultibodyCollocation hybrid(diagram.get(), *context, kMinTimeStep,
                                    kMaxTimeStep);
  auto& prog = hybrid.prog();

  EXPECT_EQ(hybrid.GetContactPairCandidates().size(), 1);
  const int kNumTimeSteps = 5;
  auto* aerial_phase = hybrid.AddMode("aerial", kNumTimeSteps, {});
  auto* ground_phase = hybrid.AddModeWithInelasticImpact(
      "ground", kNumTimeSteps, *hybrid.GetContactPairCandidates().begin());

  Vector6d x0 = Vector6d::Zero();
  x0[1] = 1.5;  // z(0) = 1.5
  prog.AddBoundingBoxConstraint(x0, x0, aerial_phase->initial_state());

  auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Ball is rolling at the final state.
  EXPECT_GE(result.GetSolution(ground_phase->final_state()[5]), 0.1);
  // Ball has rolled a non-trivial amount.
  EXPECT_GE(result.GetSolution(ground_phase->final_state()[2]), 0.5);
}

// A ball (sphere) drops onto an inclined plane, and rolls down, this time in
// 3D.  This confirms that the code works when num_positions !=
// num_velocities().
GTEST_TEST(HybridMultibodyCollocationTest, DropAndRoll3D) {
  const std::string xml = R"""(
<mujoco model="test">
<worldbody>
    <geom name="floor" type="box" size="50 50 50" pos="-12.94 0 -48.29"
        euler="0 15 0"/>
    <body name="body">
    <freejoint name="ball" />
    <geom name="ball1" type="sphere" size="1" mass="1"  pos="3 0 0"/>
    </body>
</worldbody>
</mujoco>)""";

  RobotDiagramBuilder<double> builder;
  builder.parser().AddModelsFromString(xml, "xml");
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  const double kMinTimeStep = 0.05;
  const double kMaxTimeStep = 0.5;
  HybridMultibodyCollocation hybrid(diagram.get(), *context, kMinTimeStep,
                                    kMaxTimeStep);
  auto& prog = hybrid.prog();

  EXPECT_EQ(hybrid.GetContactPairCandidates().size(), 1);
  const int kNumTimeSteps = 5;
  auto* aerial_phase = hybrid.AddMode("aerial", kNumTimeSteps, {});
  aerial_phase->AddEqualTimeIntervalsConstraints();
  auto* ground_phase = hybrid.AddModeWithInelasticImpact(
      "ground", kNumTimeSteps, *hybrid.GetContactPairCandidates().begin());
  ground_phase->AddEqualTimeIntervalsConstraints();

  VectorXd x0 = VectorXd::Zero(13);
  x0[0] = 1;    // qw(0) = 1
  x0[6] = 1.5;  // z(0) = 1.5
  prog.AddBoundingBoxConstraint(x0, x0, aerial_phase->initial_state());

  // Set (trivial) initial guess to avoid ever evaluating the quaternion at all
  // zeros.
  Vector2d breaks(0, 1);
  MatrixXd samples(13, 2);
  samples << x0, x0;
  aerial_phase->SetInitialTrajectory(
      PiecewisePolynomial<double>(),
      PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples));
  ground_phase->SetInitialTrajectory(
      PiecewisePolynomial<double>(),
      PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples));
  auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Ball is rolling (about y) at the final state.
  EXPECT_GE(result.GetSolution(ground_phase->final_state()[8]), 0.1);
  // Ball has rolled a non-trivial amount in x.
  EXPECT_GE(result.GetSolution(ground_phase->final_state()[4]), 0.1);
}

// A body on a planar joint with two rigidly attached contact spheres resting
// in static friction at a fixed point on a ramp (box).
//
// This problem has a single mode and a trivial solution; but it does require
// contact forces.
GTEST_TEST(HybridMultibodyCollocationTest, InclinedPlaneSticking) {
  const std::string inclined_plane_15deg_xml = R"""(
  <mujoco model="test">
    <worldbody>
      <geom name="floor" type="box" size="50 50 50" pos="-12.94 0 -48.29"
            euler="0 15 0"/>
      <body name="body">
        <joint type="slide" name="x" axis="1 0 0" />
        <joint type="slide" name="z" axis="0 0 1" />
        <joint type="hinge" name="theta" axis="0 1 0" />
        <geom name="ball1" type="sphere" size="1" mass="1"  pos="3 0 0"/>
        <geom name="ball2" type="sphere" size="1" mass="1"  pos="-3 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )""";
  RobotDiagramBuilder<double> builder;
  builder.parser().AddModelsFromString(inclined_plane_15deg_xml, "xml");
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->plant().GetMyContextFromRoot(*context);

  const double kTimeStep = 0.1;
  HybridMultibodyCollocation hybrid(diagram.get(), *context, kTimeStep,
                                    kTimeStep);
  auto& prog = hybrid.prog();

  EXPECT_EQ(hybrid.GetContactPairCandidates().size(), 2);
  const int kNumTimeSteps = 2;
  auto* mode =
      hybrid.AddMode("ramp", kNumTimeSteps, hybrid.GetContactPairCandidates());
  EXPECT_EQ(mode->input().size(), 0);
  EXPECT_EQ(mode->state().size(), 6);

  // velocity = 0.
  prog.AddBoundingBoxConstraint(0, 0, mode->initial_state().tail<3>());
  prog.AddBoundingBoxConstraint(0, 0, mode->final_state().tail<3>());

  // Constant force (otherwise there is a manifold of solutions, since only the
  // average force over an interval must balance gravity).
  prog.AddLinearEqualityConstraint(mode->AllContactForces(0) ==
                                   mode->AllContactForces(1));

  // Ipopt complains about TOO_FEW_DEGREES_OF_FREEDOM for this problem.
  solvers::SnoptSolver solver;
  if (!solver.available() || !solver.enabled()) {
    return;
  }
  auto result = solver.Solve(prog);
  EXPECT_TRUE(result.is_success());

  const double kTol = 1e-3;
  // Contact forces are balancing gravity.
  const Vector3d mg = diagram->plant().CalcTotalMass(plant_context) *
                      diagram->plant().gravity_field().gravity_vector();
  for (int i = 0; i < kNumTimeSteps; ++i) {
    Vector3d total_force =
        result.GetSolution(mode->AllContactForces(i).head<3>()) +
        result.GetSolution(mode->AllContactForces(i).tail<3>());
    EXPECT_TRUE(CompareMatrices(total_force, -mg, kTol));
  }
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
