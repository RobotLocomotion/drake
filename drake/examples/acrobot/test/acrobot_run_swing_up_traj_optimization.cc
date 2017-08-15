// Generates a swing-up trajectory for acrobot and displays the trajectory
// in DrakeVisualizer. Trajectory generation code is based on
// pendulum_swing_up.cc.

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/acrobot_swing_up.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::SolutionResult;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

namespace drake {
namespace examples {
namespace acrobot {
namespace {

GTEST_TEST(AcrobotTest, SwingUpTrajectoryOptimization) {
  systems::DiagramBuilder<double> builder;

  AcrobotPlant<double> acrobot;

  const int kNumTimeSamples = 21;
  const double kTrajectoryTimeLowerBound = 2;
  const double kTrajectoryTimeUpperBound = 10;

  const Eigen::Vector4d x0(0, 0, 0, 0);
  const Eigen::Vector4d xG(M_PI, 0, 0, 0);

  auto context = acrobot.CreateDefaultContext();

  systems::DircolTrajectoryOptimization dircol_traj(
      &acrobot, *context, kNumTimeSamples, kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  AddSwingUpTrajectoryParams(x0, xG, &dircol_traj);

  const double timespan_init = 4;
  auto traj_init_x =
      PiecewisePolynomialType::FirstOrderHold({0, timespan_init}, {x0, xG});
  SolutionResult result = dircol_traj.SolveTraj(
      timespan_init, PiecewisePolynomialType(), traj_init_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);

  const PiecewisePolynomialTrajectory pp_xtraj =
      dircol_traj.ReconstructStateTrajectory();
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);

  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.StepTo(kTrajectoryTimeUpperBound);
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
