
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace {

using examples::manipulation_station::ManipulationStation;
using robot_plans::PlanData;
using robot_plans::PlanType;
using std::cout;
using std::endl;
using std::vector;

int test_joint_space_plan() {
  // create plan
  PlanData plan1;

  Eigen::VectorXd t_knots(3);
  t_knots << 0, 2, 4;

  Eigen::MatrixXd q_knots(7, 3);
  q_knots.col(0) << 0, 0, 0, -1.75, 0, 1.0, 0;
  q_knots.col(1) << 0.5, 0, 0, -1.75, 0, 1.0, 0;
  q_knots.col(2) << 1, 0, 0, -1.75, 0, 1.0, 0;

  auto qtraj = trajectories::PiecewisePolynomial<double>::Cubic(
      t_knots, q_knots, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

  plan1.plan_type = PlanType::kJointSpacePlan;
  plan1.joint_traj = qtraj;
  std::vector<PlanData> plan_list{plan1};

  systems::DiagramBuilder<double> builder;

  // Set up ManipulationStation.
  const double time_step = 0.002;
  auto station =
      builder.template AddSystem<ManipulationStation<double>>(time_step);
  station->SetupDefaultStation();
  station->Finalize();

  // Set up PlanRunner.
  auto plan_runner =
      builder.template AddSystem<RobotPlanRunner>(true, time_step);

  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  plan_runner->GetInputPort("iiwa_position_measured"));
  builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                  plan_runner->GetInputPort("iiwa_velocity_estimated"));
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  plan_runner->GetInputPort("iiwa_torque_external"));
  builder.Connect(plan_runner->GetOutputPort("iiwa_position_command"),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(plan_runner->GetOutputPort("iiwa_torque_command"),
                  station->GetInputPort("iiwa_feedforward_torque"));

  // Set up PlanSender.
  auto plan_sender = builder.template AddSystem<PlanSender>(plan_list);
  builder.Connect(plan_sender->GetOutputPort("plan_data"),
                  plan_runner->GetInputPort("plan_data"));
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  plan_sender->GetInputPort("q"));

  // Add a visualizer.
  geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& station_context =
      diagram->GetMutableSubsystemContext(*station, &context);

  station->GetInputPort("wsg_force_limit").FixValue(&station_context, 40.0);
  station->GetInputPort("wsg_position").FixValue(&station_context, 0.05);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(plan_sender->get_all_plans_duration());

  return 0;
};

}  // namespace
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

int main() {
  return drake::manipulation::robot_plan_runner::test_joint_space_plan();
};