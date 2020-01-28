#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/manipulation/robot_bridge/robot_bridge.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {
namespace {

using Eigen::VectorXd;
using examples::manipulation_station::ManipulationStation;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 1e33, "Simulation duration.");

template <typename T>
multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const multibody::Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, multibody::MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  multibody::Parser parser(plant);
  const multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();
  math::RigidTransform<double> X_WC(math::RollPitchYaw<double>(M_PI, 0, 0),
                                    Eigen::Vector3d(0.5, -0.1, 1));
  station->SetupClutterClearingStation(X_WC);
  station->AddManipulandFromFile(
      "drake/examples/manipulation_station/models/061_foam_brick.sdf",
      RigidTransform<double>(RollPitchYaw<double>(-1.57, 0, 3),
                             Eigen::Vector3d(-0.3, -0.55, 0.36)));
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_mutable_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));
  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, station->get_mutable_multibody_plant(),
      station->GetOutputPort("contact_results"));

  // Create a robot bridge.
  multibody::MultibodyPlant<double> rb_plant(0.0);
  std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf");
  AddAndWeldModelFrom(iiwa_path, "iiwa", rb_plant.world_frame(), "iiwa_link_0",
                      RigidTransform<double>::Identity(), &rb_plant);
  rb_plant.Finalize();
  const auto& tool_frame = rb_plant.GetFrameByName("iiwa_link_7");
  auto robot_comm = builder.AddSystem<manipulation::robot_bridge::RobotBridge>(
      &rb_plant, &tool_frame, 1e-3);

  builder.Connect(station->GetOutputPort("iiwa_state_estimated"),
                  robot_comm->GetInputPort("state"));
  builder.Connect(robot_comm->GetOutputPort("position"),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(robot_comm->GetOutputPort("torque"),
                  station->GetInputPort("iiwa_feedforward_torque"));

  // Setup simulation.
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto& station_context = diagram->GetMutableSubsystemContext(
      *station, &simulator.get_mutable_context());
  auto& rb_context = diagram->GetMutableSubsystemContext(
      *robot_comm, &simulator.get_mutable_context());

  // Nominal WSG position is open.
  station->GetInputPort("wsg_position").FixValue(&station_context, 0.1);
  // Force limit at 40N.
  station->GetInputPort("wsg_force_limit").FixValue(&station_context, 40.0);

  // Save initial configuration and tool pose.
  double t0 = simulator.get_context().get_time();
  Eigen::VectorXd q0 = station->GetIiwaPosition(station_context);
  math::RigidTransform<double> X_WT0;
  {
    auto temp_context = rb_plant.CreateDefaultContext();
    rb_plant.SetPositions(temp_context.get(), q0);
    X_WT0 = rb_plant.CalcRelativeTransform(*temp_context,
                                           rb_plant.world_frame(), tool_frame);
  }

  // I am manually controlling the simulator stepping to mimic the anzu
  // workflow for programming behaviors such as:
  //    robot.MoveQ(q0);
  //    robot.MoveTool(X1);
  //    ...
  // Initialize command sources.
  Eigen::VectorXd q1 = q0;
  double t1 = t0 + 1;
  q1[1] -= M_PI_2;
  const trajectories::PiecewisePolynomial<double> q_traj0 =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold({t0, t1},
                                                                {q0, q1});
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj0));

  // The time for the tool traj is kind of screwy.. I set it this way to avoid
  // move tool and move j being both active (which is determined by checking
  // context time against traj time.)
  const manipulation::SingleSegmentCartesianTrajectory<double> tool_traj0(
      X_WT0.GetAsIsometry3(), X_WT0.GetAsIsometry3(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 0, 0, t0 - 1, t0 - 0.9);
  robot_comm->GetInputPort("tool_trajectory")
      .FixValue(&rb_context,
                Value<manipulation::SingleSegmentCartesianTrajectory<double>>(
                    tool_traj0));

  // Initialize controller. Important, this needs to happen after
  // diagram->SetRandomContext, otherwise it will reset robot_comm's internal
  // state to the default model values (which are totally useless), and the x0
  // values that we just initialized will be wipedout.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(rb_plant.num_multibody_states());
  x0.head(rb_plant.num_positions()) = q0;
  robot_comm->Initialize(x0, &rb_context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  // Exec first move j.
  simulator.AdvanceTo(t1);

  // Change command for the second move j.
  t0 = t1;
  t1 = t0 + 1;
  const trajectories::PiecewisePolynomial<double> q_traj1 =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold({t0, t1},
                                                                {q1, q0});
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj1));
  // Exec second move j.
  simulator.AdvanceTo(t1);

  // Change command for move tool.
  t0 = t1;
  t1 = t0 + 1;
  Eigen::Isometry3d X_WT_target =
      Eigen::Translation3d(0, 0, 0.05) * X_WT0.GetAsIsometry3() *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());
  math::RigidTransform<double> X_WT1(X_WT_target);
  const manipulation::SingleSegmentCartesianTrajectory<double> tool_traj1(
      X_WT0.GetAsIsometry3(), X_WT1.GetAsIsometry3(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 0, 0, t0, t1);
  robot_comm->GetInputPort("tool_trajectory")
      .FixValue(&rb_context,
                Value<manipulation::SingleSegmentCartesianTrajectory<double>>(
                    tool_traj1));
  // Exec last move t.
  simulator.AdvanceTo(t1 + 1);

  return 0;
}

}  // namespace
}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::manipulation::robot_bridge::do_main(argc, argv);
}
