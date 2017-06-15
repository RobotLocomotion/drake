/// @file
///
/// This demo sets up a position controlled and gravity compensated Kinova Jaco
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration through a motion
/// that reaches a Cartesian position in space, and then repeats this reaching
/// task while applying a joint configuration constraint. Note that the
/// end-effector orientation is not constrained in this demo.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;

DEFINE_double(simulation_sec, 12, "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

const char kRelUrdfPath[] =
    "/manipulation/models/jaco_description/urdf/j2n6s300.urdf";

std::unique_ptr<PiecewisePolynomialTrajectory> MakePlan() {
  const std::string kUrdfPath =
      drake::GetDrakePath() + std::string(kRelUrdfPath);
  auto tree = make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kUrdfPath, multibody::joints::kFixed, tree.get());

  // Create a basic point-wise IK trajectory for moving the Jaco arm.
  // It starts in the zero configuration (straight up).
  VectorXd zero_conf = tree->getZeroConfiguration();

  // Define upper and lower bound limits for the starting configuration
  VectorXd joint_lb = zero_conf - VectorXd::Constant(kNumDofs, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(kNumDofs, 0.01);

  // Defines the first posture constraint and makes it active for the
  // time span of 0 to 0.5 seconds
  Vector2d pc1_tspan = Vector2d(0, 0.5);
  PostureConstraint pc1(tree.get(), pc1_tspan);
  VectorXi joint_idx(kNumDofs);
  joint_idx << 0, 1, 2, 3, 4, 5, 6, 7, 8;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Defines an end effector constraint and makes it active for the time span
  // from 1 to 3 seconds.
  Vector2d wpc1_tspan = Vector2d(1, 3);
  Vector3d pos_end(-0.4, -0.4, 0.6);  // end goal in world coordinates (x,y,z)
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);  // lower bound
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);  // upper bound
  WorldPositionConstraint wpc1(tree.get(),
                               tree->FindBodyIndex("j2n6s300_end_effector"),
                               Vector3d::Zero(), pos_lb, pos_ub, wpc1_tspan);

  // After the end effector constraint is released, applies the straight
  // up configuration again from time 4 to 5.9.
  Vector2d pc2_tspan = Vector2d(4, 5.9);
  PostureConstraint pc2(tree.get(), pc2_tspan);
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Apply the same end effector constraint from time 6 to 9 of the demo.
  Vector2d wpc2_tspan = Vector2d(6, 9);
  WorldPositionConstraint wpc2(tree.get(),
                               tree->FindBodyIndex("j2n6s300_end_effector"),
                               Vector3d::Zero(), pos_lb, pos_ub, wpc2_tspan);

  // Constrains the first joint for the time wpc2 is active while preserving
  // the end effector constraint. The variable `joint_position_start_idx`
  // below is a collection of offsets into the state vector referring to the
  // positions of the joints to be constrained.
  Vector1<int> joint_position_start_idx;
  joint_position_start_idx(0) =
      tree->FindChildBodyOfJoint("j2n6s300_joint_1")
      ->get_position_start_index();
  Vector2d pc3_tspan = Vector2d(6, 12);
  PostureConstraint pc3(tree.get(), pc3_tspan);
  pc3.setJointLimits(joint_position_start_idx, Vector1d(-0.5), Vector1d(0.5));

  // Creates the constraint array vector
  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc1);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);

  // Defines the solution times
  const std::vector<double> kTimes{0.0, 2.0, 5.0, 8.0, 12.0};

  // Defines the seed (and nominal) solutions for IK
  MatrixXd q0(tree->get_num_positions(), kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    q0.col(i) = zero_conf;
  }

  // Defines other IK inputs
  IKoptions ikoptions(tree.get());
  std::vector<int> info(kTimes.size(), 0);
  MatrixXd q_sol(tree->get_num_positions(), kTimes.size());
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(tree.get(), kTimes.size(), kTimes.data(), q0, q0,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);

  // Checks the IK result for failures
  bool info_good = true;
  for (size_t i = 0; i < kTimes.size(); ++i) {
    drake::log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a valid solution.");
  }

  // Extracts the solution knot points
  std::vector<MatrixXd> knots(kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    // We only use column 0 of the matrix in knots (for joint positions),
    // so we write a vector.
    knots[i] = q_sol.col(i);
  }

  return make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots));
}

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  const std::string kUrdfPath =
      drake::GetDrakePath() + std::string(kRelUrdfPath);

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  systems::RigidBodyPlant<double>* plant = nullptr;
  std::unique_ptr<PiecewisePolynomialTrajectory> trajectory = MakePlan();

  {
    auto tree = make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFixedModelAtPose(kUrdfPath, tree.get());

    auto tree_sys =
        std::make_unique<systems::RigidBodyPlant<double>>(std::move(tree));
    plant =
        builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_sys));
    plant->set_name("plant");
  }

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);

  // Adds a controller
  VectorX<double> jaco_kp, jaco_kd, jaco_ki;
  SetPositionControlledJacoGains(&jaco_kp, &jaco_ki, &jaco_kd);
  auto control_sys = make_unique<systems::InverseDynamicsController<double>>(
      kUrdfPath, nullptr, jaco_kp, jaco_ki, jaco_kd,
      false /* no feedforward acceleration */);
  auto controller =
      builder.AddSystem<systems::InverseDynamicsController<double>>(
          std::move(control_sys));

  // Adds a trajectory source for desired state.
  auto traj_src = builder.AddSystem<systems::TrajectorySource<double>>(
      *trajectory, 1 /* outputs q + v */);
  traj_src->set_name("trajectory_source");

  builder.Connect(traj_src->get_output_port(),
                  controller->get_input_port_desired_state());

  // Connects the state port to the controller.
  const int kInstanceId =
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;
  const auto& state_out_port =
      plant->model_instance_state_output_port(kInstanceId);
  builder.Connect(state_out_port, controller->get_input_port_estimated_state());

  // Connects the controller torque output to plant.
  const auto& torque_input_port =
      plant->model_instance_actuator_command_input_port(kInstanceId);
  builder.Connect(controller->get_output_port_control(), torque_input_port);

  // Connects the visualizer and builds the diagram.
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.Initialize();
  simulator.set_target_realtime_rate(1);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kinova_jaco_arm::DoMain();
}
