/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char kUrdfPath[] =
    "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf";

unique_ptr<PiecewisePolynomialTrajectory> MakePlan() {
  auto tree = make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + kUrdfPath, multibody::joints::kFixed, tree.get());

  // Creates a basic pointwise IK trajectory for moving the iiwa arm.
  // It starts in the zero configuration (straight up).
  VectorXd zero_conf = tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Defines an end effector constraint and makes it active for the time span
  // from 1 to 3 seconds.
  Vector3d pos_end(0.6, 0, 0.325);
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(1, 3));

  // After the end effector constraint is released, applies the straight
  // up configuration again from time 4 to 5.9.
  PostureConstraint pc2(tree.get(), Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Apply the same end effector constraint from time 6 to 9 of the demo.
  WorldPositionConstraint wpc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the time wpc2 is active, constrains the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) =
      tree->FindChildBodyOfJoint("iiwa_joint_2")->get_position_start_index();
  PostureConstraint pc3(tree.get(), Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

  const std::vector<double> kTimes{0.0, 2.0, 5.0, 7.0, 9.0};
  MatrixXd q0(tree->get_num_positions(), kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    q0.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc1);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);
  IKoptions ikoptions(tree.get());
  std::vector<int> info(kTimes.size(), 0);
  MatrixXd q_sol(tree->get_num_positions(), kTimes.size());
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(tree.get(), kTimes.size(), kTimes.data(), q0, q0,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);
  bool info_good = true;
  for (size_t i = 0; i < kTimes.size(); ++i) {
    drake::log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a valid solution.");
  }

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

  auto tree = std::make_unique<RigidBodyTree<double>>();
  CreateTreedFromFixedModelAtPose(kUrdfPath, tree.get());

  std::unique_ptr<PiecewisePolynomialTrajectory> traj = MakePlan();

  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  // Adds a plant
  builder.AddPlant(std::move(tree));
  builder.AddVisualizer(&lcm);

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller =
      builder.AddController<systems::InverseDynamicsController<double>>(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
          GetDrakePath() + kUrdfPath, nullptr, iiwa_kp, iiwa_ki, iiwa_kd,
          false /* no feedforward acceleration */);

  // Adds a trajectory source for desired state.
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();
  auto traj_src =
      diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
          *traj, 1 /* outputs q + v */);

  diagram_builder->Connect(traj_src->get_output_port(),
                  controller->get_input_port_desired_state());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
