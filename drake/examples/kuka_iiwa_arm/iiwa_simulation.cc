#include "iiwa_simulation.h"

#include "drake/common/drake_path.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::RigidBodySystem;
using lcm::LCM;

std::shared_ptr<RigidBodySystem> CreateKukaIiwaSystem(
  const std::string &file_name) {
  // Instantiates a rigid body system and adds the robot arm to it.
  auto rigid_body_system = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  rigid_body_system->AddModelInstanceFromFile(drake::GetDrakePath() + file_name,
                                              DrakeJoint::FIXED);

  // Sets some simulation parameters.
  rigid_body_system->penetration_stiffness = 3000.0;
  rigid_body_system->penetration_damping = 0;

  // Adds the ground.
  double kBoxWidth = 3;
  double kBoxDepth = 0.2;
  DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -kBoxDepth / 2.0;  // top of the box is at z = 0

  const std::shared_ptr<RigidBodyTree>& tree =
      rigid_body_system->getRigidBodyTree();

  RigidBody& world = tree->world();
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758,
      1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree->addCollisionElement(
      RigidBodyCollisionElement(geom, T_element_to_link, &world), world,
      "terrain");
  tree->updateStaticCollisionElements();

  return rigid_body_system;
}

std::shared_ptr<BotVisualizer<RigidBodySystem::StateVector>>
CreateKukaIiwaVisualizer(
    const std::shared_ptr<drake::RigidBodySystem> iiwa_system,
    const std::shared_ptr<lcm::LCM> lcm) {
  // Extracts the tree.
  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  // Instantiates a BotVisualizer for @p iiwa_system
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);
  return visualizer;
}

DRAKEKUKAIIWAARM_EXPORT
Eigen::VectorXd GenerateArbitraryIiwaInitialState() {
  const int kStateDimension = 14;  // Fixed for the IIWA Arm.
  const int kNumDof = 7;           // Fixed for the IIWA Arm.
  Eigen::VectorXd arbitrary_initial_state =
      Eigen::VectorXd::Zero(kStateDimension, 1);
  arbitrary_initial_state.head(kNumDof) << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01,
      0.01;
  return (arbitrary_initial_state);
}

drake::SimulationOptions SetupSimulation(double initial_step_size,
                                         double real_time_factor) {
  // Specifies the simulation options.
  drake::SimulationOptions options;
  options.realtime_factor = real_time_factor;
  options.initial_step_size = initial_step_size;

  // Prevents exception from being thrown when simulation runs slower than real
  // time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;
  return (options);
}

void CheckLimitViolations(
    const std::shared_ptr<drake::RigidBodySystem> rigid_body_sys,
    const Eigen::VectorXd& final_robot_state) {
  const auto& tree = rigid_body_sys->getRigidBodyTree();
  int num_positions = rigid_body_sys->number_of_positions();
  int num_velocities = rigid_body_sys->number_of_velocities();

  // Ensures the size of the output is correct.
  if (final_robot_state.size() !=
      static_cast<int>(rigid_body_sys->getNumOutputs())) {
    throw std::runtime_error(
        "ERROR: Size of final robot state (" +
        std::to_string(final_robot_state.size()) +
        ") does not match size of rigid body system's output (" +
        std::to_string(rigid_body_sys->getNumOutputs()) + ").");
  }

  // Ensures the number of position states equals the number of velocity states.
  if (num_positions != num_velocities) {
    throw std::runtime_error("ERROR: Number of positions (" +
                             std::to_string(num_positions) +
                             ") does not match the number of velocities (" +
                             std::to_string(num_velocities) + ").");
  }

  // Ensures the number of position and velocity states match the size of the
  // final robot state.
  if ((num_positions + num_velocities) != final_robot_state.size()) {
    throw std::runtime_error(
        "ERROR: Total number of positions and velocities (" +
        std::to_string(num_positions + num_velocities) +
        ") does not match size of robot state (" +
        std::to_string(final_robot_state.size()) + ").");
  }

  // Ensures the robot's joints are within their position limits.
  std::vector<std::unique_ptr<RigidBody>>& bodies = tree->bodies;
  for (int robot_state_index = 0, body_index = 0;
       body_index < static_cast<int>(bodies.size()); ++body_index) {
    // Skips rigid bodies without a parent (this includes the world link).
    if (!bodies[body_index]->hasParent()) continue;

    const DrakeJoint& joint = bodies[body_index]->getJoint();
    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();

    // Defines a joint limit tolerance. This is the amount in radians over which
    // joint position limits can be violated and still be considered within the
    // limits. Once we are able to model joint limits via
    // constraints, we may be able to remove the need for this tolerance value.
    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.

    for (int ii = 0; ii < joint.getNumPositions(); ++ii) {
      double position = final_robot_state[robot_state_index++];
      if (position < min_limit[ii] - kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.getName() + " (DOF " +
                                 joint.getPositionName(ii) +
                                 ") violated minimum position limit (" +
                                 std::to_string(position) + " < " +
                                 std::to_string(min_limit[ii]) + ").");
      }
      if (position > max_limit[ii] + kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.getName() + " (DOF " +
                                 joint.getPositionName(ii) +
                                 ") violated maximum position limit (" +
                                 std::to_string(position) + " > " +
                                 std::to_string(max_limit[ii]) + ").");
      }
    }
  }
}

// TODO(naveenoid) : For now this method is copied from the kuka_ik_demo.cc.
// Modify to fit future demo scenario requirements.
void GenerateIKDemoJointTrajectory(
    const std::shared_ptr<RigidBodyTree> iiwa_tree,
    MatrixXd* joint_trajectories,
    std::vector<double>* time_stamps) {
  // Create a basic pointwise IK trajectory for moving the iiwa arm.
  // We start in the zero configuration (straight up).

  MatrixXd joint_trajectories_local;
  std::vector<double> time_stamps_local;

  VectorXd zero_conf = iiwa_tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(iiwa_tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Define an end effector constraint and make it active for the
  // timespan from 1 to 3 seconds.
  Vector3d pos_end;
  pos_end << 0.6, 0, 0.325;
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc(iiwa_tree.get(),
                              iiwa_tree->FindBodyIndex("iiwa_link_ee"),
                              Vector3d::Zero(), pos_lb, pos_ub,
                              Vector2d(1, 3));

  // After the end effector constraint is released, apply the straight
  // up (all joints at position zero) configuration again.
  PostureConstraint pc2(iiwa_tree.get(), Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Bring back the end effector constraint when simulation time reaches 9
  // seconds after the simulation's start time.
  WorldPositionConstraint wpc2(
      iiwa_tree.get(), iiwa_tree->FindBodyIndex("iiwa_link_ee"),
      Vector3d::Zero(), pos_lb, pos_ub, Vector2d(6, 9));

  // For part of the remaining time, constrain the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = iiwa_tree->FindChildBodyOfJoint("iiwa_joint_2")
                                    ->get_position_start_index();
  PostureConstraint pc3(iiwa_tree.get(), Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

  time_stamps_local.push_back(0.0);
  time_stamps_local.push_back(2.0);
  time_stamps_local.push_back(5.0);
  time_stamps_local.push_back(7.0);
  time_stamps_local.push_back(9.0);

  std::vector<RigidBodyConstraint*> constraint_array;

  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);

  int num_constraints = constraint_array.size();

  IKoptions ikoptions(iiwa_tree.get());
  std::vector<int> info(num_constraints);

  MatrixXd q0(iiwa_tree->number_of_positions(), num_constraints);
  q0 = zero_conf.replicate(1, num_constraints);

  joint_trajectories_local =
      MatrixXd::Zero(iiwa_tree->number_of_positions(), num_constraints);
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(iiwa_tree.get(), num_constraints,
    time_stamps_local.data(), q0, q0, constraint_array.size(),
    constraint_array.data(), ikoptions, &joint_trajectories_local,
    info.data(), &infeasible_constraint);

  // Check feasibility of the result at each constraint point.
  bool info_good = true;
  for (int i = 0; i < num_constraints; ++i) {
    if (info[i] != 1) {
      info_good = false;
    }
  }

  if (!info_good) {
    throw std::runtime_error("Inverse Kinematics solution not found.");
  }

  *joint_trajectories = joint_trajectories_local;
  *time_stamps = time_stamps_local;
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
