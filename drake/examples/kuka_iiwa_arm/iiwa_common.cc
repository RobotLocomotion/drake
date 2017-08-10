#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::aligned_allocator;
using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
Matrix6<T> ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<T>& world_tree,
    int iiwa_instance, const std::string& end_effector_link_name,
    int wsg_instance) {
  KinematicsCache<T> world_cache = world_tree.CreateKinematicsCache();
  world_cache.initialize(world_tree.getZeroConfiguration());
  world_tree.doKinematics(world_cache);

  const RigidBody<T>* end_effector = world_tree.FindBody(
      end_effector_link_name, "iiwa14", iiwa_instance);
  Isometry3<T> X_WEE =
    world_tree.CalcBodyPoseInWorldFrame(world_cache, *end_effector);

  // The inertia of the added gripper is lumped into the last link of the
  // controller's iiwa arm model. This is motivated by the fact that the
  // gripper inertia is relatively large compared to the last couple links
  // in the iiwa arm model. And to completely rely on using feedback to cope
  // with added inertia, we need to either rely on larger gains (which will
  // cause simulation to explode without the gripper), or wait longer for
  // the integrator to kick in.

  // Computes the lumped inertia for the gripper.
  std::set<int> gripper_instance_set = {wsg_instance};
  Matrix6<T> lumped_gripper_inertia_W =
    world_tree.LumpedSpatialInertiaInWorldFrame(
        world_cache, gripper_instance_set);
  // Transfer it to the last iiwa link's body frame.
  Matrix6<T> lumped_gripper_inertia_EE =
      transformSpatialInertia(X_WEE.inverse(), lumped_gripper_inertia_W);
  lumped_gripper_inertia_EE += end_effector->get_spatial_inertia();

  return lumped_gripper_inertia_EE;
}

template Matrix6<double>
ComputeLumpedGripperInertiaInEndEffectorFrame(
    const RigidBodyTree<double>&, int, const std::string&, int);

void VerifyIiwaTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_1"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_2"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_3"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_3"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_4"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_4"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_5"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_5"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_6"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_6"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_7"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_7"] == joint_idx++);
}

void CreateTreedFromFixedModelAtPose(const std::string& model_file_name,
                                     RigidBodyTreed* tree,
                                     const Vector3d& position,
                                     const Vector3d& orientation) {
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr, position,
      orientation);

  // TODO(naveenoid) : consider implementing SDF version of this method.
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      model_file_name, drake::multibody::joints::kFixed,
      weld_to_frame, tree);
}

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(7);
  *Kp << 100, 100, 100, 100, 100, 100, 100;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(7);
}

robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot,
    const std::vector<double>& time,
    const std::vector<int>& info,
    const MatrixX<double>& keyframes) {
  DRAKE_DEMAND(info.size() == time.size());
  DRAKE_DEMAND(keyframes.cols() == static_cast<int>(time.size()));
  DRAKE_DEMAND(keyframes.rows() == robot.get_num_positions());

  const int num_time_steps = keyframes.cols();

  robotlocomotion::robot_plan_t plan{};
  plan.utime = 0;  // I (sam.creasey) don't think this is used?
  plan.robot_name = "iiwa";  // Arbitrary, probably ignored
  plan.num_states = num_time_steps;
  const bot_core::robot_state_t default_robot_state{};
  plan.plan.resize(num_time_steps, default_robot_state);
  plan.plan_info.resize(num_time_steps, 0);
  /// Encode the q_sol returned for each timestep into the vector of
  /// robot states.
  for (int i = 0; i < num_time_steps; i++) {
    bot_core::robot_state_t& step = plan.plan[i];
    step.utime = time[i] * 1e6;
    step.num_joints = keyframes.rows();
    for (int j = 0; j < step.num_joints; j++) {
      step.joint_name.push_back(robot.get_position_name(j));
      step.joint_position.push_back(keyframes(j, i));
      step.joint_velocity.push_back(0);
      step.joint_effort.push_back(0);
    }
    plan.plan_info[i] = info[i];
  }
  plan.num_grasp_transitions = 0;
  plan.left_arm_control_type = plan.POSITION;
  plan.right_arm_control_type = plan.NONE;
  plan.left_leg_control_type = plan.NONE;
  plan.right_leg_control_type = plan.NONE;
  plan.num_bytes = 0;

  return plan;
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
