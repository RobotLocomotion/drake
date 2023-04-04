#include "drake/planning/linear_distance_interpolation_provider.h"

#include <common_robotics_utilities/math.hpp>

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {
using common_robotics_utilities::math::Interpolate;
using common_robotics_utilities::math::InterpolateXd;
using multibody::Body;
using multibody::BodyIndex;
using multibody::Joint;
using multibody::JointIndex;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;

namespace {

std::vector<int> GetQuaternionDofStartIndices(
    const MultibodyPlant<double>& plant) {
  // TODO(SeanCurtis-TRI) Body::has_quaternion_dofs() is actually a misnomer for
  // is_quaternion_floating(). The name implies general quaternion awareness but
  // its documentation doesn't guarantee that. We should re-express this in
  // terms of joints so that we can catch quaternions in any kind of joint.
  std::vector<int> quaternion_dof_start_indices;
  for (BodyIndex body_index(0); body_index < plant.num_bodies(); ++body_index) {
    const Body<double>& body = plant.get_body(body_index);
    if (body.has_quaternion_dofs()) {
      quaternion_dof_start_indices.push_back(body.floating_positions_start());
    }
  }
  return quaternion_dof_start_indices;
}

// TODO(calderpg-tri) This is borked for cases that aren't all revolute joints.
Eigen::VectorXd GetJointDistanceWeights(
    const MultibodyPlant<double>& plant,
    const std::vector<ModelInstanceIndex>& robot_model_instances,
    const std::map<std::string, double>& named_joint_distance_weights) {
  // The default weight for all joints is 1.
  Eigen::VectorXd joint_distance_weights =
      Eigen::VectorXd::Ones(plant.num_positions());

  // Collect all possible joints that are part of the robot model.
  std::vector<JointIndex> joints;
  for (const auto& robot_model_instance : robot_model_instances) {
    auto instance_joints = plant.GetJointIndices(robot_model_instance);
    joints.insert(joints.end(), instance_joints.begin(), instance_joints.end());
  }

  // Go through the model and set joint distance weights accordingly.
  for (const JointIndex& idx : joints) {
    const Joint<double>& joint = plant.get_joint(idx);
    if (joint.num_positions() > 0) {
      DRAKE_THROW_UNLESS(joint.num_positions() == 1);
      // TODO(calderpg-tri) Find a solution that incorporates model instances.
      // Note: this ignores model instances, so two joints with the same name
      // but in different model instances will receive the same weight. This
      // also assumes that joint names are usefully unique; this is not enforced
      // by drake::multibody::Joint, but is reasonably true for any sane model.
      const auto found_itr = named_joint_distance_weights.find(joint.name());
      if (found_itr != named_joint_distance_weights.end()) {
        const double joint_distance_weight = found_itr->second;
        DRAKE_THROW_UNLESS(joint_distance_weight >= 0.0);
        joint_distance_weights(joint.position_start()) = joint_distance_weight;
        drake::log()->debug(
            "Set joint {} [{}] distance weight to non-default {}", idx,
            joint.name(), joint_distance_weight);
      }
    }
  }

  return joint_distance_weights;
}

}  // namespace

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const multibody::MultibodyPlant<double>& plant,
    const std::vector<multibody::ModelInstanceIndex>& robot_model_instances,
    const std::map<std::string, double>& named_joint_distance_weights)
    : quaternion_dof_start_indices_(GetQuaternionDofStartIndices(plant)),
      distance_weights_(GetJointDistanceWeights(
          plant, robot_model_instances, named_joint_distance_weights)) {}

LinearDistanceAndInterpolationProvider::
    ~LinearDistanceAndInterpolationProvider() = default;

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const LinearDistanceAndInterpolationProvider&) = default;

std::unique_ptr<DistanceAndInterpolationProvider>
LinearDistanceAndInterpolationProvider::DoClone() const {
  return std::unique_ptr<DistanceAndInterpolationProvider>(
      new LinearDistanceAndInterpolationProvider(*this));
}

double LinearDistanceAndInterpolationProvider::DoComputeConfigurationDistance(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  return (to - from).cwiseProduct(distance_weights()).norm();
}

Eigen::VectorXd
LinearDistanceAndInterpolationProvider::DoInterpolateBetweenConfigurations(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    const double ratio) const {
  // Start with linear interpolation between from and to.
  Eigen::VectorXd interp = InterpolateXd(from, to, ratio);
  // Handle quaternion dof properly.
  for (const int quat_dof_start_index : quaternion_dof_start_indices()) {
    const Eigen::Quaterniond from_quat(from.segment<4>(quat_dof_start_index));
    const Eigen::Quaterniond to_quat(to.segment<4>(quat_dof_start_index));
    const Eigen::Quaterniond interp_quat =
        Interpolate(from_quat, to_quat, ratio);
    interp.segment<4>(quat_dof_start_index) = interp_quat.coeffs();
  }
  return interp;
}

}  // namespace planning
}  // namespace drake
