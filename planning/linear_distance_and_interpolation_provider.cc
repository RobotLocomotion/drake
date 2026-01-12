#include "drake/planning/linear_distance_and_interpolation_provider.h"

#include <stdexcept>

#include <common_robotics_utilities/math.hpp>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"

namespace drake {
namespace planning {
using common_robotics_utilities::math::InterpolateXd;
using multibody::BodyIndex;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using multibody::QuaternionFloatingJoint;
using multibody::RigidBody;

namespace {

std::vector<int> GetQuaternionDofStartIndices(
    const MultibodyPlant<double>& plant) {
  std::vector<int> quaternion_dof_start_indices;

  for (JointIndex joint_index : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(joint_index);
    if (joint.type_name() == QuaternionFloatingJoint<double>::kTypeName) {
      quaternion_dof_start_indices.push_back(joint.position_start());
    }
  }

  return quaternion_dof_start_indices;
}

Eigen::VectorXd GetDefaultDistanceWeights(
    int num_positions, const std::vector<int>& quaternion_dof_start_indices) {
  // The default weight for all joints is 1, except for quaternion DoF where
  // only the first weight may be non-zero.
  Eigen::VectorXd default_distance_weights =
      Eigen::VectorXd::Ones(num_positions);
  for (const int i : quaternion_dof_start_indices) {
    default_distance_weights.segment<4>(i) << 1.0, 0.0, 0.0, 0.0;
  }
  return default_distance_weights;
}

Eigen::VectorXd GetDistanceWeights(
    const MultibodyPlant<double>& plant,
    const std::vector<int>& quaternion_dof_start_indices,
    const std::map<JointIndex, Eigen::VectorXd>& joint_distance_weights) {
  // The default weight for all joints is 1, except for quaternion DoF where
  // only the first weight may be non-zero.
  Eigen::VectorXd distance_weights = GetDefaultDistanceWeights(
      plant.num_positions(), quaternion_dof_start_indices);

  for (const auto& [joint_index, joint_weights] : joint_distance_weights) {
    const Joint<double>& joint = plant.get_joint(joint_index);

    if (joint.num_positions() != joint_weights.size()) {
      throw std::runtime_error(fmt::format(
          "Provided distance weights for joint {} [{}] with type [{}] are [{}] "
          "which do not match that joint's num_positions {}",
          joint_index, joint.name(), joint.type_name(),
          fmt_eigen(joint_weights.transpose()), joint.num_positions()));
    }

    for (int i = 0; i < joint_weights.size(); ++i) {
      const double weight = joint_weights(i);
      if (!std::isfinite(weight) || weight < 0.0) {
        throw std::runtime_error(fmt::format(
            "Provided distance weights for joint {} [{}] are [{}] which are not"
            " non-negative and finite",
            joint_index, joint.name(), fmt_eigen(joint_weights.transpose())));
      }
    }

    distance_weights.segment(joint.position_start(), joint.num_positions()) =
        joint_weights;
  }

  return distance_weights;
}

/* Checks that provided distance weights satisfy preconditions:
- distance_weights.size() must match num_positions
- all weights must be non-negative and finite
- all quaternion DoF weights must be of the form (weight, 0, 0, 0)

Returns distance_weights unchanged if preconditions are satisfied, throws
otherwise. */
Eigen::VectorXd SanityCheckDistanceWeights(
    int num_positions, const std::vector<int>& quaternion_dof_start_indices,
    const Eigen::VectorXd& distance_weights) {
  if (num_positions != distance_weights.size()) {
    throw std::runtime_error(fmt::format(
        "Provided distance weights size {} does not match num_positions {}",
        distance_weights.size(), num_positions));
  }

  // Every weight must be finite and >= 0.
  for (int i = 0; i < distance_weights.size(); ++i) {
    const double weight = distance_weights(i);
    if (!std::isfinite(weight)) {
      throw std::runtime_error(
          fmt::format("Provided distance weight {} with value {} is not finite",
                      i, weight));
    }

    if (weight < 0.0) {
      throw std::runtime_error(fmt::format(
          "Provided distance weight {} with value {} is less than zero", i,
          weight));
    }
  }

  // Only the first weight for quaternion DoF may be non-zero.
  for (const int i : quaternion_dof_start_indices) {
    const double w_weight = distance_weights(i);
    const double x_weight = distance_weights(i + 1);
    const double y_weight = distance_weights(i + 2);
    const double z_weight = distance_weights(i + 3);

    if (x_weight != 0.0 || y_weight != 0.0 || z_weight != 0.0) {
      throw std::runtime_error(fmt::format(
          "Provided distance weights for quaternion dof starting at index {} "
          "with values ({}, {}, {}, {}) must be ({}, 0, 0, 0) instead",
          i, w_weight, x_weight, y_weight, z_weight, w_weight));
    }
  }

  return distance_weights;
}

}  // namespace

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const MultibodyPlant<double>& plant,
    const std::map<JointIndex, Eigen::VectorXd>& joint_distance_weights)
    : quaternion_dof_start_indices_(GetQuaternionDofStartIndices(plant)),
      distance_weights_(SanityCheckDistanceWeights(
          plant.num_positions(), quaternion_dof_start_indices_,
          GetDistanceWeights(plant, quaternion_dof_start_indices_,
                             joint_distance_weights))) {}

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const MultibodyPlant<double>& plant,
    const Eigen::VectorXd& distance_weights)
    : quaternion_dof_start_indices_(GetQuaternionDofStartIndices(plant)),
      distance_weights_(SanityCheckDistanceWeights(
          plant.num_positions(), quaternion_dof_start_indices_,
          distance_weights)) {}

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const MultibodyPlant<double>& plant)
    : quaternion_dof_start_indices_(GetQuaternionDofStartIndices(plant)),
      distance_weights_(SanityCheckDistanceWeights(
          plant.num_positions(), quaternion_dof_start_indices_,
          GetDefaultDistanceWeights(plant.num_positions(),
                                    quaternion_dof_start_indices_))) {}

LinearDistanceAndInterpolationProvider::
    ~LinearDistanceAndInterpolationProvider() = default;

double LinearDistanceAndInterpolationProvider::DoComputeConfigurationDistance(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  Eigen::VectorXd deltas = to - from;
  for (const int quat_dof_start_index : quaternion_dof_start_indices()) {
    const Eigen::Quaterniond from_quat(from.segment<4>(quat_dof_start_index));
    const Eigen::Quaterniond to_quat(to.segment<4>(quat_dof_start_index));
    const double quat_angle = from_quat.angularDistance(to_quat);
    deltas.segment<4>(quat_dof_start_index) << quat_angle, 0.0, 0.0, 0.0;
  }
  return deltas.cwiseProduct(distance_weights()).norm();
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
    // Make sure interpolation always does shortest angle (<= 2pi)
    const Eigen::Quaterniond interp_quat = from_quat.slerp(ratio, to_quat);
    interp.segment<4>(quat_dof_start_index) = interp_quat.coeffs();
  }
  return interp;
}

}  // namespace planning
}  // namespace drake
