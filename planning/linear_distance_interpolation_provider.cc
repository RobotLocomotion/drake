#include "drake/planning/linear_distance_interpolation_provider.h"

#include <stdexcept>

#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/print.hpp>
#include <fmt/format.h>

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {
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

Eigen::VectorXd GetJointDistanceWeights(
    const MultibodyPlant<double>& plant,
    const std::vector<int>& quaternion_dof_start_indices,
    const std::map<std::string, double>& named_joint_distance_weights) {
  // The default weight for all joints is 1, except for quaternion DoF where
  // only the first weight may be non-zero.
  Eigen::VectorXd joint_distance_weights =
      Eigen::VectorXd::Ones(plant.num_positions());
  for (const int i : quaternion_dof_start_indices) {
    joint_distance_weights.segment<4>(i) << 1.0, 0.0, 0.0, 0.0;
  }

  // Go through the model and set joint distance weights accordingly.
  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const Joint<double>& joint = plant.get_joint(i);
    if (joint.num_positions() > 0) {
      // TODO(calderpg-tri) Find a solution that incorporates model instances.
      // Note: this ignores model instances, so two joints with the same name
      // but in different model instances will receive the same weight. This
      // also assumes that joint names are usefully unique; this is not enforced
      // by drake::multibody::Joint, but is reasonably true for any sane model.
      const auto found = named_joint_distance_weights.find(joint.name());
      if (found != named_joint_distance_weights.end()) {
        const double joint_distance_weight = found->second;
        DRAKE_THROW_UNLESS(joint_distance_weight >= 0.0);

        // TODO(calderpg-tri) Find a way to support more joint types.
        if (joint.num_positions() == 1) {
          joint_distance_weights(joint.position_start()) =
              joint_distance_weight;
          drake::log()->debug(
              "Set single position joint {} [{}] distance weight to {}", i,
              joint.name(), joint_distance_weight);
        } else {
          throw std::runtime_error(fmt::format(
              "Joint {} [{}] has type [{}] not supported when named joint "
              "distance weights, construct "
              "LinearDistanceAndInterpolationProvider with a vector of distance"
              " weights instead",
              i, joint.name(), joint.type_name()));
        }
      }
    }
  }

  return joint_distance_weights;
}

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
          "with values ({}, {}, {}, {}) must be ({}, 0.0, 0.0, 0.0) instead",
          i, w_weight, x_weight, y_weight, z_weight, w_weight));
    }
  }

  return distance_weights;
}

}  // namespace

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const multibody::MultibodyPlant<double>& plant,
    const std::map<std::string, double>& named_joint_distance_weights)
    : quaternion_dof_start_indices_(GetQuaternionDofStartIndices(plant)),
      distance_weights_(SanityCheckDistanceWeights(
          plant.num_positions(), quaternion_dof_start_indices_,
          GetJointDistanceWeights(plant, quaternion_dof_start_indices_,
                                  named_joint_distance_weights))) {}

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& distance_weights)
    : quaternion_dof_start_indices_(GetQuaternionDofStartIndices(plant)),
      distance_weights_(SanityCheckDistanceWeights(
          plant.num_positions(), quaternion_dof_start_indices_,
          distance_weights)) {}

LinearDistanceAndInterpolationProvider::
    ~LinearDistanceAndInterpolationProvider() = default;

void LinearDistanceAndInterpolationProvider::SetDistanceWeights(
    const Eigen::VectorXd& distance_weights) {
  distance_weights_ = SanityCheckDistanceWeights(distance_weights_.size(),
                                                 quaternion_dof_start_indices(),
                                                 distance_weights);
}

LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider(
    const LinearDistanceAndInterpolationProvider&) = default;

std::unique_ptr<DistanceAndInterpolationProvider>
LinearDistanceAndInterpolationProvider::DoClone() const {
  return std::unique_ptr<DistanceAndInterpolationProvider>(
      new LinearDistanceAndInterpolationProvider(*this));
}

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
