#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/distance_interpolation_provider.h"

namespace drake {
namespace planning {

class LinearDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  // The copy constructor is private for use in implementing DoClone().
  // Does not allow copy, move, or assignment.
  LinearDistanceAndInterpolationProvider(
      LinearDistanceAndInterpolationProvider&&) = delete;
  LinearDistanceAndInterpolationProvider& operator=(
      const LinearDistanceAndInterpolationProvider&) = delete;
  LinearDistanceAndInterpolationProvider& operator=(
      LinearDistanceAndInterpolationProvider&&) = delete;

  LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant,
      const std::vector<multibody::ModelInstanceIndex>& robot_model_instances,
      const std::map<std::string, double>& named_joint_distance_weights);

  ~LinearDistanceAndInterpolationProvider() final;

  const Eigen::VectorXd& distance_weights() const { return distance_weights_; }

  const std::vector<int>& quaternion_dof_start_indices() const {
    return quaternion_dof_start_indices_;
  }

 private:
  LinearDistanceAndInterpolationProvider(
      const LinearDistanceAndInterpolationProvider& other);

  std::unique_ptr<DistanceAndInterpolationProvider> DoClone() const final;

  double DoComputeConfigurationDistance(const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& to) const final;

  Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const final;

  const std::vector<int> quaternion_dof_start_indices_;
  const Eigen::VectorXd distance_weights_;
};

}  // namespace planning
}  // namespace drake
