#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/distance_and_interpolation_provider.h"

namespace drake {
namespace planning {

class LinearDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearDistanceAndInterpolationProvider);

  LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant,
      const std::map<std::string, double>& named_joint_distance_weights);

  LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant,
      const Eigen::VectorXd& distance_weights);

  explicit LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant);

  ~LinearDistanceAndInterpolationProvider() final;

  const Eigen::VectorXd& distance_weights() const { return distance_weights_; }

  const std::vector<int>& quaternion_dof_start_indices() const {
    return quaternion_dof_start_indices_;
  }

 private:
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
