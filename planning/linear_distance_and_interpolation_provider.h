#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/distance_and_interpolation_provider.h"

namespace drake {
namespace planning {

/** This class represents a basic "linear" implementation of
DistanceAndInterpolationProvider.

- Configuration distance is computed as difference.cwiseProduct(weights).norm(),
where difference is computed as the angle between quaternion DoF and difference
between all other positions. Default weights are (1, 0, 0, 0) for quaternion
DoF and 1 for all other positions.
- Configuration interpolation is performed using slerp for quaternion DoF and
linear interpolation for all other positions. */
class LinearDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearDistanceAndInterpolationProvider);

  /** Constructs a LinearDistanceAndInterpolationProvider for the specified
  `plant` using the provided map of distance weights `joint_distance_weights`
  and default weights (i.e. 1) for all other positions.
  @pre all distance weights must be non-negative and finite. */
  LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant,
      const std::map<multibody::JointIndex, Eigen::VectorXd>&
          joint_distance_weights);

  /** Constructs a LinearDistanceAndInterpolationProvider for the specified
  `plant` with the provided distance weights `distance_weights`.
  @pre distance_weights must be the same size as plant.num_positions(), all
  weights must be non-negative and finite, and weights for quaternion DoF must
  be of the form (weight, 0, 0, 0). */
  LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant,
      const Eigen::VectorXd& distance_weights);

  /** Constructs a LinearDistanceAndInterpolationProvider for the specified
  `plant` with default distance weights (i.e. 1) for all positions. Equivalent
  to constructing with an empty map of named joint distance weights. */
  explicit LinearDistanceAndInterpolationProvider(
      const multibody::MultibodyPlant<double>& plant);

  ~LinearDistanceAndInterpolationProvider() final;

  /** Gets the distance weights. */
  const Eigen::VectorXd& distance_weights() const { return distance_weights_; }

  /** Gets the start indices for quaternion DoF in the position vector. */
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
