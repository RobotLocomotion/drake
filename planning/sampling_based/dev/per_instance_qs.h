#pragma once

#include <unordered_map>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
using PerInstanceQs =
    std::unordered_map<multibody::ModelInstanceIndex, Eigen::VectorXd>;

/** Copies the generalized position values in `per_instance_qs` to `full_q`,
 using a vector layout consistent with `plant`.
 @throws std::exception if `full_q` is nullptr.
 @throws std::exception if `full_q.size() != plant.num_positions()`. */
void SetInstanceQsInFullQ(const multibody::MultibodyPlant<double>& plant,
                          const PerInstanceQs& per_instance_qs,
                          EigenPtr<Eigen::VectorXd> full_q);

}  // namespace planning
}  // namespace drake
