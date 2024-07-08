#pragma once

#include <vector>

#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/plant/hydroelastic_contact_info.h"

namespace drake {
namespace multibody {
namespace internal {

// Structure used in the calculation of hydroelastic contact forces.
template <typename T>
struct HydroelasticContactForcesContinuousCacheData {
  explicit HydroelasticContactForcesContinuousCacheData(int num_bodies) {
    F_BBo_W_array.resize(num_bodies);
  }

  // Forces from hydroelastic contact applied to the origin of each body
  // (indexed by MobodIndex) in the MultibodyPlant.
  std::vector<SpatialForce<T>> F_BBo_W_array;

  // Information used for contact reporting collected through the evaluation
  // of the hydroelastic model.
  std::vector<HydroelasticContactInfo<T>> contact_info;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
