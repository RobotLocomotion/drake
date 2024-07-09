#pragma once

#include <vector>

#include "drake/multibody/math/spatial_force.h"

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

  // The spatial force applied at the centroid (Point C) of the surface mesh.
  // The element ordering is congruent with the vector of surfaces in
  // GeometryContactData.
  std::vector<SpatialForce<T>> F_Ac_W_array;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
