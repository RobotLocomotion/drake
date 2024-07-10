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

  // The i'th element of this vector is congruent with the i'th element of the
  // vector of `surfaces` in its related GeometryContactData. Together, this
  // i'th force and i'th surface pair up to form the HydroelasticContactInfo for
  // a hydroelastic contact. See HydroelasticContactInfo::F_Ac_W() for full
  // documentation on the semantics (but in short: it's the force on the body
  // associated with the i'th surface).
  std::vector<SpatialForce<T>> F_Ac_W_array;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
