#include "drake/multibody/multibody_tree/spatial_inertia.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class SpatialInertia<double>;
template class SpatialInertia<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
