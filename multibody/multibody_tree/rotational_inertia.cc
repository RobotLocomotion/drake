#include "drake/multibody/multibody_tree/rotational_inertia.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class RotationalInertia<double>;
template class RotationalInertia<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
