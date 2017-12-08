#include "drake/multibody/multibody_tree/unit_inertia.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class UnitInertia<double>;
template class UnitInertia<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
