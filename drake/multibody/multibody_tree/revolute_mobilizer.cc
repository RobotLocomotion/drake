#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

#include "drake/common/eigen_types.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class RevoluteMobilizer<double>;
template class RevoluteMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
