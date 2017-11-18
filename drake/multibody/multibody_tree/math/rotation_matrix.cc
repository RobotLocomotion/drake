#include "drake/multibody/multibody_tree/math/rotation_matrix.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class RotationMatrix<double>;
template class RotationMatrix<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
