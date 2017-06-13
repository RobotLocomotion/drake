#include "drake/multibody/multibody_tree/fixed_offset_frame.h"

#include <exception>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const Frame<T>& P, const Isometry3<T>& X_PF) :
    Frame<T>(P.get_body()), parent_frame_(P), X_PF_(X_PF) {}

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const Body<T>& B, const Isometry3<T>& X_BF) :
    Frame<T>(B), parent_frame_(B.get_body_frame()), X_PF_(X_BF) {}

// Explicitly instantiates on the most common scalar types.
template class FixedOffsetFrame<double>;
template class FixedOffsetFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
