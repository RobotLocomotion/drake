#include "drake/multibody/multibody_tree/fixed_offset_frame.h"

#include <exception>
#include <memory>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
const FixedOffsetFrame<T>& FixedOffsetFrame<T>::Create(
    MultibodyTree<T>* tree,
    const Body<T>& body, const Isometry3<T>& X_BF) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  // However we can still create a unique_ptr as below where ownership is clear
  // and an exception would call the destructor.
  return *tree->AddPhysicalFrame(
      std::unique_ptr<FixedOffsetFrame<T>>(
          new FixedOffsetFrame<T>(body.get_body_frame(), X_BF)));
}

template <typename T>
const FixedOffsetFrame<T>& FixedOffsetFrame<T>::Create(
    MultibodyTree<T>* tree,
    const PhysicalFrame<T>& P, const Isometry3<T>& X_PF) {
  if (dynamic_cast<const BodyFrame<T>*>(&P) == nullptr) {
    throw std::logic_error(
        "Chaining of FixedOffsetFrame frames is not yet supported. "
            "Therefore we only allow to fix frames to body frames.");
  }

  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  // However we can still create a unique_ptr as below where ownership is clear
  // and an exception would call the destructor.
  return *tree->AddPhysicalFrame(
      std::unique_ptr<FixedOffsetFrame<T>>(new FixedOffsetFrame<T>(P, X_PF)));
}

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const PhysicalFrame<T>& P, const Isometry3<T>& X_PF) :
    PhysicalFrame<T>(P.get_body()), X_PF_(X_PF) {}

// Explicitly instantiates on the most common scalar types.
template class FixedOffsetFrame<double>;
template class FixedOffsetFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
