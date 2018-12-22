#include "drake/multibody/tree/fixed_offset_frame.h"

#include <exception>
#include <memory>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const std::string& name, const Frame<T>& P,
    const Isometry3<double>& X_PF,
    optional<ModelInstanceIndex> model_instance) :
    Frame<T>(name, P.body(), model_instance.value_or(P.model_instance())),
    parent_frame_(P), X_PF_(X_PF) {}

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const std::string& name, const Body<T>& B, const Isometry3<double>& X_BF) :
    Frame<T>(name, B), parent_frame_(B.body_frame()), X_PF_(X_BF) {}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> FixedOffsetFrame<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& parent_frame_clone =
      tree_clone.get_variant(parent_frame_);
  return std::make_unique<FixedOffsetFrame<ToScalar>>(
      this->name(), parent_frame_clone, X_PF_);
}

template <typename T>
std::unique_ptr<Frame<double>> FixedOffsetFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> FixedOffsetFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class FixedOffsetFrame<double>;
template class FixedOffsetFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
