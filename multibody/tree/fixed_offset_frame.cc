#include "drake/multibody/tree/fixed_offset_frame.h"

#include <exception>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const std::string& name, const Frame<T>& P,
    const math::RigidTransform<double>& X_PF,
    std::optional<ModelInstanceIndex> model_instance) :
    Frame<T>(name, P.body(), model_instance.value_or(P.model_instance())),
    parent_frame_(P), X_PF_(X_PF) {}

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(const std::string& name, const Body<T>& B,
                                      const math::RigidTransform<double>& X_BF)
    : Frame<T>(name, B), parent_frame_(B.body_frame()), X_PF_(X_BF) {}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> FixedOffsetFrame<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyElementAccessor<ToScalar, T>& handle) const {
  const Frame<ToScalar>& parent_frame_clone =
      handle.get_variant(parent_frame_);
  return std::make_unique<FixedOffsetFrame<ToScalar>>(
      handle.get_new_name(*this), parent_frame_clone, X_PF_);
}

template <typename T>
std::unique_ptr<Frame<double>> FixedOffsetFrame<T>::DoCloneToScalar(
      const internal::MultibodyElementAccessor<double, T>& handle) const {
  return TemplatedDoCloneToScalar(handle);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> FixedOffsetFrame<T>::DoCloneToScalar(
      const internal::MultibodyElementAccessor<AutoDiffXd, T>& handle) const {
  return TemplatedDoCloneToScalar(handle);
}

template <typename T>
std::unique_ptr<Frame<symbolic::Expression>>
FixedOffsetFrame<T>::DoCloneToScalar(
    const internal::MultibodyElementAccessor<symbolic::Expression, T>& handle)
    const {
  return TemplatedDoCloneToScalar(handle);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::FixedOffsetFrame)
