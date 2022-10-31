#include "drake/multibody/tree/body.h"

#include <memory>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> BodyFrame<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyElementAccessor<ToScalar, T>& handle) const {
  const Body<ToScalar>& body_clone = handle.get_variant(this->body());
  // BodyFrame's constructor cannot be called from std::make_unique since it is
  // private and therefore we use "new".
  return std::unique_ptr<BodyFrame<ToScalar>>(
      new BodyFrame<ToScalar>(body_clone));
}

template <typename T>
std::unique_ptr<Frame<double>> BodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyElementAccessor<double, T>& handle) const {
  return TemplatedDoCloneToScalar(handle);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> BodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyElementAccessor<AutoDiffXd, T>& handle) const {
  return TemplatedDoCloneToScalar(handle);
}

template <typename T>
std::unique_ptr<Frame<symbolic::Expression>> BodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyElementAccessor<symbolic::Expression, T>& handle)
    const {
  return TemplatedDoCloneToScalar(handle);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::BodyFrame)
