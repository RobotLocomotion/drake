#include "drake/multibody/tree/mobilizer.h"

#include <sstream>
#include <string>

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
Mobilizer<T>::~Mobilizer() = default;

template <typename T>
std::pair<Eigen::Quaternion<T>, Vector3<T>> Mobilizer<T>::GetPosePair(
    const systems::Context<T>& context) const {
  const math::RigidTransform<T> X_FM = CalcAcrossMobilizerTransform(context);
  return std::pair(X_FM.rotation().ToQuaternion(), X_FM.translation());
}

template <typename T>
void Mobilizer<T>::MapVelocityDotToQDDot(const systems::Context<T>&,
                                         const Eigen::Ref<const VectorX<T>>&,
                                         EigenPtr<VectorX<T>>) const {
  // TODO(Mitiguy) remove this function when Mobilizer::MapVelocityDotToQDDot()
  //  is changed to a pure virtual function that requires override.
  const std::string error_message = fmt::format(
      "The function {}() has not been implemented for this "
      "mobilizer.",
      __func__);
  throw std::logic_error(error_message);
}

template <typename T>
void Mobilizer<T>::MapQDDotToVelocityDot(const systems::Context<T>&,
                                         const Eigen::Ref<const VectorX<T>>&,
                                         EigenPtr<VectorX<T>>) const {
  // TODO(Mitiguy) remove this function when Mobilizer::MapVelocityDotToQDDot()
  //  is changed to a pure virtual function that requires override.
  const std::string error_message = fmt::format(
      "The function {}() has not been implemented for this "
      "mobilizer.",
      __func__);
  throw std::logic_error(error_message);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::Mobilizer);
