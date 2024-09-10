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

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::Mobilizer);
