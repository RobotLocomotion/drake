#include "drake/multibody/plant/external_force_field.h"

namespace drake {
namespace multibody {

template <typename T>
Vector3<T> ExternalForceField<T>::Eval(const Vector3<T>& p_WQ) const {
  Vector3<T> result = Vector3<T>::Zero();
  for (const auto& f : fields_) {
    result += f(p_WQ);
  }
  return result;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ExternalForceField)
