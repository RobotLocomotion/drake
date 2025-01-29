#include "drake/multibody/tree/force_element.h"

#include "drake/common/default_scalars.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {

template <typename T>
ForceElement<T>::~ForceElement() = default;

template <typename T>
std::unique_ptr<ForceElement<T>> ForceElement<T>::ShallowClone() const {
  std::unique_ptr<ForceElement<T>> result = DoShallowClone();
  DRAKE_THROW_UNLESS(result != nullptr);
  return result;
}

template <typename T>
std::unique_ptr<ForceElement<T>> ForceElement<T>::DoShallowClone() const {
  throw std::logic_error(fmt::format("{} failed to override DoShallowClone()",
                                     NiceTypeName::Get(*this)));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ForceElement);
