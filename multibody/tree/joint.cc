#include "drake/multibody/tree/joint.h"

namespace drake {
namespace multibody {

template <typename T>
bool Joint<T>::can_rotate() const {
  const std::vector<internal::MobilizedBody<T>*>& mobilizers =
      get_implementation().mobilizers_;
  for (const internal::MobilizedBody<T>* mobilizer : mobilizers) {
    if (mobilizer->can_rotate()) return true;
  }
  return false;
}

template <typename T>
bool Joint<T>::can_translate() const {
  const std::vector<internal::MobilizedBody<T>*>& mobilizers =
      get_implementation().mobilizers_;
  for (const internal::MobilizedBody<T>* mobilizer : mobilizers) {
    if (mobilizer->can_translate()) return true;
  }
  return false;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Joint)
