#include "drake/multibody/tree/joint.h"

namespace drake {
namespace multibody {

template <typename T>
Joint<T>::~Joint() = default;

template <typename T>
bool Joint<T>::can_rotate() const {
  DRAKE_DEMAND(this->get_implementation().has_mobilizer());
  return get_implementation().mobilizer->can_rotate();
}

template <typename T>
bool Joint<T>::can_translate() const {
  DRAKE_DEMAND(this->get_implementation().has_mobilizer());
  return get_implementation().mobilizer->can_translate();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Joint);
