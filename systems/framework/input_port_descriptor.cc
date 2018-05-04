#include "drake/systems/framework/input_port_descriptor.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template <typename T>
const System<T>* InputPortDescriptor<T>::get_system() const {
  return dynamic_cast<const System<T>*>(&get_system_base());
}

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class InputPortDescriptor<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class InputPortDescriptor<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::InputPortDescriptor)
