#include "drake/multibody/tree/frame.h"

#include "drake/common/identifier.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace internal {

std::string DeprecateWhenEmptyName(std::string name, std::string_view type) {
  if (name.empty()) {
    throw std::runtime_error(fmt::format(
        "The name parameter to the {} constructor is required.", type));
  }
  return name;
}

}  // namespace internal

template <typename T>
ScopedName Frame<T>::scoped_name() const {
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

}  // namespace multibody
}  // namespace drake

// Ideally, we'd be instantiating the entire class here, instead of just one
// member function. However, the MultibodyTree physical design is so contrary to
// GSG best practices that trying to do the entire class here doesn't work.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &drake::multibody::Frame<T>::scoped_name
))
