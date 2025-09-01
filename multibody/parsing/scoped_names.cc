#include "drake/multibody/parsing/scoped_names.h"

#include <string>

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace parsing {

template <typename T>
const drake::multibody::Frame<T>* GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<T>& plant,
    std::string_view full_name) {
  if (full_name == "world") {
    return &plant.world_frame();
  }
  auto scoped_name = ScopedName::Parse(std::string{full_name});
  if (!scoped_name.get_namespace().empty()) {
    if (plant.HasModelInstanceNamed(scoped_name.get_namespace())) {
      auto instance = plant.GetModelInstanceByName(scoped_name.get_namespace());
      if (plant.HasFrameNamed(scoped_name.get_element(), instance)) {
        return &plant.GetFrameByName(scoped_name.get_element(), instance);
      }
    }
  } else if (plant.HasFrameNamed(scoped_name.get_element())) {
    return &plant.GetFrameByName(scoped_name.get_element());
  }
  return nullptr;
}

template <typename T>
const drake::multibody::Frame<T>& GetScopedFrameByName(
    const drake::multibody::MultibodyPlant<T>& plant,
    std::string_view full_name) {
  if (full_name == "world") {
    return plant.world_frame();
  }
  auto scoped_name = ScopedName::Parse(std::string{full_name});
  if (!scoped_name.get_namespace().empty()) {
    auto instance = plant.GetModelInstanceByName(scoped_name.get_namespace());
    return plant.GetFrameByName(scoped_name.get_element(), instance);
  } else {
    return plant.GetFrameByName(scoped_name.get_element());
  }
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &GetScopedFrameByNameMaybe<T>,
    &GetScopedFrameByName<T>
));
// clang-format on

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
