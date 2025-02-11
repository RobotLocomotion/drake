#include "drake/multibody/parsing/scoped_names.h"

#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace parsing {

const drake::multibody::Frame<double>* GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  if (full_name == "world") {
    return &plant.world_frame();
  }
  auto scoped_name = multibody::ScopedName::Parse(full_name);
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

const drake::multibody::Frame<double>& GetScopedFrameByName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  if (full_name == "world") {
    return plant.world_frame();
  }
  auto scoped_name = multibody::ScopedName::Parse(full_name);
  if (!scoped_name.get_namespace().empty()) {
    auto instance = plant.GetModelInstanceByName(scoped_name.get_namespace());
    return plant.GetFrameByName(scoped_name.get_element(), instance);
  } else {
    return plant.GetFrameByName(scoped_name.get_element());
  }
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
