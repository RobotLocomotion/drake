#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace parsing {

constexpr char const* kDelim = "::";

const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  if (full_name == "world")
    return &plant.world_frame();
  auto result = ParseScopedName(full_name);
  if (!result.instance_name.empty()) {
    auto instance = plant.GetModelInstanceByName(result.instance_name);
    if (plant.HasFrameNamed(result.name, instance)) {
      return &plant.GetFrameByName(result.name, instance);
    }
  } else if (plant.HasFrameNamed(result.name)) {
    return &plant.GetFrameByName(result.name);
  }
  return nullptr;
}

std::string GetScopedFrameName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame) {
  if (&frame == &plant.world_frame())
    return "world";
  return PrefixName(GetInstanceScopeName(
      plant, frame.model_instance()), frame.name());
}

ScopedName ParseScopedName(const std::string& full_name) {
  size_t pos = full_name.rfind(kDelim);
  ScopedName result;
  if (pos == std::string::npos) {
    result.name = full_name;
  } else {
    result.instance_name = full_name.substr(0, pos);
    // "Global scope" (e.g. "::my_frame") not supported.
    DRAKE_DEMAND(!result.instance_name.empty());
    result.name = full_name.substr(pos + std::string(kDelim).size());
  }
  return result;
}

std::string PrefixName(const std::string& namespace_, const std::string& name) {
  if (namespace_.empty())
    return name;
  else if (name.empty())
    return namespace_;
  else
    return namespace_ + kDelim + name;
}

std::string GetInstanceScopeName(
    const MultibodyPlant<double>& plant,
    ModelInstanceIndex instance) {
  if (instance != plant.world_body().model_instance()) {
    return plant.GetModelInstanceName(instance);
  } else {
    return "";
  }
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
