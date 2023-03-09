#include "drake/multibody/parsing/scoped_names.h"

#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace parsing {

namespace {
// TODO(jwnimmer-tri) Remove me 2023-07-01 as part of deprecation.
constexpr char kScopedNameDelim[] = "::";
}  // namespace

const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  if (full_name == "world")
    return &plant.world_frame();
  auto scoped_name = multibody::ScopedName::Parse(full_name);
  if (!scoped_name.get_namespace().empty()) {
    auto instance = plant.GetModelInstanceByName(scoped_name.get_namespace());
    if (plant.HasFrameNamed(scoped_name.get_element(), instance)) {
      return &plant.GetFrameByName(scoped_name.get_element(), instance);
    }
  } else if (plant.HasFrameNamed(scoped_name.get_element())) {
    return &plant.GetFrameByName(scoped_name.get_element());
  }
  return nullptr;
}

const drake::multibody::Frame<double>& GetScopedFrameByName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  auto* frame = GetScopedFrameByNameMaybe(plant, full_name);
  if (frame == nullptr) {
    throw std::runtime_error("Could not find frame: " + full_name);
  }
  return *frame;
}

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push
// Remove 2023-07-01 with deprecation.
std::string GetScopedFrameName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame) {
  if (&frame == &plant.world_frame())
    return "world";
  return PrefixName(GetInstanceScopeName(
      plant, frame.model_instance()), frame.name());
}
#pragma GCC diagnostic pop

// Remove 2023-07-01 with deprecation.
ScopedNameIsDeprecated ParseScopedName(const std::string& full_name) {
  size_t pos = full_name.rfind(kScopedNameDelim);
  ScopedNameIsDeprecated result;
  if (pos == std::string::npos) {
    result.name = full_name;
  } else {
    result.instance_name = full_name.substr(0, pos);
    // "Global scope" (e.g. "::my_frame") not supported.
    DRAKE_DEMAND(!result.instance_name.empty());
    result.name = full_name.substr(pos + std::string(kScopedNameDelim).size());
  }
  return result;
}

// Remove 2023-07-01 with deprecation.
std::string PrefixName(const std::string& namespace_, const std::string& name) {
  if (namespace_.empty())
    return name;
  else if (name.empty())
    return namespace_;
  else
    return namespace_ + kScopedNameDelim + name;
}

// Remove 2023-07-01 with deprecation.
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
