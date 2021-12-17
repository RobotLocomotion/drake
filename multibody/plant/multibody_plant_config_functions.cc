#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include <array>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

using AddResult = AddMultibodyPlantSceneGraphResult<double>;

AddResult AddMultibodyPlant(
    const MultibodyPlantConfig& config,
    systems::DiagramBuilder<double>* builder) {
  AddResult result = AddMultibodyPlantSceneGraph(builder, config.time_step);
  result.plant.set_penetration_allowance(config.penetration_allowance);
  result.plant.set_stiction_tolerance(config.stiction_tolerance);
  result.plant.set_contact_model(
      internal::GetContactModelFromString(config.contact_model));
  return result;
}

namespace internal {
namespace {

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody add a new value to the enum. New values must be listed here
// as well as in the list of kContactModels below.
constexpr const char* ContactModelToChars(ContactModel contact_model) {
  switch (contact_model) {
    case ContactModel::kPointContactOnly:
      return "point";
    case ContactModel::kHydroelasticsOnly:
      return "hydroelastic";
    case ContactModel::kHydroelasticWithFallback:
      return "hydroelastic_with_fallback";
  }
}

constexpr auto MakeContactModelPair(ContactModel value) {
  return std::pair(value, ContactModelToChars(value));
}

constexpr std::array<std::pair<ContactModel, const char*>, 3> kContactModels{{
  MakeContactModelPair(ContactModel::kPointContactOnly),
  MakeContactModelPair(ContactModel::kHydroelasticsOnly),
  MakeContactModelPair(ContactModel::kHydroelasticWithFallback),
}};

}  // namespace

ContactModel GetContactModelFromString(std::string_view contact_model) {
  for (const auto& [value, name] : kContactModels) {
    if (name == contact_model) {
      return value;
    }
  }
  throw std::logic_error(fmt::format(
      "Unknown contact_model: '{}'", contact_model));
}

std::string GetStringFromContactModel(ContactModel contact_model) {
  for (const auto& [value, name] : kContactModels) {
    if (value == contact_model) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
