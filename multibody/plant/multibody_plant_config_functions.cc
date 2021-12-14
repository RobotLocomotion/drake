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
  result.plant.set_contact_surface_representation(
      internal::GetContactSurfaceRepresentationFromString(
          config.contact_surface_representation));
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

// Take an alias to limit verbosity, especially in the constexpr boilerplate.
using Rep = geometry::HydroelasticContactRepresentation;

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody add a new value to the enum. New values must be listed here
// as well as in the list of kReps below.
constexpr const char* RepToChars(Rep contact_model) {
  switch (contact_model) {
    case Rep::kTriangle:
      return "triangle";
    case Rep::kPolygon:
      return "polygon";
  }
}

constexpr auto MakeRepPair(Rep value) {
  return std::pair(value, RepToChars(value));
}

constexpr std::array<std::pair<Rep, const char*>, 2> kReps{{
  MakeRepPair(Rep::kTriangle),
  MakeRepPair(Rep::kPolygon),
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

Rep GetContactSurfaceRepresentationFromString(
    std::string_view contact_representation) {
  for (const auto& [value, name] : kReps) {
    if (name == contact_representation) {
      return value;
    }
  }
  throw std::logic_error(fmt::format(
      "Unknown hydroelastic contact representation: '{}'",
      contact_representation));
}

std::string GetStringFromContactSurfaceRepresentation(
    Rep contact_representation) {
  for (const auto& [value, name] : kReps) {
    if (value == contact_representation) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
