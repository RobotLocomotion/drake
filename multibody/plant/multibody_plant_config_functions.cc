#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include <array>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"

// Remove on 2026-09-01 per TAMSI deprecation.
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {

using AddResult = AddMultibodyPlantSceneGraphResult<double>;

AddResult AddMultibodyPlant(const MultibodyPlantConfig& config,
                            systems::DiagramBuilder<double>* builder) {
  AddResult result = AddMultibodyPlantSceneGraph(builder, config.time_step);
  ApplyMultibodyPlantConfig(config, &result.plant);
  return result;
}

AddMultibodyPlantSceneGraphResult<double> AddMultibodyPlant(
    const MultibodyPlantConfig& plant_config,
    const geometry::SceneGraphConfig& scene_graph_config,
    systems::DiagramBuilder<double>* builder) {
  AddResult result =
      AddMultibodyPlantSceneGraph(builder, plant_config.time_step);
  ApplyMultibodyPlantConfig(plant_config, &result.plant);
  result.scene_graph.set_config(scene_graph_config);
  return result;
}

void ApplyMultibodyPlantConfig(const MultibodyPlantConfig& config,
                               MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  // TODO(russt): Add MultibodyPlant.set_time_step() and use it here.
  DRAKE_THROW_UNLESS(plant->time_step() == config.time_step);
  if (plant->is_discrete()) {
    plant->SetUseSampledOutputPorts(config.use_sampled_output_ports);
  }
  plant->set_penetration_allowance(config.penetration_allowance);
  plant->set_stiction_tolerance(config.stiction_tolerance);
  plant->set_contact_model(
      internal::GetContactModelFromString(config.contact_model));
  if (plant->is_discrete()) {
    // When empty, use the recommended value.
    plant->set_discrete_contact_approximation(
        internal::GetDiscreteContactApproximationFromString(
            config.discrete_contact_approximation.empty()
                ? MultibodyPlantConfig{}.discrete_contact_approximation
                : config.discrete_contact_approximation));
  }
  plant->set_sap_near_rigid_threshold(config.sap_near_rigid_threshold);
  plant->set_contact_surface_representation(
      internal::GetContactSurfaceRepresentationFromString(
          config.contact_surface_representation));
  plant->set_adjacent_bodies_collision_filters(
      config.adjacent_bodies_collision_filters);
}

namespace internal {
namespace {

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody add a new value to the enum. New values must be listed here
// as well as in the list of kContactModels below.
constexpr const char* EnumToChars(ContactModel enum_value) {
  switch (enum_value) {
    case ContactModel::kPointContactOnly:
      return "point";
    case ContactModel::kHydroelasticsOnly:
      return "hydroelastic";
    case ContactModel::kHydroelasticWithFallback:
      return "hydroelastic_with_fallback";
  }
}

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody adds a new value to the enum. New values must be listed here
// as well as in the list of kDiscreteContactApproximations below.
constexpr const char* EnumToChars(DiscreteContactApproximation enum_value) {
  switch (enum_value) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    case DiscreteContactApproximation::kTamsi:
      return "tamsi";
#pragma GCC diagnostic pop
    case DiscreteContactApproximation::kSap:
      return "sap";
    case DiscreteContactApproximation::kSimilar:
      return "similar";
    case DiscreteContactApproximation::kLagged:
      return "lagged";
  }
}

// Take an alias to limit verbosity, especially in the constexpr boilerplate.
using ContactRep = geometry::HydroelasticContactRepresentation;

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody add a new value to the enum. New values must be listed here
// as well as in the list of (enum, name) pairs kContactReps below.
constexpr const char* EnumToChars(ContactRep enum_value) {
  switch (enum_value) {
    case ContactRep::kTriangle:
      return "triangle";
    case ContactRep::kPolygon:
      return "polygon";
  }
}

template <typename Enum>
struct NamedEnum {
  // An implicit conversion here enables the convenient initializer_list syntax
  // that's used below, so we'll say NOLINTNEXTLINE(runtime/explicit).
  constexpr NamedEnum(Enum value_in)
      : value(value_in), name(EnumToChars(value_in)) {}
  const Enum value;
  const char* const name;
};

constexpr std::array<NamedEnum<ContactModel>, 3> kContactModels{{
    {ContactModel::kPointContactOnly},
    {ContactModel::kHydroelasticsOnly},
    {ContactModel::kHydroelasticWithFallback},
}};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
constexpr std::array<NamedEnum<DiscreteContactApproximation>, 4>
    kDiscreteContactApproximations{{
        {DiscreteContactApproximation::kTamsi},
        {DiscreteContactApproximation::kSap},
        {DiscreteContactApproximation::kSimilar},
        {DiscreteContactApproximation::kLagged},
    }};
#pragma GCC diagnostic pop

constexpr std::array<NamedEnum<ContactRep>, 2> kContactReps{{
    {ContactRep::kTriangle},
    {ContactRep::kPolygon},
}};

}  // namespace

ContactModel GetContactModelFromString(std::string_view contact_model) {
  for (const auto& [value, name] : kContactModels) {
    if (name == contact_model) {
      return value;
    }
  }
  throw std::logic_error(
      fmt::format("Unknown contact_model: '{}'", contact_model));
}

std::string GetStringFromContactModel(ContactModel contact_model) {
  for (const auto& [value, name] : kContactModels) {
    if (value == contact_model) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

DiscreteContactApproximation GetDiscreteContactApproximationFromString(
    std::string_view discrete_contact_approximation) {
  if (discrete_contact_approximation == "tamsi") {
    static const logging::Warn log_once(
        "DRAKE_DEPRECATED: The TAMSI solver is deprecated, and will be removed "
        "from Drake on or after 2026-09-01.");
  }
  for (const auto& [value, name] : kDiscreteContactApproximations) {
    if (name == discrete_contact_approximation) {
      return value;
    }
  }
  throw std::logic_error(
      fmt::format("Unknown discrete_contact_approximation: '{}'",
                  discrete_contact_approximation));
}

std::string GetStringFromDiscreteContactApproximation(
    DiscreteContactApproximation contact_model) {
  for (const auto& [value, name] : kDiscreteContactApproximations) {
    if (value == contact_model) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

ContactRep GetContactSurfaceRepresentationFromString(
    std::string_view contact_representation) {
  for (const auto& [value, name] : kContactReps) {
    if (name == contact_representation) {
      return value;
    }
  }
  throw std::logic_error(
      fmt::format("Unknown hydroelastic contact representation: '{}'",
                  contact_representation));
}

std::string GetStringFromContactSurfaceRepresentation(
    ContactRep contact_representation) {
  for (const auto& [value, name] : kContactReps) {
    if (value == contact_representation) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
