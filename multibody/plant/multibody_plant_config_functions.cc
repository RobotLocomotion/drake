#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include <array>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

using AddResult = AddMultibodyPlantSceneGraphResult<double>;

AddResult AddMultibodyPlant(const MultibodyPlantConfig& config,
                            systems::DiagramBuilder<double>* builder) {
  AddResult result = AddMultibodyPlantSceneGraph(builder, config.time_step);
  // Sanity check that MultibodyPlant defaults match the MultibodyPlantConfig
  // defaults.
  if (result.plant.is_discrete()) {
    DRAKE_DEMAND(result.plant.get_discrete_contact_approximation() ==
                 DiscreteContactApproximation::kTamsi);
    DRAKE_DEMAND(result.plant.get_discrete_contact_solver() ==
                 DiscreteContactSolver::kTamsi);
  }

  ApplyMultibodyPlantConfig(config, &result.plant);
  return result;
}

void ApplyMultibodyPlantConfig(const MultibodyPlantConfig& config,
                               MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  // TODO(russt): Add MultibodyPlant.set_time_step() and use it here.
  DRAKE_THROW_UNLESS(plant->time_step() == config.time_step);
  plant->set_penetration_allowance(config.penetration_allowance);
  plant->set_stiction_tolerance(config.stiction_tolerance);
  plant->set_contact_model(
      internal::GetContactModelFromString(config.contact_model));
  // Only one of these can be non-empty at a time.
  if (!config.discrete_contact_solver.empty() &&
      !config.discrete_contact_approximation.empty()) {
    throw std::logic_error(
        "In a MultibodyPlantConfig, only one of discrete_contact_solver and "
        "discrete_contact_approximation can be non-empty at a time.");
  }
  if (plant->is_discrete()) {
    if (!config.discrete_contact_solver.empty()) {
      // discrete_contact_approximation is empty, therefore
      // discrete_contact_solver determines both model approximation and solver.
      static const drake::internal::WarnDeprecated warn_once(
          "2024-04-01",
          "Use MultibodyPlantConfig::discrete_contact_approximation instead of "
          "MultibodyPlantConfig::discrete_contact_solver.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      // Until its removal, discrete_contact_solver has precedence over
      // discrete_contact_approximation.
      plant->set_discrete_contact_solver(
          internal::GetDiscreteContactSolverFromString(
              config.discrete_contact_solver));
#pragma GCC diagnostic pop
    } else if (!config.discrete_contact_approximation.empty()) {
      // discrete_contact_solver is empty, therefore
      // discrete_contact_approximation determines both model approximation and
      // solver.
      plant->set_discrete_contact_approximation(
          internal::GetDiscreteContactApproximationFromString(
              config.discrete_contact_approximation));
    } else {
      // Both discrete_contact_approximation and discrete_contact_solver are
      // empty. Default to TAMSI.
      plant->set_discrete_contact_approximation(
          DiscreteContactApproximation::kTamsi);
    }
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
// as well as in the list of kDiscreteContactSolvers below.
constexpr const char* EnumToChars(DiscreteContactSolver enum_value) {
  switch (enum_value) {
    case DiscreteContactSolver::kTamsi:
      return "tamsi";
    case DiscreteContactSolver::kSap:
      return "sap";
  }
}

constexpr const char* EnumToChars(DiscreteContactApproximation enum_value) {
  switch (enum_value) {
    case DiscreteContactApproximation::kTamsi:
      return "tamsi";
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

constexpr std::array<NamedEnum<DiscreteContactSolver>, 2> kContactSolvers{{
    {DiscreteContactSolver::kTamsi},
    {DiscreteContactSolver::kSap},
}};

constexpr std::array<NamedEnum<DiscreteContactApproximation>, 4>
    kDiscreteContactApproximations{{
        {DiscreteContactApproximation::kTamsi},
        {DiscreteContactApproximation::kSap},
        {DiscreteContactApproximation::kSimilar},
        {DiscreteContactApproximation::kLagged},
    }};

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

DiscreteContactSolver GetDiscreteContactSolverFromString(
    std::string_view discrete_contact_solver) {
  for (const auto& [value, name] : kContactSolvers) {
    if (name == discrete_contact_solver) {
      return value;
    }
  }
  throw std::logic_error(fmt::format("Unknown discrete_contact_solver: '{}'",
                                     discrete_contact_solver));
}

std::string GetStringFromDiscreteContactSolver(
    DiscreteContactSolver contact_solver) {
  for (const auto& [value, name] : kContactSolvers) {
    if (value == contact_solver) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

DiscreteContactApproximation GetDiscreteContactApproximationFromString(
    std::string_view discrete_contact_approximation) {
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
