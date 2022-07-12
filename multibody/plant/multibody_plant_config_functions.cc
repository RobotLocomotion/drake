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
  result.plant.set_discrete_contact_solver(
      internal::GetDiscreteContactSolverFromString(
          config.discrete_contact_solver));
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

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody adds a new value to the enum. New values must be listed here
// as well as in the list of kDiscreteContactSolvers below.
constexpr const char* DiscreteContactSolverToChars(
    DiscreteContactSolver type) {
  switch (type) {
    case DiscreteContactSolver::kTamsi:
      return "tamsi";
    case DiscreteContactSolver::kSap:
      return "sap";
  }
}

constexpr auto MakeDiscreteContactSolverPair(
    DiscreteContactSolver value) {
  return std::pair(value, DiscreteContactSolverToChars(value));
}

constexpr std::array<std::pair<DiscreteContactSolver, const char*>, 2>
    kDiscreteContactSolvers{{
        MakeDiscreteContactSolverPair(DiscreteContactSolver::kTamsi),
        MakeDiscreteContactSolverPair(DiscreteContactSolver::kSap),
    }};

// Take an alias to limit verbosity, especially in the constexpr boilerplate.
using ContactRep = geometry::HydroelasticContactRepresentation;

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody add a new value to the enum. New values must be listed here
// as well as in the list of (enum, name) pairs kContactReps below.
constexpr const char* ContactRepToChars(ContactRep contact_model) {
  switch (contact_model) {
    case ContactRep::kTriangle:
      return "triangle";
    case ContactRep::kPolygon:
      return "polygon";
  }
}

constexpr auto MakeContactRepPair(ContactRep value) {
  return std::pair(value, ContactRepToChars(value));
}

constexpr std::array<std::pair<ContactRep, const char*>, 2> kContactReps{{
  MakeContactRepPair(ContactRep::kTriangle),
  MakeContactRepPair(ContactRep::kPolygon),
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

DiscreteContactSolver GetDiscreteContactSolverFromString(
    std::string_view discrete_contact_solver) {
  for (const auto& [value, name] : kDiscreteContactSolvers) {
    if (name == discrete_contact_solver) {
      return value;
    }
  }
  throw std::logic_error(
      fmt::format("Unknown discrete_contact_solver: '{}'",
                  discrete_contact_solver));
}

std::string GetStringFromDiscreteContactSolver(
    DiscreteContactSolver contact_solver) {
  for (const auto& [value, name] : kDiscreteContactSolvers) {
    if (value == contact_solver) {
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
  throw std::logic_error(fmt::format(
      "Unknown hydroelastic contact representation: '{}'",
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
