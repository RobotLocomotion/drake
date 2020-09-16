#pragma once

#include <string>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Finds an optionally model-scoped frame according to
/// `internal::ScopedNameParser::Parse`.
const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name);

/// Required version of `GetScopedFrameByNameMaybe`.
inline const drake::multibody::Frame<double>&
GetScopedFrameByName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  auto* frame = GetScopedFrameByNameMaybe(plant, full_name);
  if (frame == nullptr) {
    throw std::runtime_error("Could not find frame: " + full_name);
  }
  return *frame;
}

struct ScopedName {
  // If empty, implies no scope.
  std::string instance_name;
  std::string name;
};

/// Attempts to find a name using the following scoping rules:
/// - The delimiter "::" may appear zero or more times.
/// - If one more delimiters are present, the full name is split by the *last*
///   delimiter. The provided model instance name must exist.
ScopedName ParseScopedName(const std::string& full_name);

/// Compose a "namespace::name" name from its components.
std::string PrefixName(const std::string& namespace_, const std::string& name);

/// Get the namespace prefix for a given model instance.
std::string GetInstanceScopeName(
    const drake::multibody::MultibodyPlant<double>&,
    drake::multibody::ModelInstanceIndex);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
