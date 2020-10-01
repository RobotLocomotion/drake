#pragma once

#include <string>

#include "drake/multibody/plant/multibody_plant.h"

/// @file Implements the namespace scoping semantics described in
/// `README_model_directives.md` for multibody plants built from model
/// directives (and possibly future SDFormat files).

namespace drake {
namespace multibody {
namespace parsing {

/// Finds an optionally model-scoped frame, using the naming rules of
/// `ParseScopedName`.
///
/// Returns `nullptr` if the frame is not found, as well as all the error
/// cases of `MultibodyPlant::HasFrameByName(std::string)`.
const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name);

/// Equivalent to `GetScopedFrameByNameMaybe`, but throws if the frame
/// is not found.
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

/// Constructs and returns a scoped frame name for the requested frame.
std::string GetScopedFrameName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame);

/// Convenience class for a scoped name.
struct ScopedName {
  /// The name of the multibody instance part of a scoped name.  If empty,
  /// implies no multibody instance scope.
  std::string instance_name;

  /// The model-instance-specific part of the name, ie the name of the frame,
  /// body, etc. within the instance.
  std::string name;
};

/// Attempts to find a name using the following scoping rules:
/// - The delimiter "::" may appear zero or more times.
/// - If one more delimiters are present, the full name is split by the *last*
///   delimiter. The provided model instance name must exist.
ScopedName ParseScopedName(const std::string& full_name);

/// Composes a "namespace::name" name from its components.
std::string PrefixName(const std::string& namespace_, const std::string& name);

/// Gets the namespace prefix for a given model instance.
std::string GetInstanceScopeName(
    const drake::multibody::MultibodyPlant<double>&,
    drake::multibody::ModelInstanceIndex);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
