#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {

DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name);

DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
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

DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
std::string GetScopedFrameName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame);

struct DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
ScopedName {
  std::string instance_name;
  std::string name;
};

DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
ScopedName ParseScopedName(const std::string&);

DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
std::string PrefixName(const std::string& namespace_, const std::string& name);

DRAKE_DEPRECATED("2022-10-01", "Use multibody::ScopedName instead")
std::string GetInstanceScopeName(
    const drake::multibody::MultibodyPlant<double>&,
    drake::multibody::ModelInstanceIndex);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
