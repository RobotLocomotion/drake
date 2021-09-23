#pragma once

#include <string>

#include "drake/common/name_value.h"

namespace anzu {
namespace sim {

/// The set of configurable properties on a MultibodyPlant.
///
/// The field names and defaults here match MultibodyPlant's defaults exactly,
/// with the exception of time_step -- it has no default within MultibodyPlant,
/// since it is passed in as a constructor argument. Here, we choose a nominal
/// value as a reasonably conservative estimate that works in many cases.
struct MultibodyPlantConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(penetration_allowance));
    a->Visit(DRAKE_NVP(stiction_tolerance));
    a->Visit(DRAKE_NVP(contact_model));
  }

  double time_step{0.001};
  double penetration_allowance{0.001};
  double stiction_tolerance{0.001};

  /// Refer to drake::multibody::ContactModel for details.
  /// Valid strings are:
  /// - "point"
  /// - "hydroelastic"
  /// - "hydroelastic_with_fallback"
  std::string contact_model{"point"};
};

}  // namespace sim
}  // namespace anzu
