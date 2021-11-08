#pragma once

#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace multibody {

/// The set of configurable properties on a MultibodyPlant.
///
/// The field names and defaults here match MultibodyPlant's defaults exactly,
/// with the exception of time_step.
struct MultibodyPlantConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(penetration_allowance));
    a->Visit(DRAKE_NVP(stiction_tolerance));
    a->Visit(DRAKE_NVP(contact_model));
  }

  /// Configures the MultibodyPlant::MultibodyPlant() constructor time_step.
  ///
  /// There is no default value for this within MultibodyPlant itself, so here
  /// we choose a nominal value (a discrete system, with a 1ms periodic update)
  /// as a reasonably conservative estimate that works in many cases.
  double time_step{0.001};

  /// Configures the MultibodyPlant::set_penetration_allowance().
  double penetration_allowance{0.001};

  /// Configures the MultibodyPlant::set_stiction_tolerance().
  double stiction_tolerance{0.001};

  /// Configures the MultibodyPlant::set_contact_model().
  /// Refer to drake::multibody::ContactModel for details.
  /// Valid strings are:
  /// - "point"
  /// - "hydroelastic"
  /// - "hydroelastic_with_fallback"
  std::string contact_model{"point"};
};

}  // namespace multibody
}  // namespace drake
