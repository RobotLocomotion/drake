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
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(penetration_allowance));
    a->Visit(DRAKE_NVP(stiction_tolerance));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(discrete_contact_solver));
    a->Visit(DRAKE_NVP(contact_surface_representation));
    a->Visit(DRAKE_NVP(adjacent_bodies_collision_filters));
    a->Visit(DRAKE_NVP(default_floating_joint_type));
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
  std::string contact_model{"hydroelastic_with_fallback"};

  /// Configures the MultibodyPlant::set_discrete_contact_solver().
  /// Refer to drake::multibody::DiscreteContactSolver for details.
  /// Valid strings are:
  /// - "tamsi"
  /// - "sap"
  std::string discrete_contact_solver{"tamsi"};

  /// Configures the MultibodyPlant::set_contact_surface_representation().
  /// Refer to drake::geometry::HydroelasticContactRepresentation for details.
  /// Valid strings are:
  /// - "triangle"
  /// - "polygon"
  ///
  /// The default value used here is consistent with the default time_step
  /// chosen above; keep this consistent with
  /// MultibodyPlant::GetDefaultContactSurfaceRepresentation().
  std::string contact_surface_representation{"polygon"};

  /// Configures the MultibodyPlant::set_adjacent_bodies_collision_filters().
  bool adjacent_bodies_collision_filters{true};

  /// The default joint type for free bodies.
  /// Valid strings are:
  ///  - quaternion_floating
  ///  - space_xyz_floating
  /// The space_xyz convention refers to extrinsic Euler angles, also known as
  /// roll-pitch-yaw (or rpy) by roboticists.
  std::string default_floating_joint_type{"quaternion_floating"};
};

}  // namespace multibody
}  // namespace drake
