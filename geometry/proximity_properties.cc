#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {

}  // namespace internal

using internal::PropName;

void AddContactMaterial(
    const std::optional<double>& elastic_modulus,
    const std::optional<double>& dissipation,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties) {
  AddContactMaterial(elastic_modulus, dissipation, {}, friction, properties);
}

void AddContactMaterial(
    const std::optional<double>& elastic_modulus,
    const std::optional<double>& dissipation,
    const std::optional<double>& point_stiffness,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties) {

  if (elastic_modulus.has_value()) {
    if (*elastic_modulus <= 0) {
      throw std::logic_error(fmt::format(
          "The elastic modulus must be positive; given {}", *elastic_modulus));
    }
    properties->Add(properties->material_elastic_modulus(), *elastic_modulus);
  }
  if (dissipation.has_value()) {
    if (*dissipation < 0) {
      throw std::logic_error(fmt::format(
          "The dissipation can't be negative; given {}", *dissipation));
    }
    properties->Add(properties->material_hunt_crossley_dissipation(),
                    *dissipation);
  }

  if (point_stiffness.has_value()) {
    if (*point_stiffness <= 0) {
      throw std::logic_error(fmt::format(
          "The point_contact_stiffness must be strictly positive; given {}",
          *point_stiffness));
    }
    properties->Add(properties->material_point_contact_stiffness(),
                    *point_stiffness);
  }

  if (friction.has_value()) {
    properties->Add(properties->material_coulomb_friction(), *friction);
  }
}

// NOTE: Although these functions currently do the same thing, we're leaving
// the two functions in place to facilitate future differences.

void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  properties->Add(properties->hydroelastic_resolution_hint(), resolution_hint);
  AddRigidHydroelasticProperties(properties);
}

void AddRigidHydroelasticProperties(ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  // The bare minimum of defining a rigid geometry is to declare its compliance
  // type. Downstream consumers (ProximityEngine) will determine if this is
  // sufficient.
  properties->Add(properties->hydroelastic_compliance_type(),
                  internal::HydroelasticType::kRigid);
}

void AddSoftHydroelasticProperties(double resolution_hint,
                                   ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  properties->Add(properties->hydroelastic_resolution_hint(), resolution_hint);
  AddSoftHydroelasticProperties(properties);
}

void AddSoftHydroelasticProperties(ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  // The bare minimum of defining a soft geometry is to declare its compliance
  // type. Downstream consumers (ProximityEngine) will determine if this is
  // sufficient.
  properties->Add(properties->hydroelastic_compliance_type(),
                  internal::HydroelasticType::kSoft);
}

void AddSoftHydroelasticPropertiesForHalfSpace(
    double slab_thickness, ProximityProperties* properties) {
  DRAKE_DEMAND(properties);
  properties->Add(properties->hydroelastic_slab_thickness(), slab_thickness);
  AddSoftHydroelasticProperties(properties);
}

}  // namespace geometry
}  // namespace drake
