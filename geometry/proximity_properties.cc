#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {

const char* const kMaterialGroup = "material";
const char* const kFriction = "coulomb_friction";
const char* const kHcDissipation = "hunt_crossley_dissipation";
const char* const kPointStiffness = "point_contact_stiffness";

const char* const kHydroGroup = "hydroelastic";
const char* const kElastic = "hydroelastic_modulus";
const char* const kRezHint = "resolution_hint";
const char* const kComplianceType = "compliance_type";
const char* const kSlabThickness = "slab_thickness";

std::ostream& operator<<(std::ostream& out, const HydroelasticType& type) {
  switch (type) {
    case HydroelasticType::kUndefined:
      out << "undefined";
      break;
    case HydroelasticType::kRigid:
      out << "rigid";
      break;
    case HydroelasticType::kSoft:
      out << "soft";
      break;
    default:
      DRAKE_UNREACHABLE();
  }
  return out;
}

}  // namespace internal

void AddContactMaterial(
    const std::optional<double>& dissipation,
    const std::optional<double>& point_stiffness,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  if (dissipation.has_value()) {
    if (*dissipation < 0) {
      throw std::logic_error(fmt::format(
          "The dissipation can't be negative; given {}", *dissipation));
    }
    properties->AddProperty(internal::kMaterialGroup, internal::kHcDissipation,
                            *dissipation);
  }

  if (point_stiffness.has_value()) {
    if (*point_stiffness <= 0) {
      throw std::logic_error(fmt::format(
          "The point_contact_stiffness must be strictly positive; given {}",
          *point_stiffness));
    }
    properties->AddProperty(internal::kMaterialGroup, internal::kPointStiffness,
                            *point_stiffness);
  }

  if (friction.has_value()) {
    properties->AddProperty(internal::kMaterialGroup, internal::kFriction,
                            *friction);
  }
}

// NOTE: Although these functions currently do the same thing, we're leaving
// the two functions in place to facilitate future differences.

void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  properties->AddProperty(internal::kHydroGroup, internal::kRezHint,
                          resolution_hint);
  AddRigidHydroelasticProperties(properties);
}

void AddRigidHydroelasticProperties(ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  // The bare minimum of defining a rigid geometry is to declare its compliance
  // type. Downstream consumers (ProximityEngine) will determine if this is
  // sufficient.
  properties->AddProperty(internal::kHydroGroup, internal::kComplianceType,
                          internal::HydroelasticType::kRigid);
}

namespace {
void AddSoftHydroelasticProperties(double hydroelastic_modulus,
                                   ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  // The bare minimum of defining a compliant geometry is to declare its
  // compliance type. Downstream consumers (ProximityEngine) will determine
  // if this is sufficient.
  if (hydroelastic_modulus <= 0) {
    throw std::logic_error(
        fmt::format("The hydroelastic modulus must be positive; given {}",
                    hydroelastic_modulus));
  }
  properties->AddProperty(internal::kHydroGroup, internal::kElastic,
                          hydroelastic_modulus);
  properties->AddProperty(internal::kHydroGroup, internal::kComplianceType,
                          internal::HydroelasticType::kSoft);
}
}  // namespace

void AddSoftHydroelasticProperties(double resolution_hint,
                                   double hydroelastic_modulus,
                                   ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  properties->AddProperty(internal::kHydroGroup, internal::kRezHint,
                          resolution_hint);
  AddSoftHydroelasticProperties(hydroelastic_modulus, properties);
}

void AddSoftHydroelasticPropertiesForHalfSpace(
    double slab_thickness, double hydroelastic_modulus,
    ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  properties->AddProperty(internal::kHydroGroup, internal::kSlabThickness,
                          slab_thickness);
  AddSoftHydroelasticProperties(hydroelastic_modulus, properties);
}

}  // namespace geometry
}  // namespace drake
