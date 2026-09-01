#include "drake/geometry/proximity_properties.h"

#include <array>
#include <cmath>
#include <string>

namespace drake {
namespace geometry {
namespace internal {

const char* const kMaterialGroup = "material";
const char* const kFriction = "coulomb_friction";
const char* const kHcDissipation = "hunt_crossley_dissipation";
const char* const kRelaxationTime = "relaxation_time";
const char* const kPointStiffness = "point_contact_stiffness";

const char* const kHydroGroup = "hydroelastic";
const char* const kElastic = "hydroelastic_modulus";
const char* const kRezHint = "resolution_hint";
const char* const kComplianceType = "compliance_type";
const char* const kSlabThickness = "slab_thickness";
const char* const kMargin = "margin";

namespace {

// Use a switch() statement here, to ensure the compiler sends us a reminder
// when somebody adds a new value to the enum. New values must be listed here
// as well as in the list of kHydroelasticTypes below.
constexpr const char* EnumToChars(HydroelasticType enum_value) {
  switch (enum_value) {
    case HydroelasticType::kUndefined:
      return "undefined";
    case HydroelasticType::kRigid:
      return "rigid";
    case HydroelasticType::kCompliant:
      return "compliant";
  }
  DRAKE_UNREACHABLE();
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

constexpr std::array<NamedEnum<HydroelasticType>, 3> kHydroelasticTypes{{
    {HydroelasticType::kUndefined},
    {HydroelasticType::kRigid},
    {HydroelasticType::kCompliant},
}};

}  // namespace

HydroelasticType GetHydroelasticTypeFromString(
    std::string_view hydroelastic_type) {
  for (const auto& [value, name] : kHydroelasticTypes) {
    if (name == hydroelastic_type) {
      return value;
    }
  }
  throw std::logic_error(
      fmt::format("Unknown hydroelastic_type: '{}'", hydroelastic_type));
}

std::string GetStringFromHydroelasticType(HydroelasticType hydroelastic_type) {
  for (const auto& [value, name] : kHydroelasticTypes) {
    if (value == hydroelastic_type) {
      return name;
    }
  }
  DRAKE_UNREACHABLE();
}

std::string_view to_string(const HydroelasticType& type) {
  return EnumToChars(type);
}

void ThrowIfInvalidHydroelasticModulus(double hydroelastic_modulus) {
  // +∞ is allowed (equivalent to rigid); NaN is not (`!(nan > 0)`).
  if (!(hydroelastic_modulus > 0)) {
    throw std::logic_error(
        fmt::format("The hydroelastic modulus must be positive; given {}",
                    hydroelastic_modulus));
  }
}

void ThrowIfInvalidResolutionHint(double resolution_hint) {
  if (!(std::isfinite(resolution_hint) && resolution_hint > 0)) {
    throw std::logic_error(fmt::format(
        "The resolution_hint must be positive and finite; given {}",
        resolution_hint));
  }
}

void ThrowIfInvalidSlabThickness(double slab_thickness) {
  if (!(std::isfinite(slab_thickness) && slab_thickness > 0)) {
    throw std::logic_error(fmt::format(
        "The slab_thickness must be positive and finite; given {}",
        slab_thickness));
  }
}

void ThrowIfInvalidMargin(double margin) {
  if (!(std::isfinite(margin) && margin >= 0)) {
    throw std::logic_error(fmt::format(
        "The margin must be non-negative and finite; given {}", margin));
  }
}

void ThrowIfInvalidHuntCrossleyDissipation(double dissipation) {
  // +∞ is allowed; NaN is not (`!(nan >= 0)`).
  if (!(dissipation >= 0)) {
    throw std::logic_error(fmt::format(
        "The dissipation can't be negative; given {}", dissipation));
  }
}

void ThrowIfInvalidRelaxationTime(double relaxation_time) {
  if (!(std::isfinite(relaxation_time) && relaxation_time >= 0)) {
    throw std::logic_error(fmt::format(
        "The relaxation_time must be non-negative and finite; given {}",
        relaxation_time));
  }
}

void ThrowIfInvalidPointStiffness(double point_stiffness) {
  // +∞ is allowed; NaN is not (`!(nan > 0)`).
  if (!(point_stiffness > 0)) {
    throw std::logic_error(fmt::format(
        "The point_contact_stiffness must be strictly positive; given {}",
        point_stiffness));
  }
}

void ThrowIfInvalidFrictionCoefficient(double friction_coefficient) {
  // +∞ is allowed; NaN is not (`!(nan >= 0)`).
  if (!(friction_coefficient >= 0)) {
    throw std::logic_error(fmt::format(
        "The friction coefficient can't be negative; given {}",
        friction_coefficient));
  }
}

}  // namespace internal

void AddContactMaterial(
    std::optional<double> dissipation, std::optional<double> point_stiffness,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  if (dissipation.has_value()) {
    internal::ThrowIfInvalidHuntCrossleyDissipation(*dissipation);
    properties->AddProperty(internal::kMaterialGroup, internal::kHcDissipation,
                            *dissipation);
  }

  if (point_stiffness.has_value()) {
    internal::ThrowIfInvalidPointStiffness(*point_stiffness);
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
  internal::ThrowIfInvalidResolutionHint(resolution_hint);
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
void AddCompliantHydroelasticProperties(double hydroelastic_modulus,
                                        ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  // The bare minimum of defining a compliant geometry is to declare its
  // compliance type. Downstream consumers (ProximityEngine) will determine
  // if this is sufficient.
  internal::ThrowIfInvalidHydroelasticModulus(hydroelastic_modulus);
  properties->AddProperty(internal::kHydroGroup, internal::kElastic,
                          hydroelastic_modulus);
  properties->AddProperty(internal::kHydroGroup, internal::kComplianceType,
                          internal::HydroelasticType::kCompliant);
}
}  // namespace

void AddCompliantHydroelasticProperties(double resolution_hint,
                                        double hydroelastic_modulus,
                                        ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  internal::ThrowIfInvalidResolutionHint(resolution_hint);
  properties->AddProperty(internal::kHydroGroup, internal::kRezHint,
                          resolution_hint);
  AddCompliantHydroelasticProperties(hydroelastic_modulus, properties);
}

void AddCompliantHydroelasticPropertiesForHalfSpace(
    double slab_thickness, double hydroelastic_modulus,
    ProximityProperties* properties) {
  DRAKE_DEMAND(properties != nullptr);
  internal::ThrowIfInvalidSlabThickness(slab_thickness);
  properties->AddProperty(internal::kHydroGroup, internal::kSlabThickness,
                          slab_thickness);
  AddCompliantHydroelasticProperties(hydroelastic_modulus, properties);
}

}  // namespace geometry
}  // namespace drake
