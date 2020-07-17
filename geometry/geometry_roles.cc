#include "drake/geometry/geometry_roles.h"

#include <string>

#include "drake/geometry/proximity/hydroelastic_type.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {

std::string to_string(const Role& role) {
  switch (role) {
    case Role::kProximity:
      return "proximity";
    case Role::kPerception:
      return "perception";
    case Role::kIllustration:
      return "illustration";
    case Role::kUnassigned:
      return "unassigned";
  }
  return "unknown";
}

std::ostream& operator<<(std::ostream& out, const Role& role) {
  out << to_string(role);
  return out;
}

IllustrationProperties MakePhongIllustrationProperties(
    const Vector4<double>& diffuse) {
  IllustrationProperties props;
  props.Add("phong/diffuse", diffuse);
  return props;
}

namespace {

// Various functions to use with DoThrowIfInvalid.
bool is_positive_double(const double& value) { return value > 0.0; }

bool is_nonnegative_double(const double& value) { return value >= 0.0; }

template <typename ValueType>
bool always_true(const ValueType&) {
  return true;
}

bool has_defined_compliance(const internal::HydroelasticType& value) {
  return value != internal::HydroelasticType::kUndefined;
}

}  // namespace

void ProximityProperties::DoThrowIfInvalid(const PropertyName& property,
                                           const AbstractValue& value) const {
  if (property == material_elastic_modulus()) {
    ValidateOrThrow<double>(property, value, &is_positive_double,
                            "value must be > 0");
  } else if (property == material_coulomb_friction()) {
    ValidateOrThrow<multibody::CoulombFriction<double>>(
        property, value, always_true<multibody::CoulombFriction<double>>,
        "any valid friction");
  } else if (property == material_hunt_crossley_dissipation()) {
    ValidateOrThrow<double>(property, value, is_nonnegative_double,
                            "value must be >= 0");
  } else if (property == material_point_contact_stiffness()) {
    ValidateOrThrow<double>(property, value, is_positive_double,
                    "value must be > 0");
  } else if (property == hydroelastic_resolution_hint()) {
    ValidateOrThrow<double>(property, value, is_positive_double,
                            "value must be > 0");
  } else if (property == hydroelastic_slab_thickness()) {
    ValidateOrThrow<double>(property, value, is_positive_double,
                            "value must be > 0");
  } else if (property == hydroelastic_compliance_type()) {
    ValidateOrThrow<internal::HydroelasticType>(
        property, value, has_defined_compliance,
        "compliance can't be undefined");
  } else if (property == hydrolastic_tessellation_strategy()) {
    ValidateOrThrow<internal::TessellationStrategy>(
        property, value, always_true<internal::TessellationStrategy>,
        "any valid strategy");
  }
}

}  // namespace geometry
}  // namespace drake
