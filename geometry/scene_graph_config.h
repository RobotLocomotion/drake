#pragma once

#include <optional>
#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {


// TODO(rpoyner-tri): adjust doc when implementation is ready.
// TODO(rpoyner-tri): ref hydro user quick start doc when available.
/** (FUTURE) These properties will be used as defaults when the geometry as
added via API calls or parsed from model files doesn't say anything more
specific.  @see @ref hug_title, @ref hug_properties,
@ref stribeck_approximation. */
struct DefaultProximityProperties {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(compliance_type));
    a->Visit(DRAKE_NVP(hydroelastic_modulus));
    a->Visit(DRAKE_NVP(resolution_hint));
    a->Visit(DRAKE_NVP(slab_thickness));
    a->Visit(DRAKE_NVP(dynamic_friction));
    a->Visit(DRAKE_NVP(static_friction));
    a->Visit(DRAKE_NVP(hunt_crossley_dissipation));
    a->Visit(DRAKE_NVP(relaxation_time));
    a->Visit(DRAKE_NVP(point_stiffness));
    ValidateOrThrow();
  }
  /** @name Hydroelastic Contact Properties

  These properties affect hydroelastic contact only. For more detail, including
  limits of the numeric parameters, @see
  geometry::AddRigidHydroelasticProperties,
  geometry::AddCompliantHydroelasticProperties,
  geometry::AddCompliantHydroelasticPropertiesForHalfSpace.

  For more context, @see @ref hug_properties. */
  /// @{
  /** There are three valid options for `compliance_type`:
  - "undefined": hydroelastic contact will not be used.
  - "rigid": the default hydroelastic compliance type will be rigid; note that
     rigid-rigid contact is not supported by the hydroelastic contact model,
     but is supported by point contact (multibody::ContactModel::kPoint or
     multibody::ContactModel::kHydroelasticWithFallback).
  - "compliant": the default hydroelastic compliance type will be compliant. */
  std::string compliance_type{"undefined"};

  /** A measure of material stiffness, in units of Pascals. */
  std::optional<double> hydroelastic_modulus{1e7};

  /** Controls how finely primitive geometries are tessellated, units of
  meters. */
  std::optional<double> resolution_hint{0.5};

  /** For a halfspace, the thickness of compliant material to model, in units
  of meters. */
  std::optional<double> slab_thickness{10.0};
  /// @}

  /** @name General Contact Properties

  These properties affect contact in general. For more detail, including limits
  of the numeric parameters, @see geometry::AddContactMaterial,
  multibody::CoulombFriction, @ref mbp_contact_modeling, @ref
  mbp_dissipation_model. */
  /// @{
  /** To be valid, either both friction values must be populated, or
  neither. Friction quantities are unitless. */
  std::optional<double> dynamic_friction{0.5};
  /** @see dynamic_friction. */
  std::optional<double> static_friction{0.5};

  /** Controls energy damping from contact, for contact models *other than*
  multibody::DiscreteContactApproximation::kSap. Units are seconds per
  meter. */
  std::optional<double> hunt_crossley_dissipation;

  /** Controls energy damping from contact, *only for*
  multibody::DiscreteContactApproximation::kSap. Units are seconds. */
  std::optional<double> relaxation_time;
  /// @}

  /** @name Point Contact Properties

  These properties point contact only. For complete descriptions of
  the numeric parameters, @see geometry::AddContactMaterial. */
  /// @{
  /** A measure of material stiffness, in units of Newtons per meter. */
  std::optional<double> point_stiffness;
  /// @}

  /** Throws if the values are inconsistent. */
  void ValidateOrThrow() const;
};

// TODO(rpoyner-tri): document SceneGraph integration when ready.
/** (FUTURE) The set of configurable properties on a SceneGraph. */
struct SceneGraphConfig {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_proximity_properties));
  }

  // TODO(rpoyner-tri): ref hydro user quick start doc when available.
  /** Provides SceneGraph-wide contact material values to use when none have
  been otherwise specified. */
  DefaultProximityProperties default_proximity_properties;

  /** Throws if the values are inconsistent. */
  void ValidateOrThrow() const;
};

}  // namespace geometry
}  // namespace drake
