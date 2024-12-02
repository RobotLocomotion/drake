#pragma once

#include <optional>
#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/** These properties will be used as defaults when the geometry as
added via API calls or parsed from model files doesn't say anything more
specific.  @see @ref compliant_contact, @ref hydroelastic_user_guide,
@ref friction_model and subsections therein. */
struct DefaultProximityProperties {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(compliance_type));
    a->Visit(DRAKE_NVP(hydroelastic_modulus));
    a->Visit(DRAKE_NVP(resolution_hint));
    a->Visit(DRAKE_NVP(slab_thickness));
    a->Visit(DRAKE_NVP(margin));
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
  meters.

  While no single value is universally appropriate, this value was selected
  based on the following idea. We're attempting to make introducing novel
  manipulands as easy as possible. Considering a simple soup can as a
  representative manipuland, we've picked a value that would result in a
  tessellated cylinder with enough faces to be appropriate for contact with
  a compliant gripper. */
  std::optional<double> resolution_hint{0.02};

  /** For a halfspace, the thickness of compliant material to model, in units
  of meters. */
  std::optional<double> slab_thickness;

  /** (Advanced) Specifies a thin layer of thickness "margin" (in meters) around
  each geometry. Two bodies with margins δ₁ and δ₂ are considered for contact
  resolution whenever their distance is within δ₁ + δ₂. That is, (speculative)
  contact constraints are added for objects at a distance smaller than δ₁+δ₂.

  Refer to @ref hydro_margin for further details, including theory, examples,
  recommended margin values and limitations.

  There will only be _contact_ if the two zero level sets intersect. Unless the
  zero level sets of both geometries overlap, there is no contact and no
  contact force. However, (speculative) contact constraints will be added
  allowing our discrete contact solvers to predict if a contact "will" occur at
  the next time step. This leads to additional time coherence and stability.
  Analytical studies of stability show that a value of 0.1 mm to 1.0 mm is more
  than enough for most robotics applications. This is not an
  "action-at-a-distance" trick, there is no contact when the thin margin layers
  of two objects overlap. The margin is simply a "cheap" mechanism to avoid
  significantly more complex and costly strategies such as Continuous Collision
  Detection.

  @note Inflating the geometries does not appreciably change the domain of
  contact. In the original mesh, the contact pressure is zero on the mesh's
  boundary surface. When inflating the meshes, we redefine the pressure field
  so that its zero level set field is a good approximation to the surface of
  the original geometry. When visualizing hydroelastic proximity geometry, the
  rendered geometry will include the inflation.

  @note Currently margin only applies to _compliant_ hydroelastic contact and
  it does not affect point contact. */
  std::optional<double> margin;
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

  /** Controls energy dissipation from contact, for contact approximations
  *other than* multibody::DiscreteContactApproximation::kSap. Units are seconds
  per meter.

  If a non-deformable geometry is missing a value for dissipation,
  MultibodyPlant will generate a default value (based on
  multibody::MultibodyPlantConfig::penetration_allowance). However, this
  behavior will be going away. Therefore, we recommend guaranteeing that every
  geometry has a dissipation value either by assigning the property directly to
  the geometry or by providing a non-null value here.

  Please refer to @ref contact_defaults "Default Contact Parameters" for more
  details on Drake's defaults along with guidelines on how to estimate
  parameters specific to your model. */
  std::optional<double> hunt_crossley_dissipation{50.0};

  /** Controls energy damping from contact, *only for*
  multibody::DiscreteContactApproximation::kSap. Units are seconds. */
  std::optional<double> relaxation_time{0.1};
  /// @}

  /** @name Point Contact Properties

  These properties affect point contact only. For complete descriptions of
  the numeric parameters, See
  @ref point_forces_modeling "Compliant Point Contact Forces",
  geometry::AddContactMaterial. */
  /// @{

  /** A measure of material stiffness, in units of Newtons per meter.

  If a non-deformable geometry is missing a value for stiffness,
  MultibodyPlant will generate a default value (based on
  multibody::MultibodyPlantConfig::penetration_allowance). However, this
  behavior will be going away. Therefore, we recommend guaranteeing that every
  geometry has a stiffness value either by assigning the property directly to
  the geometry or by providing a non-null value here.

  Please refer to @ref contact_defaults "Default Contact Parameters" for more
  details on Drake's defaults along with guidelines on how to estimate
  parameters specific to your model. */
  std::optional<double> point_stiffness{1e6};
  /// @}

  /** Throws if the values are inconsistent. */
  void ValidateOrThrow() const;
};

/** The set of configurable properties on a SceneGraph. */
struct SceneGraphConfig {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_proximity_properties));
  }

  /** Provides SceneGraph-wide contact material values to use when none have
  been otherwise specified. */
  DefaultProximityProperties default_proximity_properties;

  /** Throws if the values are inconsistent. */
  void ValidateOrThrow() const;
};

}  // namespace geometry
}  // namespace drake
