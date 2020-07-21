#pragma once

/** @file
 A collection of data types and functions to help manage defining properties
 for geometries with the proximity role. These functions facilitate properties
 that are _explicitly_ known in Drake's core functionality. These functions in
 no way limit the inclusion of any other additional, arbitrary properties.
 */

#include <optional>
#include <ostream>
#include <string>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/hydroelastic_type.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {

/**
 * @anchor contact_material_utility_functions
 * @name         Contact Material Utility Functions
 * AddContactMaterial() adds contact material properties to the given set of
 * proximity `properties`. Only the parameters that carry values will be added
 * to the given set of `properties`; no default values will be provided.
 * Downstream consumers of the contact materials can optionally provide
 * defaults for missing properties.
 *
 * For legacy and backwards compatibility purposes, two overloads for
 * AddContactMaterial() are provided. One supports all contact material
 * properties **except** `point_stiffness`, and the other includes it.
 * Users are encouraged to use the overload that contains the argument for
 * `point_stiffness`.
 *
 * These functions will throw an error if:
 * - `elastic_modulus` is not positive
 * - `dissipation` is negative
 * - `point_stiffness` is not positive
 * - Any of the contact material properties have already been defined in
 *   `properties`.
 */
///@{
/**
 * @throws std::logic_error if any parameter doesn't satisfy the requirements
 *                          listed in @ref contact_material_utility_functions
 *                          "Contact Material Utility Functions".
 */
void AddContactMaterial(
    const std::optional<double>& elastic_modulus,
    const std::optional<double>& dissipation,
    const std::optional<double>& point_stiffness,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties);

/**
 * @warning Please use the overload of AddContactMaterial() that includes the
 * argument for `point_stiffness` rather than this one.
 */
void AddContactMaterial(
    const std::optional<double>& elastic_modulus,
    const std::optional<double>& dissipation,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties);
///@}

/** Adds properties to the given set of proximity properties sufficient to cause
 the associated geometry to generate a rigid hydroelastic representation.

 @param resolution_hint       If the geometry is to be tessellated, it is the
                              parameter that guides the level of mesh
                              refinement. See @ref MODULE_NOT_WRITTEN_YET. This
                              will be ignored for geometry types that don't
                              require tessellation.
 @param[in,out] properties    The properties will be added to this property set.
 @throws std::logic_error     If `properties` already has properties with the
                              names that this function would need to add.
 @pre 0 < `resolution_hint` < ∞ and `properties` is not nullptr.  */
void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties);

/** Overload, intended for shapes that don't get tessellated in their
 hydroelastic representation (e.g., HalfSpace and Mesh).
 See @ref MODULE_NOT_WRITTEN_YET.  */
void AddRigidHydroelasticProperties(ProximityProperties* properties);

// TODO(SeanCurtis-TRI): Add module that explains resolution hint and reference
//  it in the documentation below.
/** Adds properties to the given set of proximity properties sufficient to cause
 the associated geometry to generate a soft hydroelastic representation. The
 geometry's pressure field will be the function p(e) = Ee, where E is the
 elastic modulus stored in the given `properties`.

 @param resolution_hint       If the geometry is to be tessellated, it is the
                              parameter that guides the level of mesh
                              refinement. This will be ignored for geometry
                              types that don't require tessellation.
 @param[in,out] properties    The properties will be added to this property set.
 @throws std::logic_error     If `properties` already has properties with the
                              names that this function would need to add.
 @pre 0 < `resolution_hint` < ∞, `properties` is not nullptr, and `properties`
      contains a valid elastic modulus value. */
void AddSoftHydroelasticProperties(double resolution_hint,
                                   ProximityProperties* properties);

/** Overload, intended for shapes that don't get tessellated in their
 hydroelastic representation (e.g., HalfSpace).
 See @ref MODULE_NOT_WRITTEN_YET.  */
void AddSoftHydroelasticProperties(ProximityProperties* properties);

/** Soft half spaces are handled as a special case; they do not get tessellated.
 Instead, they are treated as infinite slabs with a finite thickness. This
 variant is required for hydroelastic half spaces.

 @param slab_thickness      The distance from the half space boundary to its
                            rigid core (this helps define the extent field of
                            the half space).
 @param[out] properties     The properties will be added to this property set.
 @throws std::logic_error If `properties` already has properties with the names
                          that this function would need to add.
 @pre 0 < `slab_thickness` < ∞ . */
void AddSoftHydroelasticPropertiesForHalfSpace(double slab_thickness,
                                               ProximityProperties* properties);

//@}

}  // namespace geometry
}  // namespace drake
