#pragma once

/** @file
 A collection of data types and functions to help manage defining properties
 for geometries with the proximity role. These functions facilitate properties
 that are _explicitly_ known in Drake's core functionality. These functions in
 no way limit the inclusion of any other additional, arbitrary properties.
 */

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {

/** @name  Declaring general contact material properties

 String constants used to access the contact material properties that
 SceneGraph depends on. These are not the exhaustive set of contact material
 properties that downstream consumers (e.g., MultibodyPlant) may require.

 <!-- TODO(SeanCurtis-TRI): Extend this to include other contact material
 properties and an API for setting them more conveniently.  */
//@{

extern const char* const kMaterialGroup;  ///< The contact material group name.
extern const char* const kElastic;        ///< Elastic modulus property name.

//@}

/** @name  Declaring geometry for hydroelastic contact.

 In order for a geometry to be used in hydroelastic contact, it must be declared
 as such. The declaration consists of setting a number of properties in the
 geometry's associated ProximityProperties.

 These utilities include:

   - string constants to read and write the indicated properties, and
   - utility functions for declaring consistent hydroelastic properties
     including
       - differentiating between a rigid and soft geometry

 @todo Add reference to discussion of hydroelastic proximity properties along
 the lines of "For the full discussion of preparing geometry for use in the
 hydroelastic contact model, see `@ref MODULE_NOT_WRITTEN_YET`.
 */
//@{

extern const char* const kHydroGroup;      ///< Hydroelastic group name.
extern const char* const kRezHint;         ///< Resolution hint property name.

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

//@}

}  // namespace geometry
}  // namespace drake
