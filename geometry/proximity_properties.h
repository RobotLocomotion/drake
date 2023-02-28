#pragma once

/** @file
 A collection of data types and functions to help manage defining properties
 for geometries with the proximity role. These functions facilitate properties
 that are _explicitly_ known in Drake's core functionality. These functions in
 no way limit the inclusion of any other additional, arbitrary properties.
 */

#include <optional>
#include <ostream>

#include "drake/common/fmt_ostream.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {
namespace internal {

/* @name  Declaring general contact material properties

 String constants used to access the contact material properties that
 SceneGraph depends on. These are not the exhaustive set of contact material
 properties that downstream consumers (e.g., MultibodyPlant) may require.

 As the namespace indicates, these are for internal use only, so Drake entities
 can implicitly coordinate the values they use to define proximity properties.
 These strings don't suggest what constitutes a valid property *value*. For
 those definitions, one should refer to the consumer of the properties (as
 called out in the documentation of the ProximityProperties class).  */
//@{

extern const char* const kMaterialGroup;   ///< The contact material group name.
extern const char* const kFriction;        ///< Friction coefficients property
                                           ///< name.
extern const char* const kHcDissipation;   ///< Hunt-Crossley dissipation
                                           ///< property name.
extern const char* const kRelaxationTime;  ///< Linear dissipation
                                           ///< property name.
extern const char* const kPointStiffness;  ///< Point stiffness property
                                           ///< name.

//@}

/* @name  Declaring geometry for hydroelastic contact.

 In order for a geometry to be used in hydroelastic contact, it must be declared
 as such. The declaration consists of setting a number of properties in the
 geometry's associated ProximityProperties.

 These utilities include:

   - string constants to read and write the indicated properties, and
   - utility functions for declaring consistent hydroelastic properties
     including
       - differentiating between a rigid and compliant geometry
       - accounting for differences between tessellated meshes and half spaces.

 For the full discussion of preparing geometry for use in the hydroelastic
 contact model, see @ref creating_hydro_reps.
 */
//@{

extern const char* const kHydroGroup;       ///< Hydroelastic group name.
extern const char* const kElastic;          ///< Hydroelastic modulus property
                                            ///< name.
extern const char* const kRezHint;          ///< Resolution hint property name.
extern const char* const kComplianceType;   ///< Compliance type property name.
extern const char* const kSlabThickness;    ///< Slab thickness property name
                                            ///< (for half spaces).

//@}

// TODO(SeanCurtis-TRI): Update this to have an additional classification: kBoth
//  when we have the need from the algorithm. For example: when we have two
//  very stiff objects, we'd want to process them as compliant. But when one
//  very stiff and one very compliant object interact, it might make sense to
//  consider the stiff object as effectively rigid and simplify the computation.
//  In this case, the object would get two representations.
/* Classification of the type of representation a shape has for the
 hydroelastic contact model: rigid or soft.  */
enum class HydroelasticType {
  kUndefined,
  kRigid,
  kSoft
};

/* Streaming operator for writing hydroelastic type to output stream.  */
std::ostream& operator<<(std::ostream& out, const HydroelasticType& type);

}  // namespace internal

/**
 * AddContactMaterial() adds general contact material properties to the given
 * set of proximity `properties`. These are the properties required by the
 * default point contact model. However, other contact models can opt to use
 * these properties as well. Only the parameters that carry values will be
 * added to the given set of `properties`; no default values will be provided.
 * Downstream consumers of the contact materials can optionally provide
 * defaults for missing properties.
 *
 * @throws std::exception if `dissipation` is negative, `point_stiffness` is
 * not positive, of any of the contact material properties have already been
 * defined in ``properties`.
 * @pre `properties` is not nullptr.
 */
void AddContactMaterial(
    const std::optional<double>& dissipation,
    const std::optional<double>& point_stiffness,
    const std::optional<multibody::CoulombFriction<double>>& friction,
    ProximityProperties* properties);

/** Adds properties to the given set of proximity properties sufficient to cause
 the associated geometry to generate a rigid hydroelastic representation.

 @param resolution_hint       If the geometry is to be tessellated, it is the
                              parameter that guides the level of mesh
                              refinement. It has length units (in meters) and
                              roughly corresponds to a typical edge length in
                              the resulting mesh.  See @ref hug_properties.
                              This will be ignored for geometry types that don't
                              require tessellation.
 @param[in,out] properties    The properties will be added to this property set.
 @throws std::exception       If `properties` already has properties with the
                              names that this function would need to add.
 @pre 0 < `resolution_hint` < ∞ and `properties` is not nullptr.  */
void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties);

/** Overload, intended for shapes that don't get tessellated in their
 hydroelastic representation (e.g., HalfSpace and Mesh).
 See @ref hug_properties.  */
void AddRigidHydroelasticProperties(ProximityProperties* properties);

/** Adds properties to the given set of proximity properties sufficient to cause
 the associated geometry to generate a compliant hydroelastic representation.
 The geometry's pressure field will be the function p(e) = Ee, where E is the
 hydroelastic modulus stored in the given `properties`.

 @param resolution_hint      If the geometry is to be tessellated, it is the
                             parameter that guides the level of mesh
                             refinement. It has length units (in meters) and
                             roughly corresponds to a typical edge length in
                             the resulting mesh.  See @ref hug_properties.
                             This will be ignored for geometry types that don't
                             require tessellation.
 @param hydroelastic_modulus A multiplier that maps penetration to pressure. See
                             @ref hug_properties.
 @param[in,out] properties   The properties will be added to this property set.
 @throws std::exception      If `properties` already has properties with the
                             names that this function would need to add.
 @pre 0 < `resolution_hint` < ∞, 0 < `hydroelastic_modulus`, and `properties`
      is not nullptr. */
void AddCompliantHydroelasticProperties(double resolution_hint,
                                        double hydroelastic_modulus,
                                        ProximityProperties* properties);

/** Compliant half spaces are handled as a special case; they do not get
 tessellated. Instead, they are treated as infinite slabs with a finite
 thickness. This variant is required for hydroelastic half spaces.

 @param slab_thickness       The distance from the half space boundary to its
                             rigid core (this helps define the extent field of
                             the half space).
 @param hydroelastic_modulus A multiplier that maps penetration to pressure. See
                             @ref hug_properties.
 @param[out] properties      The properties will be added to this property set.
 @throws std::exception If `properties` already has properties with the names
                        that this function would need to add.
 @pre 0 < `slab_thickness` < ∞, 0 < `hydroelastic_modulus`, and `properties`
      is not nullptr. */
void AddCompliantHydroelasticPropertiesForHalfSpace(
    double slab_thickness, double hydroelastic_modulus,
    ProximityProperties* properties);

//@}

}  // namespace geometry
}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::geometry::internal::HydroelasticType>
    : drake::ostream_formatter {};
}  // namespace fmt
#endif
