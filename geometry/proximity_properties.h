#pragma once

/** @file
 A collection of data types and functions to help manage defining properties
 for geometries with the proximity role. These functions facilitate properties
 that are _explicitly_ known in Drake's core functionality. These functions in
 no way limit the inclusion of any other additional, arbitrary properties.
 */

#include <functional>

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace multibody {

// Forward declaration
template <typename T>
class CoulombFriction;

}  // namespace multibody

namespace geometry {

/** @name  Declaring general contact material properties

 String constants to use to access the contact material properties that
 SceneGraph depends on. These are not the exhaustive set of contact material
 properties that downstream consumers (e.g., MultibodyPlant) may require.

 <!-- TODO(SeanCurtis-TRI): Extend this to include other contact material
 properties and an API for setting them more conveniently.  */
//@{

extern const char* kMaterialGroup;    ///< The contact material *group* name.
extern const char* kElastic;          ///< Elastic modulus property name.

//@}

/** @name  Static pressure for soft hydroelastic geometry

 In the hydroelastic geometry model, soft geometries must define a "static
 pressure" field p₀ in the domain of the mesh. SceneGraph computes the "extent"
 field -- a measure of how deeply a point penetrates into the mesh -- and the
 static pressure is a function of extent: p₀(e).

 The static pressure field is defined as an instance of PressureField providing
 both p₀(e) as well as dp₀/de. The two are specified independently to allow for
 the possibility of the need to approximate dp₀/de with an alternative function.

 The Make*PressureField() family of functions facilitates the creation of common
 definitions of static pressure field. Alternatively, a PressureField can be
 instantiated with arbitrary functions and provided to SceneGraph to define the
 hydroelastic geometry's behavior.

 @todo Add reference to overview of static pressure.
 */
//@{

/** The function that computes static pressure (p₀) from extent. Extent is a
 measure of penetration of a point inside the compliant region of a geometry and
 its range is [0, 1].  */
using PressureFunc = std::function<double(double)>;

/** The derivative of the static pressure function with respect to extent.  */
using PressureDerivFunc = std::function<double(double)>;

/** The definition of the static hydroelastic pressure field: p₀(e) (where e
 is a hydroelastic geometry's extent field) and its derivative: dp₀_de.  */
struct PressureField {
  PressureFunc p0;
  PressureDerivFunc dp0_de;
};

/** Defines a PressureField such that p₀(e) = e.  */
PressureField MakeUnitPressureField();

/** Defines a PressureField such that p₀(e) = Ee, where E is the elastic modulus
 of the geometry's material.
 @throws std::logic_error if `elastic_modulus` is infinite or non-positive.  */
PressureField MakeLinearPressureField(double elastic_modulus);

/** Defines the model to generate static pressure from extent for soft geometry.
 */
enum class PressureModel {
  kUnit,   ///< Invokes MakeUnitPressureField().
  kLinear  ///< Invokes MakeLinearPressureField().
};

/** Instantiates a pressure field based on the enumerated pressure model and the
 (possibly ignored) elastic modulus value.  */
PressureField MakePressureField(PressureModel pressure_model,
                                double elastic_modulus);

//@}

/** @name  Declaring geometry for hydroelastic contact.

 In order for a geometry to be used in hydroelastic contact, it must be declared
 as such. The declaration consists of setting a number of properties in the
 geometry's associated ProximityProperties.

 These utilities include:

   - string constants to read and write the indicated properties, and
   - utility functions for declaring consistent hydroelastic properties

 @todo Add reference to discussion of hydroelastic proximity properties along
 the lines of "For the full discussion of preparing geometry for use in the
 hydroelastic contact model, see `@ref MODULE_NOT_WRITTEN_YET`.
 */
//@{

extern const char* kHydroGroup;      ///< Hydroelastic *group* name.
extern const char* kRezHint;         ///< Resolution hint property name.
extern const char* kPressure;        ///< Static pressure field property name.

/** Adds properties to the given set of proximity properties sufficient to cause
 the associated geometry to generate a rigid hydroelastic representation.

 @param resolution_hint    If the geometry is to be tesselated, it is the
                           parameter that guides the level of mesh refinement.
                           See @ref MODULE_NOT_WRITTEN_YET. This will be ignored
                           for geometry types that don't require tesselation.
 @param[out] properties    The properties will be added to this property set.
 @throws std::logic_error  If `properties` already has properties with the names
                           that this function would need to add.
 @pre 0 < `resolution_hint` < ∞.  */
void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties);

/** Adds properties to the given set of proximity properties sufficient to cause
 the associated geometry to generate a soft hydroelastic representation.

 param resolution_hint     If the geometry is to be tesselated, it is the
                           parameter that guides the level of mesh refinement.
                           See @ref MODULE_NOT_WRITTEN_YET. This will be ignored
                           for geometry types that don't require tesselation.
 @param pressure_model     The type of pre-defined PressureField to apply to the
                           geometry.
 @param[out] properties    The properties will be added to this property set.
 @throws std::logic_error  If `properties` already has properties with the names
                           that this function would need to add.
 @pre 0 < `resolution_hint` < ∞ and `properties` contains a valid elasticity
      value. */
void AddSoftHydroelasticProperties(double resolution_hint,
                                   PressureModel pressure_model,
                                   ProximityProperties* properties);

/** Overloaded variant where the `static_pressure` p0 is provided explicitly.
 */
void AddSoftHydroelasticProperties(double resolution_hint,
                                   PressureField static_pressure,
                                   ProximityProperties* properties);

//@}

}  // namespace geometry
}  // namespace drake
