#pragma once

#include <iostream>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_properties.h"

namespace drake {
namespace geometry {

/** @addtogroup geometry_roles

 Geometry roles help define how a real-world object is modeled in Drake.

 We model the physical presence of real-world objects with geometric
 representations. For a single object, we can assign many different properties
 to the corresponding geometry depending on how the geometry is used (or what
 aspect of the real-world object it represents). There are many operations that
 can be applied to geometric representations. Each operation may only depend on
 some of the possible properties. Furthermore, it may be advantageous to
 represent a single real-world object with different geometries for different
 operations.

 __An example__

 A physical robot arm (such as the KUKA Iiwa) has a great deal of detail:
 flanges, vents, bolts, inset holes, stylistic creases, shiny paint, dull paint,
 lettering, etc. Furthermore, the materials the arm is made of have meaningful
 properties (e.g., Young's modulus, Poisson ratio, etc.) All of these
 taken together are part of the physical whole.

 The physical arm is partially represented in Drake with geometric
 approximations and a set of properties which map the physical detail to the
 Drake's underlying mathematical models (other aspects of the arm, e.g., mass
 properties, are associated with the arm components as "bodies"). There are many
 things we may want to use this geometric representation of the arm for:

 - Display the progress of an interactive simulation of the arm in a GUI-based
   visualization tool.
 - Simulate a perception system which estimates arm state based on RGB images
   of the arm.
 - Compute contact forces between the virtual arm and its virtual environment.
 - Find clearances between objects.
 - Simulate what a camera or other sensor reports when exposed to the simulated
   environment.
 - Calculate aerodynamic or fluid forces acting on objects.
 - And more in the future.

 These are all meaningful operations on the virtual arm, but each operation
 requires a subset of the objects' properties. For example, the Young's
 modulus of the arm's end effector is not relevant to the external display
 of the state of the arm.

 Beyond the relevancy of a property for any given operation, two different
 operations may be best served by having different geometric representations of
 the same physical object. For example, in modeling a bowling ball, the finger
 holes may not be necessary for determining contact forces and by simply
 representing the ball as a sphere the operation becomes much more efficient.
 On the other hand, a perception system for a robot that is attempting to
 pick up the ball and place fingers in the holes, would require a representation
 with the actual holes.

 __Geometry roles and geometry operations__

 _Geometry roles_ are the Drake mechanism that allows us to represent a single
 real-world object with different geometries best suited to the type of
 operation. (Rather than a single, monolithic representation which may be
 ill-suited for all operations.)

 Drake partitions its geometry operations into classes (in the non-C++ sense of
 the word) and defines a unique role for each class of operations.

 <!-- TODO(SeanCurtis-TRI): Can we come up with a better name than "proximity"
   akin to "Perception" and "Illustration"? -->
   - __Proximity role__: these are the operations that are related to
     evaluations of the signed distance between two geometries. When the
     objects are separated, the distance is positive, when penetrating, the
     distance is negative. The distance value can be characterized by e.g.,
     nearest points in the separated case and by points of deepest penetration
     in the penetrating case (although this is not an exhaustive list). Due to
     the cost of these types of algorithms, these geometric representations tend
     to be simple approximations of real-world objects: single primitive shapes,
     unions of multiple primitive shapes, simple convex meshes, lower resolution
     versions of otherwise complex meshes. Typically, these types of queries are
     used in motion planning and the generation of contact forces. The
     properties associated with this role are typically things like Young's
     modulus, Poisson ratio, coefficients of friction, etc. Generally, these
     properties don't affect the _geometric_ operation, but are used in
     conjunction with the geometry query results to produce forces and the like.
     Since these properties are defined on a per-geometry basis, SceneGraph
     stores these quantities with the geometries as a courtesy and these values
     can be requested from SceneGraph to, e.g., calculate forces. This role is
     unique in this regard -- the geometry parameters for the other roles
     affect the result of the geometric operation.
   - __Perception role__: these are the operations that contribute to sensor
     simulation. In other words, what can be seen? Typically, these are meshes
     of medium to high fidelity (depending on the fidelity of the sensor). The
     properties are models of the real world object's optical properties (its
     color, shininess, opacity, etc.)
   - __Illustration role__: these are the operations that connect drake to some
     external visualizers. The intent is that geometries with this role don't
     contribute to system calculations, they provide the basis for visualizing,
     or _illustrating_, the state of the system. It can include geometries that
     illustrate abstract concepts or geometries which are literal
     representations of real-world object surfaces. The properties associated
     with this role are those necessary to draw the illustration.

 Role assignment is achieved by assigning a set of role-related _properties_
 to a geometry. The properties can either be assigned to the GeometryInstance
 prior to registration, or after registration via the registered geometry's
 identifier (see SceneGraph::AssignRole()). The set _can_ be empty. Each
 role has a specific property set associated with it:
   - __Proximity role__: ProximityProperties
   - __Perception role__: PerceptionProperties
   - __Illustration role__: IllustrationProperties

 Even for a single role, different consumers of a geometry may use different
 properties. For example, a contact model is a likely consumer of geometries
 with the proximity role. However, how contact is implemented may vary from
 model to model. Those implementations can require model-specific parameters.
 The ProximityProperties need to include properties compatible with all
 the consumers active in the system.

 In such a case, each consumer should document the properties it requires and
 how they are organized (see GeometryProperties). The properties can be
 segregated by collecting their model-specific parameters into a named property
 group as specified by each consumer.

 To make a geometry _universally_ compatible with _anything_ in Drake, it would
 provide properties for all known consumers of geometry properties (e.g.,
 contact models, visualizers, etc.) In practice, it is sufficient to satisfy
 those consumers used in a particular system.

 Finally, a geometry is not limited to having a single role. A geometry must
 always have at least one role to have any impact. But it can have multiple
 roles.

 Generally, any code that is dependent on geometry roles, should document the
 type of role that it depends on, and the properties (if any) associated with
 that role that it requires/prefers.

 Next topic: @ref proximity_queries  */

/** The set of properties for geometry used in a _proximity_ role.

 <!-- TODO(SeanCurtis-TRI): When the hydroelastic geometry module is written,
  put a reference to the discussion of ProximityProperties here.  -->
 Examples of functionality that depends on the proximity role:
 <!-- TODO(SeanCurtis-TRI): Write up a module on hydroelastic proximity
       properties and link to that.  -->
 */
class ProximityProperties final : public GeometryProperties {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ProximityProperties);
  // TODO(SeanCurtis-TRI): Should this have the physical properties built in?
  ProximityProperties() = default;
};

/** The set of properties for geometry used in a "perception" role.

 Examples of functionality that depends on the perception role:
   - render::RenderEngineVtk
   - render::RenderEngineOspray
 */
class PerceptionProperties final : public GeometryProperties{
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PerceptionProperties);
  // TODO(SeanCurtis-TRI): Should this have a render label built in?
  PerceptionProperties() = default;
};

/** The set of properties for geometry used in an "illustration" role.

 Examples of functionality that depends on the illustration role:
   - @ref geometry_visualization_role_dependency "drake::geometry::ConnectDrakeVisualizer()"
 */
class IllustrationProperties final : public GeometryProperties {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IllustrationProperties);

  IllustrationProperties() = default;
};

/** General enumeration for indicating geometry role.  */
enum class Role {
  // Bitmask-able values so they can be OR'd together.
  kUnassigned = 0x0,
  kProximity = 0x1,
  kIllustration = 0x2,
  kPerception = 0x4
};

// NOTE: Currently this only includes new and replace; but someday it could also
// include other operations: merge, subtract, etc.
/** The operations that can be performed on the given properties when assigning
 roles to geometry.  */
enum class RoleAssign {
  kNew,      ///< Assign the properties to a geometry that doesn't already have
             ///< the role.
  kReplace   ///< Replace the existing role properties completely.
};

/** @name  Geometry role to string conversions

 These are simply convenience functions for converting the Role enumeration into
 a human-readable string.  */
//@{

std::string to_string(const Role& role);

std::ostream& operator<<(std::ostream& out, const Role& role);

//@}

/** @name  Convenience functions

 A collection of functions to help facilitate working with properties.  */
//@{

/** Constructs an IllustrationProperties instance compatible with a simple
 "phong" material using only the given `diffuse` color.  */
IllustrationProperties MakePhongIllustrationProperties(
    const Vector4<double>& diffuse);

//@}

}  // namespace geometry
}  // namespace drake
