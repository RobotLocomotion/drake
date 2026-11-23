#pragma once

#include <string_view>

#include <fcl/fcl.h>

#include "drake/common/drake_export.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

/* @file
 Drake's proximity queries are written against a *backend policy*: a trait
 struct that names the third-party collision library's types and wraps the
 handful of places where libraries of this family disagree. The algorithm code
 is templated on that policy, so a single source of truth serves every backend.

 FclBackend, below, is the policy for FCL
 (https://github.com/flexible-collision-library/fcl).

 <h3>What a policy must absorb</h3>

 The policy boundary accounts for differences between FCL and Coal
 (https://github.com/coal-library/coal), a descendant of FCL. The two libraries
 agree on most of the surface Drake uses -- `getNodeType()`,
 `collisionGeometry()`, `setUserData()`, `setTransform()`, `computeLocalAABB()`,
 the broadphase manager's register/update/getObjects methods, and the shapes of
 the free `collide()` and `distance()` functions are all common. They disagree
 in six places, and absorbing exactly these six is the policy's whole job:

 - D1 Shape data members are spelled differently. FCL stores a box's full
      extent in `side` and a cylinder's full length in `lz`; Coal stores
      `halfSide` and `halfLength`. Hence the static accessors (half_extents(),
      half_length(), ...) rather than direct member access at the call sites.
      Constructors, by contrast, agree, so reification needs no shim.
 - D2 Request fields diverge in both directions. On `DistanceRequest`, FCL's
      `distance_tolerance` and `gjk_solver_type` have no Coal counterpart
      (Coal's accuracy knob is `gjk_tolerance`), and Coal deprecates
      `enable_nearest_points` because it always computes nearest points. On
      `CollisionRequest`, both libraries have `gjk_tolerance`, but only FCL has
      `gjk_solver_type`. Conversely, Coal's requests (via their common
      `QueryRequest` base) have `collision_distance_threshold`, which FCL
      lacks. Hence the Configure*Request() statics.
 - D3 Contact sign convention. FCL's `Contact::penetration_depth` is a positive
      depth; Coal's is a signed distance, negative when the shapes overlap.
      Drake wants a positive overlap. Hence ContactDepth().
 - D4 Convex hull representation. FCL's `Convex` consumes Drake's
      PolygonSurfaceMesh face encoding directly; Coal's needs the faces
      fan-triangulated into `Triangle32`, and its constructor takes a different
      argument list. Hence MakeFaces()/MakeConvex().
 - D5 Broadphase callback protocol. FCL takes a function pointer plus an opaque
      `void* cdata`; Coal takes a subclass of CollisionCallBackBase /
      DistanceCallBackBase. Drake's narrowphase callbacks keep the FCL-style
      free-function signature and the Coal policy adapts them. Hence the
      Collide()/Distance() broadphase overloads.
 - D6 The NODE_TYPE enumerator sets differ. Coal adds GEOM_CONVEX16, HF_AABB
      and HF_OBBRSS, makes GEOM_CONVEX an alias for GEOM_CONVEX32, and orders
      the enumerators differently. An exhaustive switch therefore cannot be
      shared. Hence NodeTypeName().

 The policy interface is structural: it contains only the members used by
 backend-independent proximity code, with no separate base class or interface
 declaration to keep synchronized. */
struct FclBackend {
  using Object = fcl::CollisionObjectd;
  using Geometry = fcl::CollisionGeometryd;
  using NodeType = fcl::NODE_TYPE;

  /* Returns the name Drake uses for the given node type (see D6).

   Drake only ever registers the seven geometry types named in the
   implementation; every other node type the library can represent is reported
   as "Unsupported". Note in particular that Drake's Mesh and Convex shapes are
   both represented as convex hulls, and so report "Convex". Neither
   GEOM_TRIANGLE (FCL's single-triangle shape, TriangleP) nor a BVH of
   triangles (whose node type is one of the BV_* values) is something Drake
   creates.  */
  static std::string_view NodeTypeName(NodeType node_type);
};

// clang-format off
}  // namespace internal
// clang-format on
}  // namespace geometry
}  // namespace drake
