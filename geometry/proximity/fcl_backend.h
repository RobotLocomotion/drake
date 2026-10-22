#pragma once

#include <string_view>

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
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
 - D2 Request fields diverge in both directions. FCL has
      `enable_nearest_points`, `distance_tolerance` and `gjk_solver_type`;
      Coal has `gjk_tolerance` and `collision_distance_threshold` instead.
      Hence the Configure*Request() statics.
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

   Drake only ever registers the seven geometry types named below; every other
   node type the library can represent is reported as "Unsupported". Note in
   particular that Drake's Mesh and Convex shapes are both represented as
   convex hulls, and so report "Convex" -- GEOM_TRIANGLE (a BVH of triangles)
   is not something Drake creates.  */
  static std::string_view NodeTypeName(NodeType node_type) {
    switch (node_type) {
      case fcl::GEOM_BOX:
        return "Box";
      case fcl::GEOM_CAPSULE:
        return "Capsule";
      case fcl::GEOM_CONVEX:
        return "Convex";
      case fcl::GEOM_CYLINDER:
        return "Cylinder";
      case fcl::GEOM_ELLIPSOID:
        return "Ellipsoid";
      case fcl::GEOM_HALFSPACE:
        return "Halfspace";
      case fcl::GEOM_SPHERE:
        return "Sphere";
      case fcl::BV_UNKNOWN:
      case fcl::BV_AABB:
      case fcl::BV_OBB:
      case fcl::BV_RSS:
      case fcl::BV_kIOS:
      case fcl::BV_OBBRSS:
      case fcl::BV_KDOP16:
      case fcl::BV_KDOP18:
      case fcl::BV_KDOP24:
      case fcl::GEOM_CONE:
      case fcl::GEOM_OCTREE:
      case fcl::GEOM_PLANE:
      case fcl::GEOM_TRIANGLE:
      case fcl::NODE_COUNT:
        return "Unsupported";
    }
    DRAKE_UNREACHABLE();
  }
};

// clang-format off
}  // namespace internal
// clang-format on
}  // namespace geometry
}  // namespace drake
