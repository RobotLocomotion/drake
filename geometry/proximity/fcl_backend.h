#pragma once

#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/math/rigid_transform.h"

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
      shared. Hence Classify(), which maps the library's node type onto the
      backend-independent ShapeKind that shared code switches on instead.

 The policy interface is structural: it contains only the members used by
 backend-independent proximity code, with no separate base class or interface
 declaration to keep synchronized. */
struct FclBackend {
  using Object = fcl::CollisionObjectd;
  using Geometry = fcl::CollisionGeometryd;
  using NodeType = fcl::NODE_TYPE;
  using Transform = Eigen::Isometry3d;

  using Box = fcl::Boxd;
  using Capsule = fcl::Capsuled;
  using Convex = fcl::Convexd;
  using Cylinder = fcl::Cylinderd;
  using Ellipsoid = fcl::Ellipsoidd;
  using Halfspace = fcl::Halfspaced;
  using Sphere = fcl::Sphered;

  /* The per-face data a Convex is built from (see D4). FCL consumes Drake's
   PolygonSurfaceMesh encoding -- [count, v0, v1, ..., count, ...] -- as is. */
  using FaceData = std::vector<int>;

  using CollisionRequest = fcl::CollisionRequestd;
  using CollisionResult = fcl::CollisionResultd;
  using DistanceRequest = fcl::DistanceRequestd;
  using DistanceResult = fcl::DistanceResultd;

  /* The scalar the library's own API is written in. Drake's broadphase
   distance callbacks take their threshold by reference in this type.  */
  using Scalar = double;

  /* Configures a request asking only whether two geometries are in strict
   penetration -- no contact data (see D2).

   NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
   superior to FCL's "independent" implementation. Furthermore, libccd
   appears to behave badly if its gjk tolerance is much tighter than
   2e-12. Until this changes, we explicitly specify these parameters rather
   than relying on FCL's defaults.  */
  static void ConfigureHasCollisionRequest(CollisionRequest* request) {
    request->num_max_contacts = 1;
    request->enable_contact = false;
    request->gjk_tolerance = 2e-12;
    request->gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  }

  /* Configures a request for a signed-distance query that reports witness
   points, with `tol` governing the accuracy of the separated-distance answer
   (see D2).  */
  static void ConfigureSignedDistanceRequest(DistanceRequest* request,
                                             double tol) {
    request->enable_nearest_points = true;
    request->enable_signed_distance = true;
    request->distance_tolerance = tol;
    request->gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  }

  /* Performs a narrowphase distance query between two geometries at the given
   poses. Results are expressed in whatever frame the poses are measured in.
   */
  static void Distance(const Geometry* a, const Transform& X_WA,
                       const Geometry* b, const Transform& X_WB,
                       const DistanceRequest& request, DistanceResult* result) {
    fcl::distance(a, X_WA, b, X_WB, request, *result);
  }

  /* Performs narrowphase collision between two objects.  */
  static void Collide(const Object* a, const Object* b,
                      const CollisionRequest& request,
                      CollisionResult* result) {
    fcl::collide(a, b, request, *result);
  }

  /* Converts a Drake pose into the transform type the library's API consumes.
   */
  static Transform ToTransform(const math::RigidTransformd& X_AB) {
    return X_AB.GetAsIsometry3();
  }

  /* Converts the faces of a convex PolygonSurfaceMesh, given in that class's
   flat encoding, into the face data a Convex requires (see D4). FCL stores
   polygons directly, so this is the identity. */
  static FaceData MakeFaces(const std::vector<int>& face_data) {
    return face_data;
  }

  /* Returns the number of faces in `faces`, which for FCL is the number of
   polygons the mesh reported rather than anything derived from `faces` (see
   D4). */
  static int CountFaces(const FaceData&, int num_polygons) {
    return num_polygons;
  }

  /* Constructs a convex hull geometry. The argument list is normalized across
   backends; FCL infers its vertex count and does not use `num_vertices` (see
   D4). */
  static std::shared_ptr<Convex> MakeConvex(
      std::shared_ptr<std::vector<Vector3<double>>> vertices,
      int /* num_vertices */, std::shared_ptr<FaceData> faces, int num_faces) {
    return std::make_shared<Convex>(std::move(vertices), num_faces,
                                    std::move(faces));
  }

  /* Maps the library's node type onto the backend-independent ShapeKind that
   shared code switches on (see D6). Everything Drake does not register --
   cones, planes, octrees, triangle BVHs and the bounding-volume types --
   reports kUnsupported. Note in particular that Drake's Mesh and Convex
   shapes are both represented as convex hulls, so GEOM_TRIANGLE (a BVH of
   triangles) is not something Drake creates.  */
  static ShapeKind Classify(NodeType node_type) {
    switch (node_type) {
      case fcl::GEOM_BOX:
        return ShapeKind::kBox;
      case fcl::GEOM_CAPSULE:
        return ShapeKind::kCapsule;
      case fcl::GEOM_CONVEX:
        return ShapeKind::kConvex;
      case fcl::GEOM_CYLINDER:
        return ShapeKind::kCylinder;
      case fcl::GEOM_ELLIPSOID:
        return ShapeKind::kEllipsoid;
      case fcl::GEOM_HALFSPACE:
        return ShapeKind::kHalfspace;
      case fcl::GEOM_SPHERE:
        return ShapeKind::kSphere;
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
        return ShapeKind::kUnsupported;
    }
    DRAKE_UNREACHABLE();
  }

  /* @name Shape accessors (see D1)

   The libraries spell these differently -- FCL stores a box's full extent and
   a capsule's/cylinder's full length, Coal stores the halves -- so shared code
   goes through these rather than touching members directly.  */
  //@{
  static Eigen::Vector3d half_extents(const Box& box) { return box.side / 2.0; }
  static double radius(const Capsule& capsule) { return capsule.radius; }
  static double half_length(const Capsule& capsule) { return capsule.lz / 2.0; }
  static double radius(const Cylinder& cylinder) { return cylinder.radius; }
  static double half_length(const Cylinder& cylinder) {
    return cylinder.lz / 2.0;
  }
  static double radius(const Sphere& sphere) { return sphere.radius; }
  static const Eigen::Vector3d& radii(const Ellipsoid& ellipsoid) {
    return ellipsoid.radii;
  }
  static const Eigen::Vector3d& normal(const Halfspace& halfspace) {
    return halfspace.n;
  }
  static double offset(const Halfspace& halfspace) { return halfspace.d; }
  //@}
};

// clang-format off
}  // namespace internal
// clang-format on
}  // namespace geometry
}  // namespace drake
