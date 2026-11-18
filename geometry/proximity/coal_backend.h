#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/distance.h>
#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

/* The backend policy for Coal (https://github.com/coal-library/coal), a
 descendant of FCL. See fcl_backend.h for what a backend policy is and for the
 catalogue (D1-D6) of what the two libraries disagree about; this file is the
 other half of each of those six. */
struct CoalBackend {
  /* Identifies this policy; see collision_backend.h. */
  static constexpr CollisionBackend kId = CollisionBackend::kCoal;

  using Object = coal::CollisionObject;

  /* The signatures Drake's narrowphase callbacks present to the broadphase
   (see D5). These are FCL-shaped by choice: Drake's callbacks are free
   functions taking an opaque `void*`, and the adapters below bridge them to
   the callback objects Coal wants.  */
  using CollisionFunction = bool (*)(Object*, Object*, void*);
  using DistanceFunction = bool (*)(Object*, Object*, void*, coal::Scalar&);

  using Geometry = coal::CollisionGeometry;
  using NodeType = coal::NODE_TYPE;
  using Transform = coal::Transform3s;

  using Box = coal::Box;
  using Capsule = coal::Capsule;
  /* Coal's convex polytope is templated on the polygon type it stores. Drake
   always uses 32-bit triangles; see MakeFaces(). */
  using Convex = coal::Convex<coal::Triangle32>;
  using Cylinder = coal::Cylinder;
  using Ellipsoid = coal::Ellipsoid;
  using Halfspace = coal::Halfspace;
  using Sphere = coal::Sphere;

  /* The common base of the primitive shape types, which the engine holds
   while reifying. */
  using ShapeBase = coal::ShapeBase;

  /* The per-face data a Convex is built from (see D4). Coal stores triangles,
   so Drake's polygon encoding has to be triangulated; see MakeFaces(). */
  using FaceData = std::vector<coal::Triangle32>;

  using CollisionRequest = coal::CollisionRequest;
  using CollisionResult = coal::CollisionResult;
  using DistanceRequest = coal::DistanceRequest;
  using DistanceResult = coal::DistanceResult;
  using Contact = coal::Contact;
  using Broadphase = coal::DynamicAABBTreeCollisionManager;

  /* The scalar the library's own API is written in. Drake's broadphase
   distance callbacks take their threshold by reference in this type.  */
  using Scalar = coal::Scalar;

  /* Configures a request asking only whether two geometries are in strict
   penetration -- no contact data (see D2).  */
  static void ConfigureHasCollisionRequest(CollisionRequest* request) {
    request->num_max_contacts = 1;
    request->enable_contact = false;
    // This is the tolerance Drake has historically asked of the GJK solver.
    // Note that we deliberately leave `epa_tolerance` at Coal's default.
    request->gjk_tolerance = 2e-12;
    // Coal defaults this to 1e-12, which would report pairs that are
    // separated by a positive distance as colliding. Drake wants strict
    // penetration, so we ask for it explicitly.
    request->collision_distance_threshold = 0;
  }

  /* Configures a request for a single contact, with contact data (see D2).
   See ConfigureHasCollisionRequest() for the notes on the two tolerances.  */
  static void ConfigurePenetrationRequest(CollisionRequest* request) {
    request->num_max_contacts = 1;
    request->enable_contact = true;
    request->gjk_tolerance = 2e-12;
    request->collision_distance_threshold = 0;
  }

  /* Returns the contact's penetration depth as Drake defines it: positive
   when the geometries overlap (see D3). Coal reports
   Contact::penetration_depth as a *signed distance*, negative when the shapes
   overlap, so this negates it.  */
  static double ContactDepth(const Contact& contact) {
    return -contact.penetration_depth;
  }

  /* Configures a request for a signed-distance query that reports witness
   points, with `tol` governing the accuracy of the separated-distance answer
   (see D2). Coal has no distance_tolerance and no enable_nearest_points; the
   knob that governs the accuracy of the separated-distance answer is the GJK
   tolerance, and witness points always come back. We leave `epa_tolerance` at
   Coal's default.  */
  static void ConfigureSignedDistanceRequest(DistanceRequest* request,
                                             double tol) {
    request->enable_signed_distance = true;
    request->gjk_tolerance = tol;
  }

  /* Performs a narrowphase distance query between two geometries at the given
   poses. Results are expressed in whatever frame the poses are measured in.
   */
  static void Distance(const Geometry* a, const Transform& X_WA,
                       const Geometry* b, const Transform& X_WB,
                       const DistanceRequest& request, DistanceResult* result) {
    coal::distance(a, X_WA, b, X_WB, request, *result);
  }

  /* Performs narrowphase collision between two objects.  */
  static void Collide(const Object* a, const Object* b,
                      const CollisionRequest& request,
                      CollisionResult* result) {
    coal::collide(a, b, request, *result);
  }

 private:
  /* Coal's broadphase takes a callback *object* where FCL took a function
   pointer plus an opaque `void*` (see D5). Drake's narrowphase callbacks keep
   the FCL-style signature -- they are free functions that the unit tests also
   invoke directly -- so these adapters bind such a function and its data into
   the object Coal wants. They are only ever created on the stack,
   immediately before the broadphase call that consumes them.  */
  class CollisionAdapter final : public coal::CollisionCallBackBase {
   public:
    CollisionAdapter(CollisionFunction function, void* data)
        : function_(function), data_(data) {}

    bool collide(Object* o1, Object* o2) final {
      return function_(o1, o2, data_);
    }

   private:
    CollisionFunction function_{};
    void* data_{};
  };

  class DistanceAdapter final : public coal::DistanceCallBackBase {
   public:
    DistanceAdapter(DistanceFunction function, void* data)
        : function_(function), data_(data) {}

    bool distance(Object* o1, Object* o2, Scalar& dist) final {
      return function_(o1, o2, data_, dist);
    }

   private:
    DistanceFunction function_{};
    void* data_{};
  };

 public:
  /* @name Broadphase traversal (see D5)

   These also absorb the libraries' inconsistent use of `const`: the
   broadphase structures do not change during a query, but are nevertheless
   declared non-const.  */
  //@{
  static void Collide(const Broadphase& tree, void* data,
                      CollisionFunction callback) {
    CollisionAdapter adapter(callback, data);
    tree.collide(&adapter);
  }

  static void Collide(const Broadphase& tree1, const Broadphase& tree2,
                      void* data, CollisionFunction callback) {
    CollisionAdapter adapter(callback, data);
    tree1.collide(const_cast<Broadphase*>(&tree2), &adapter);
  }

  static void Distance(const Broadphase& tree, void* data,
                       DistanceFunction callback) {
    DistanceAdapter adapter(callback, data);
    tree.distance(&adapter);
  }

  static void Distance(const Broadphase& tree, Object* object, void* data,
                       DistanceFunction callback) {
    DistanceAdapter adapter(callback, data);
    tree.distance(object, &adapter);
  }

  static void Distance(const Broadphase& tree1, const Broadphase& tree2,
                       void* data, DistanceFunction callback) {
    DistanceAdapter adapter(callback, data);
    tree1.distance(const_cast<Broadphase*>(&tree2), &adapter);
  }
  //@}

  /* Converts a Drake pose into the transform type the library's API consumes.
   */
  static Transform ToTransform(const math::RigidTransformd& X_AB) {
    return Transform(X_AB.rotation().matrix(), X_AB.translation());
  }

  /* Converts a transform from the library's API back into a Drake pose; the
   inverse of ToTransform(). */
  static math::RigidTransformd FromTransform(const Transform& X_AB) {
    return math::RigidTransformd(math::RotationMatrixd(X_AB.getRotation()),
                                 X_AB.getTranslation());
  }

  /* Triangulates the faces of a convex PolygonSurfaceMesh -- given in that
   class's flat [count, v0, v1, ..., count, ...] encoding -- into the triangle
   list a Convex requires (see D4). Each n-gon becomes a fan of n - 2
   triangles around its first vertex, preserving the polygon's vertex order.

   Note that Coal stores this list but, as Drake constructs a Convex, derives
   nothing from it -- the support function reads the vertex array -- so no
   proximity query observes whether this is right. It is tested directly in
   coal_backend_test.cc rather than through the engine.

   @pre `face_data` is the face_data() of a PolygonSurfaceMesh whose faces are
        all planar and convex (which is true of a convex hull).  */
  static FaceData MakeFaces(const std::vector<int>& face_data);

  /* Returns the number of faces in `faces`. Unlike FCL, Coal's count is the
   triangle count, which triangulation has made larger than the polygon count
   (see D4).  */
  static int CountFaces(const FaceData& faces, int /* num_polygons */) {
    return ssize(faces);
  }

  /* Constructs a convex hull geometry.  */
  static std::shared_ptr<Convex> MakeConvex(
      std::shared_ptr<std::vector<Vector3<double>>> vertices, int num_vertices,
      std::shared_ptr<FaceData> faces, int num_faces) {
    return std::make_shared<Convex>(std::move(vertices), num_vertices,
                                    std::move(faces), num_faces);
  }

  /* Maps the library's node type onto the backend-independent ShapeKind that
   shared code switches on (see D6). Everything Drake does not register --
   cones, planes, octrees, triangle BVHs, height fields and the
   bounding-volume types -- reports kUnsupported.

   Note that Coal's GEOM_CONVEX is an alias for GEOM_CONVEX32, so it must not
   get a case label of its own here.  */
  static ShapeKind Classify(NodeType node_type) {
    switch (node_type) {
      case coal::GEOM_BOX:
        return ShapeKind::kBox;
      case coal::GEOM_CAPSULE:
        return ShapeKind::kCapsule;
      case coal::GEOM_CONVEX16:
      case coal::GEOM_CONVEX32:
        return ShapeKind::kConvex;
      case coal::GEOM_CYLINDER:
        return ShapeKind::kCylinder;
      case coal::GEOM_ELLIPSOID:
        return ShapeKind::kEllipsoid;
      case coal::GEOM_HALFSPACE:
        return ShapeKind::kHalfspace;
      case coal::GEOM_SPHERE:
        return ShapeKind::kSphere;
      case coal::BV_UNKNOWN:
      case coal::BV_AABB:
      case coal::BV_OBB:
      case coal::BV_RSS:
      case coal::BV_kIOS:
      case coal::BV_OBBRSS:
      case coal::BV_KDOP16:
      case coal::BV_KDOP18:
      case coal::BV_KDOP24:
      case coal::GEOM_CONE:
      case coal::GEOM_OCTREE:
      case coal::GEOM_PLANE:
      case coal::GEOM_TRIANGLE:
      case coal::HF_AABB:
      case coal::HF_OBBRSS:
      case coal::NODE_COUNT:
        return ShapeKind::kUnsupported;
    }
    DRAKE_UNREACHABLE();
  }

  /* @name Shape accessors (see D1)

   Coal stores the halves where FCL stores the full extents, which is exactly
   why shared code goes through these rather than touching members directly.
   */
  //@{
  static Eigen::Vector3d half_extents(const Box& box) { return box.halfSide; }
  static double radius(const Capsule& capsule) { return capsule.radius; }
  static double half_length(const Capsule& capsule) {
    return capsule.halfLength;
  }
  static double radius(const Cylinder& cylinder) { return cylinder.radius; }
  static double half_length(const Cylinder& cylinder) {
    return cylinder.halfLength;
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
