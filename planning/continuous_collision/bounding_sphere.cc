#include "drake/planning/continuous_collision/bounding_sphere.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Convex;
using drake::geometry::Cylinder;
using drake::geometry::Ellipsoid;
using drake::geometry::Mesh;
using drake::geometry::PolygonSurfaceMesh;
using drake::geometry::Shape;
using drake::geometry::ShapeReifier;
using drake::geometry::Sphere;
using drake::math::RigidTransform;

/* Computes the bounding sphere of a supported shape posed at X_LG in a body
 (link) frame L.

 Every formula below is an *exact containment* statement about the shape's
 canonical frame G: `radius` is the circumradius of the shape about Go, and the
 sphere is centred at Go's image in L, i.e. c_L = X_LG.translation(). Because
 the rotation part of X_LG is an isometry, ‖X_LG·p − c_L‖ = ‖R_LG·p‖ = ‖p‖ for
 every material point p of the shape, so containment in L follows from
 containment in G with no dependence on the orientation. That is why the centre
 never needs a search and the radius never needs inflating for rotation.

 The origin-centred radius the reach chain consumes is ‖c_L‖ + radius, sound by
 the triangle inequality; the tighter centre is what the broadphase prefilter
 wants.

 An under-bounding formula produces an unsound λ with no other symptom, so this
 reifier enumerates the closed set of supported shapes and lets every other
 shape fall through to ShapeReifier's default, which routes to
 ThrowUnsupportedGeometry() below. */
class BoundingSphereReifier final : public ShapeReifier {
 public:
  explicit BoundingSphereReifier(const RigidTransform<double>& X_LG)
      : X_LG_(X_LG) {}

  const BoundingSphere& sphere() const { return sphere_; }

  /* Pulls in ShapeReifier's throwing defaults for every shape this class does
   not override below (HalfSpace, MeshcatCone, and any shape a future Drake
   adds). The overrides declared after it hide the corresponding defaults. */
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void*) final {
    SetCentered(sphere.radius());
  }

  void ImplementGeometry(const Box& box, void*) final {
    // Drake's Box stores FULL side lengths, so the circumradius about the box
    // centre is half the space diagonal: max over the 8 corners
    // (±w/2, ±d/2, ±h/2) of ‖c‖ = ½·√(w² + d² + h²).
    SetCentered(0.5 * box.size().norm());
  }

  void ImplementGeometry(const Capsule& capsule, void*) final {
    // Spine segment [−L/2, L/2]·ẑ inflated by r; the farthest point is a pole.
    SetCentered(0.5 * capsule.length() + capsule.radius());
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) final {
    // The farthest point from Go is always on a rim. For a point
    // p = z·ẑ + r'·û with |z| ≤ L/2, r' ≤ r and û ⊥ ẑ,
    //   ‖p‖² = z² + r'²,
    // which is maximised at |z| = L/2 and r' = r, so R = √(r² + (L/2)²).
    // Cap-disk interior points (r' < r) and lateral points with |z| < L/2 are
    // both strictly dominated. An origin-centred form of the same argument
    // would pick up the ‖t‖ cross terms; here the centre rides along with the
    // geometry, so only the canonical-frame extent matters.
    SetCentered(std::hypot(cylinder.radius(), 0.5 * cylinder.length()));
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) final {
    // ‖diag(a,b,c)·u‖ ≤ max(a,b,c)·‖u‖ for every unit u, with equality along
    // the largest semi-axis: exact for the axis-aligned ellipsoid in its own
    // frame, which is all this centre-following sphere needs.
    SetCentered(std::max({ellipsoid.a(), ellipsoid.b(), ellipsoid.c()}));
  }

  void ImplementGeometry(const Convex& convex, void*) final {
    SetFromHull(convex.GetConvexHull());
  }

  void ImplementGeometry(const Mesh& mesh, void*) final {
    // Drake collides a Mesh as its convex hull in signed-distance queries, and
    // the hull contains the mesh, so bounding the hull bounds the geometry
    // actually checked.
    SetFromHull(mesh.GetConvexHull());
  }

 private:
  void ThrowUnsupportedGeometry(const std::string& shape_name) final {
    throw std::runtime_error(fmt::format(
        "ComputeBoundingSphere(): does not support the shape "
        "type '{}'. Supported proximity shapes are Sphere, Box, Capsule, "
        "Cylinder, Ellipsoid, Convex and Mesh. HalfSpace has no finite "
        "bounding sphere and is governed by dedicated rules instead: it must "
        "be anchored, or move only by translation relative to its partner. "
        "Any other shape must be replaced by a Convex/Mesh approximation "
        "before it can be certified.",
        shape_name));
  }

  /* Sets the sphere centred on the geometry frame origin's image in L, with
   the given circumradius about that origin. */
  void SetCentered(double radius_about_Go) {
    DRAKE_DEMAND(std::isfinite(radius_about_Go));
    DRAKE_DEMAND(radius_about_Go >= 0.0);
    sphere_.center_L = X_LG_.translation();
    sphere_.radius = radius_about_Go;
  }

  /* Centroid-centred sphere over the hull vertices. Unlike the primitives this
   sphere is NOT centred on Go: the centroid is a much better centre for the
   broadphase prefilter, and ‖c_L‖ + radius still bounds the origin-centred
   reach the λ chain needs. The hull is a convex polytope, so containing every
   vertex contains the whole shape. */
  void SetFromHull(const PolygonSurfaceMesh<double>& hull) {
    const int num_vertices = hull.num_vertices();
    // Drake's hull computation refuses degenerate vertex sets, so a hull
    // always has at least a tetrahedron's worth of vertices; assert the
    // non-empty precondition the centroid needs regardless.
    DRAKE_DEMAND(num_vertices > 0);
    Eigen::Vector3d centroid_L = Eigen::Vector3d::Zero();
    for (int v = 0; v < num_vertices; ++v) {
      centroid_L += X_LG_ * hull.vertex(v);
    }
    centroid_L /= static_cast<double>(num_vertices);
    double radius = 0.0;
    for (int v = 0; v < num_vertices; ++v) {
      radius = std::max(radius, (X_LG_ * hull.vertex(v) - centroid_L).norm());
    }
    sphere_.center_L = centroid_L;
    sphere_.radius = radius;
  }

  const RigidTransform<double>& X_LG_;
  BoundingSphere sphere_;
};

}  // namespace

BoundingSphere ComputeBoundingSphere(const Shape& shape,
                                     const RigidTransform<double>& X_LG) {
  BoundingSphereReifier reifier(X_LG);
  shape.Reify(&reifier);
  const BoundingSphere& result = reifier.sphere();
  // A zero or non-finite radius under-bounds every λ built on it, so
  // re-assert the postcondition every caller relies on.
  DRAKE_DEMAND(std::isfinite(result.radius) && result.radius >= 0.0);
  DRAKE_DEMAND(result.center_L.allFinite());
  return result;
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
