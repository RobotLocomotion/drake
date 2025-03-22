#include "drake/geometry/proximity/make_convex_hull_mesh.h"

#include <string>
#include <utility>

#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Creates a convex hull for mesh and convex shapes.
class Hullifier final : public ShapeReifier {
 public:
  Hullifier() = default;
  ~Hullifier() = default;

  PolygonSurfaceMesh<double>&& release_hull() { return std::move(hull_); }

 private:
  void ThrowUnsupportedGeometry(const std::string& shape_name) final {
    throw std::runtime_error(fmt::format(
        "MakeConvexHull only applies to Mesh and Convex types, given {}.",
        shape_name));
  }
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Mesh& mesh, void*) final {
    hull_ = MakeConvexHull(mesh.source(), mesh.scale3());
  }

  void ImplementGeometry(const Convex& convex, void*) final {
    hull_ = MakeConvexHull(convex.source(), convex.scale3());
  }

  PolygonSurfaceMesh<double> hull_;
};

}  // namespace

PolygonSurfaceMesh<double> MakeConvexHull(const Shape& shape) {
  Hullifier hullifier;
  shape.Reify(&hullifier);
  return hullifier.release_hull();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
