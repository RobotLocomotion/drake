#include "drake/geometry/proximity/test/fcl_utilities.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

std::shared_ptr<fcl::Convexd> MakeConvex(const Convex& convex) {
  const PolygonSurfaceMesh<double>& hull = convex.GetConvexHull();
  auto vertices = std::make_shared<std::vector<Vector3d>>();
  vertices->reserve(hull.num_vertices());
  for (int i = 0; i < hull.num_vertices(); ++i) {
    vertices->push_back(hull.vertex(i));
  }
  auto faces = std::make_shared<std::vector<int>>(hull.face_data());
  return std::make_shared<fcl::Convexd>(std::move(vertices),
                                        hull.num_elements(), std::move(faces));
}

}  // namespace

std::unique_ptr<fcl::CollisionObjectd> MakeFclObject(
    const Shape& shape, GeometryId id, bool is_dynamic,
    const math::RigidTransformd& X_WG) {
  struct FclObjectMaker final : public ShapeReifier {
    using ShapeReifier::ImplementGeometry;
    void ImplementGeometry(const Box& box, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Boxd>(box.size());
    }
    void ImplementGeometry(const Capsule& capsule, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Capsuled>(capsule.radius(), capsule.length());
    }
    void ImplementGeometry(const Convex& convex, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          MakeConvex(convex);
    }
    void ImplementGeometry(const Cylinder& cylinder, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Cylinderd>(cylinder.radius(),
                                           cylinder.length());
    }
    void ImplementGeometry(const HalfSpace&, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    }
    void ImplementGeometry(const Sphere& sphere, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Sphered>(sphere.radius());
    }
  };
  FclObjectMaker maker;
  std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry;
  shape.Reify(&maker, &fcl_geometry);
  auto object = std::make_unique<fcl::CollisionObjectd>(fcl_geometry);
  EncodedData(id, is_dynamic).write_to(object.get());
  object->setTransform(X_WG.GetAsIsometry3());
  return object;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
