#include "drake/geometry/proximity/test/backend_utilities.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/fcl_backend.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

template <typename Backend>
std::shared_ptr<typename Backend::Convex> MakeConvex(const Convex& convex) {
  const PolygonSurfaceMesh<double>& hull = convex.GetConvexHull();
  auto vertices = std::make_shared<std::vector<Vector3d>>();
  vertices->reserve(hull.num_vertices());
  for (int i = 0; i < hull.num_vertices(); ++i) {
    vertices->push_back(hull.vertex(i));
  }
  auto faces = std::make_shared<typename Backend::FaceData>(
      Backend::MakeFaces(hull.face_data()));
  const int num_faces = Backend::CountFaces(*faces, hull.num_elements());
  return Backend::MakeConvex(std::move(vertices), hull.num_vertices(),
                             std::move(faces), num_faces);
}

}  // namespace

template <typename Backend>
std::unique_ptr<typename Backend::Object> MakeBackendObject(
    const Shape& shape, GeometryId id, bool is_dynamic,
    const math::RigidTransformd& X_WG) {
  struct ObjectMaker final : public ShapeReifier {
    using ShapeReifier::ImplementGeometry;
    using GeometryPtr = std::shared_ptr<typename Backend::Geometry>;
    void ImplementGeometry(const Box& box, void* data) override {
      *static_cast<GeometryPtr*>(data) =
          std::make_shared<typename Backend::Box>(box.size());
    }
    void ImplementGeometry(const Capsule& capsule, void* data) override {
      *static_cast<GeometryPtr*>(data) =
          std::make_shared<typename Backend::Capsule>(capsule.radius(),
                                                      capsule.length());
    }
    void ImplementGeometry(const Convex& convex, void* data) override {
      *static_cast<GeometryPtr*>(data) = MakeConvex<Backend>(convex);
    }
    void ImplementGeometry(const Cylinder& cylinder, void* data) override {
      *static_cast<GeometryPtr*>(data) =
          std::make_shared<typename Backend::Cylinder>(cylinder.radius(),
                                                       cylinder.length());
    }
    void ImplementGeometry(const HalfSpace&, void* data) override {
      *static_cast<GeometryPtr*>(data) =
          std::make_shared<typename Backend::Halfspace>(0, 0, 1, 0);
    }
    void ImplementGeometry(const Sphere& sphere, void* data) override {
      *static_cast<GeometryPtr*>(data) =
          std::make_shared<typename Backend::Sphere>(sphere.radius());
    }
  };
  ObjectMaker maker;
  std::shared_ptr<typename Backend::Geometry> geometry;
  shape.Reify(&maker, &geometry);
  auto object = std::make_unique<typename Backend::Object>(geometry);
  EncodedData(id, is_dynamic).write_to(object.get());
  object->setTransform(Backend::ToTransform(X_WG));
  return object;
}

template std::unique_ptr<FclBackend::Object> MakeBackendObject<FclBackend>(
    const Shape&, GeometryId, bool, const math::RigidTransformd&);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
