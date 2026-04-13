#include "drake/geometry/proximity/test/fcl_utilities.h"

#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<fcl::CollisionObjectd> MakeFclObject(const Shape& shape,
                                                     GeometryId id,
                                                     bool is_dynamic) {
  struct FclObjectMaker final : public ShapeReifier {
    using ShapeReifier::ImplementGeometry;
    void ImplementGeometry(const Sphere& sphere, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Sphered>(sphere.radius());
    }
    void ImplementGeometry(const Box& box, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Boxd>(box.size());
    }
    void ImplementGeometry(const Capsule& capsule, void* data) override {
      *static_cast<std::shared_ptr<fcl::CollisionGeometryd>*>(data) =
          std::make_shared<fcl::Capsuled>(capsule.radius(), capsule.length());
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
  };
  FclObjectMaker maker;
  std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry;
  shape.Reify(&maker, &fcl_geometry);
  auto object = std::make_unique<fcl::CollisionObjectd>(fcl_geometry);
  EncodedData(id, is_dynamic).write_to(object.get());
  return object;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
