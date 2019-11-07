#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

std::string GetGeometryName(const fcl::CollisionObjectd& object) {
  switch (object.collisionGeometry()->getNodeType()) {
    case fcl::BV_UNKNOWN:
    case fcl::BV_AABB:
    case fcl::BV_OBB:
    case fcl::BV_RSS:
    case fcl::BV_kIOS:
    case fcl::BV_OBBRSS:
    case fcl::BV_KDOP16:
    case fcl::BV_KDOP18:
    case fcl::BV_KDOP24:
      return "Unsupported";
    case fcl::GEOM_BOX:
      return "Box";
    case fcl::GEOM_SPHERE:
      return "Sphere";
    case fcl::GEOM_ELLIPSOID:
      return "Ellipsoid";
    case fcl::GEOM_CAPSULE:
      return "Capsule";
    case fcl::GEOM_CONE:
      return "Cone";
    case fcl::GEOM_CYLINDER:
      return "Cylinder";
    case fcl::GEOM_CONVEX:
      return "Convex";
    case fcl::GEOM_PLANE:
      return "Plane";
    case fcl::GEOM_HALFSPACE:
      return "Halfspace";
    case fcl::GEOM_TRIANGLE:
      return "Mesh";
    case fcl::GEOM_OCTREE:
      return "Octtree";
    case fcl::NODE_COUNT:
      return "Unsupported";
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
