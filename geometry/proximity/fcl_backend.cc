#include "drake/geometry/proximity/fcl_backend.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

std::string_view FclBackend::NodeTypeName(NodeType node_type) {
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake
