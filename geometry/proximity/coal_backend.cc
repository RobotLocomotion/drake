#include "drake/geometry/proximity/coal_backend.h"

#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

CoalBackend::FaceData CoalBackend::MakeFaces(
    const std::vector<int>& face_data) {
  FaceData triangles;
  const int size = ssize(face_data);
  int i = 0;
  while (i < size) {
    const int count = face_data[i];
    DRAKE_DEMAND(count >= 3);
    DRAKE_DEMAND(i + count < size);
    const int v0 = face_data[i + 1];
    // Fan the n-gon around its first vertex: (v0, v1, v2), (v0, v2, v3), ...
    // This preserves the winding, and therefore the outward normal.
    for (int t = 1; t + 1 < count; ++t) {
      triangles.emplace_back(v0, face_data[i + 1 + t], face_data[i + 2 + t]);
    }
    i += count + 1;
  }
  return triangles;
}

ShapeKind CoalBackend::Classify(NodeType node_type) {
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake
