#include "drake/geometry/proximity/proximity_utilities.h"

#include <set>
#include <unordered_set>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"

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

int CountEdges(const VolumeMesh<double>& mesh) {
  std::unordered_set<SortedPair<int>> edges;

  for (auto& t : mesh.tetrahedra()) {
    // 6 edges of a tetrahedron
    edges.emplace(t.vertex(0), t.vertex(1));
    edges.emplace(t.vertex(1), t.vertex(2));
    edges.emplace(t.vertex(0), t.vertex(2));
    edges.emplace(t.vertex(0), t.vertex(3));
    edges.emplace(t.vertex(1), t.vertex(3));
    edges.emplace(t.vertex(2), t.vertex(3));
  }
  return edges.size();
}

int CountFaces(const VolumeMesh<double>& mesh) {
  std::set<SortedTriplet<int>> faces;

  for (const auto& t : mesh.tetrahedra()) {
    // 4 faces of a tetrahedron, all facing in
    faces.emplace(t.vertex(0), t.vertex(1), t.vertex(2));
    faces.emplace(t.vertex(1), t.vertex(0), t.vertex(3));
    faces.emplace(t.vertex(2), t.vertex(1), t.vertex(3));
    faces.emplace(t.vertex(0), t.vertex(2), t.vertex(3));
  }

  return faces.size();
}

int ComputeEulerCharacteristic(const VolumeMesh<double>& mesh) {
  const int k0 = mesh.vertices().size();
  const int k1 = CountEdges(mesh);
  const int k2 = CountFaces(mesh);
  const int k3 = mesh.tetrahedra().size();

  return k0 - k1 + k2 - k3;
}

double CalcDistanceToSurface(const Capsule& capsule, const Vector3d& p_CP) {
  const double half_length = capsule.length() / 2;
  const double z = std::clamp(p_CP.z(), -half_length, half_length);
  const Vector3d p_CQ(0, 0, z);
  return (p_CQ - p_CP).norm() - capsule.radius();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
