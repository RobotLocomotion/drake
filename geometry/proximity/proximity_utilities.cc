#include "drake/geometry/proximity/proximity_utilities.h"

#include <set>
#include <unordered_set>
#include <utility>

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
  std::unordered_set<SortedPair<VolumeVertexIndex>> edges;

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
  std::set<SortedTriplet<VolumeVertexIndex>> faces;

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

bool IsTetrahedronRespectingMa(
    const VolumeElement& tetrahedron, const VolumeMesh<double>& mesh,
    std::function<double(const Eigen::Vector3d&)> distance_to_boundary,
    std::function<double(int, const Eigen::Vector3d&)>
        distance_to_boundary_face,
    int num_faces, const double tolerance) {
  DRAKE_ASSERT(mesh.kVertexPerElement > 0);

  // For each vertex in `tetrahedron`, compute the set of "closest" faces.
  // A vertex lying on the medial axis has multiple "closest" faces (by virtue
  // of the definition of the medial axis.)
  std::vector<std::vector<int>> closest_faces(mesh.kVertexPerElement);
  for (int i = 0; i < mesh.kVertexPerElement; ++i) {
    const Eigen::Vector3d vertex = mesh.vertex(tetrahedron.vertex(i)).r_MV();
    const double vi_distance_to_boundary = distance_to_boundary(vertex);
    for (int j = 0; j < num_faces; ++j) {
      if (distance_to_boundary_face(j, vertex) - vi_distance_to_boundary <=
          tolerance) {
        closest_faces[i].push_back(j);
      }
    }
  }

  // Compute the intersection of the closest faces for all vertices.
  // If the intersection is empty, then there exist two vertices of this tet
  //   that belong to different blocks of the medial axis.
  // If the intersection is non-empty and has size > 1, then all vertices
  //   belong to the same facet of the medial axis. For a convex mesh, this
  //   facet is planar and therefore the tet has zero volume.
  // If the inetersection is non-empty and has size == 1, then the tet conforms
  //   to a single block of the media axis subdivision of the shape.
  std::vector<int> last_intersection = closest_faces[0];
  std::vector<int> curr_intersection;
  for (int i = 1; i < mesh.kVertexPerElement; ++i) {
    std::set_intersection(last_intersection.begin(), last_intersection.end(),
                          closest_faces[i].begin(), closest_faces[i].end(),
                          std::back_inserter(curr_intersection));
    std::swap(last_intersection, curr_intersection);
    curr_intersection.clear();
  }
  return last_intersection.size() == 1;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
