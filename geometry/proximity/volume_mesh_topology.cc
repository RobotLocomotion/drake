#include "drake/geometry/proximity/volume_mesh_topology.h"

#include <map>

namespace drake {
namespace geometry {
namespace internal {

VolumeMeshTopology::~VolumeMeshTopology() = default;

template <typename T>
VolumeMeshTopology::VolumeMeshTopology(const VolumeMesh<T>& mesh) {
  tetrahedra_neighbors_.resize(mesh.num_elements(),
                               std::array<int, 4>{-1, -1, -1, -1});

  // Get the cannonical representation of face f of element e.
  auto get_face = [](const VolumeElement& e, int f) {
    int a = e.vertex((f + 1) % 4);
    int b = e.vertex((f + 2) % 4);
    int c = e.vertex((f + 3) % 4);

    return SortedTriplet<int>(a, b, c);
  };

  // Maps faces to the first tet we encountered containing that tet.
  std::map<SortedTriplet<int>, int> face_to_tet;

  // Establish neighbors for all tets.
  for (int tet_index = 0; tet_index < mesh.num_elements(); ++tet_index) {
    const VolumeElement& e = mesh.element(tet_index);

    for (int face_index = 0; face_index < 4; ++face_index) {
      SortedTriplet<int> face = get_face(e, face_index);

      // If we've seen this face before, we now know the two tets that
      // neighbor on `face`.
      if (face_to_tet.contains(face)) {
        int neighbor_tet_index = face_to_tet.at(face);

        // Set the neighbor of the current tet at `face`'s index to the
        // neighbor tet index.
        tetrahedra_neighbors_[tet_index][face_index] = neighbor_tet_index;

        // Find the index of `face` in the neighbor tet.
        int matching_face_index = -1;
        for (int neighbor_face_index = 0; neighbor_face_index < 4;
             ++neighbor_face_index) {
          if (get_face(mesh.element(neighbor_tet_index), neighbor_face_index) ==
              face) {
            matching_face_index = neighbor_face_index;
            break;
          }
        }
        DRAKE_ASSERT(matching_face_index != -1);
        // Set the neighbor of the neighboring tet at `face`'s matching index
        // to the current tet index.
        tetrahedra_neighbors_[neighbor_tet_index][matching_face_index] =
            tet_index;

      } else {
        // We haven't seen `face` before, so just map it to the current tet.
        face_to_tet[face] = tet_index;
      }
    }
  }
}

template VolumeMeshTopology::VolumeMeshTopology<double>(
    const VolumeMesh<double>& mesh);
template VolumeMeshTopology::VolumeMeshTopology<AutoDiffXd>(
    const VolumeMesh<AutoDiffXd>& mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
