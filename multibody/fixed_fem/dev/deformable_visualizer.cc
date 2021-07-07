#include "drake/multibody/fixed_fem/dev/deformable_visualizer.h"

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <utility>

#include "drake/experimental_lcmt_deformable_tri.hpp"
#include "drake/experimental_lcmt_deformable_tri_mesh_init.hpp"
#include "drake/experimental_lcmt_deformable_tri_mesh_update.hpp"
#include "drake/experimental_lcmt_deformable_tri_meshes_init.hpp"
#include "drake/experimental_lcmt_deformable_tri_meshes_update.hpp"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace multibody {
namespace fem {

using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::internal::SortedTriplet;
using std::array;
using std::map;
using std::set;
using std::vector;
using systems::Context;
using systems::EventStatus;

DeformableVisualizer::DeformableVisualizer(
    double update_period, vector<std::string> mesh_names,
    const vector<VolumeMesh<double>>& tet_meshes, lcm::DrakeLcmInterface* lcm)
    : lcm_(lcm), mesh_names_(std::move(mesh_names)) {
  if (lcm == nullptr) {
    owned_lcm_ = std::make_unique<lcm::DrakeLcm>();
    lcm_ = owned_lcm_.get();
  }

  AnalyzeTets(tet_meshes);

  vertex_positions_port_ =
      this->DeclareAbstractInputPort("vertex_positions",
                                     Value<vector<VectorX<double>>>())
          .get_index();
  this->DeclareInitializationPublishEvent(
      &DeformableVisualizer::PublishMeshInit);
  this->DeclarePeriodicPublishEvent(update_period, 0.0,
                                    &DeformableVisualizer::PublishMeshUpdate);
}

void DeformableVisualizer::AnalyzeTets(
    const vector<VolumeMesh<double>>& tet_meshes) {
  /* For each tet mesh, extract all the border faces. Those are the triangles
   that are only referenced by a single face. So, for every tet, we examine its
   four faces and determine if any other tet shares it. Any face that is only
   referenced once is a border face.

   Each face has a unique key: a SortedTriplet (so the ordering of the vertices
   won't matter). The first time we see a face, we add it to a map. The second
   time we see the face, we remove it. When we're done, the keys in the map
   will be those faces only referenced once.

   The values in the map represent the triangle, with the vertices ordered so
   that they point *out* of the tetrahedron. Therefore, they will also point
   outside of the mesh.

   A typical tetrahedral element looks like:
       p2 *
          |
          |
       p3 *---* p0
         /
        /
    p1 *
   The index order for a particular tetrahedron has the order [p0, p1, p2, p3].
   These local indices enumerate each of the tet faces with outward-pointing
   normals with respect to the right-hand rule.  */
  const array<array<int, 3>, 4> local_indices{
      {{{1, 0, 2}}, {{3, 0, 1}}, {{3, 1, 2}}, {{2, 0, 3}}}};

  volume_vertex_counts_.resize(tet_meshes.size());
  surface_to_volume_vertices_.resize(tet_meshes.size());
  surface_triangles_.resize(tet_meshes.size());
  for (int i = 0; i < static_cast<int>(tet_meshes.size()); ++i) {
    /* While visiting all of the referenced vertices, identify the largest
     index.  */
    int largest_index = -1;
    const VolumeMesh<double>& tet_mesh = tet_meshes[i];
    map<SortedTriplet<int>, array<int, 3>> border_faces;
    for (const VolumeElement& tet : tet_mesh.tetrahedra()) {
      for (const array<int, 3>& tet_face : local_indices) {
        const array<int, 3> face{tet.vertex(tet_face[0]),
                                 tet.vertex(tet_face[1]),
                                 tet.vertex(tet_face[2])};
        largest_index = std::max({largest_index, face[0], face[1], face[2]});
        const SortedTriplet face_key(face[0], face[1], face[2]);
        if (auto itr = border_faces.find(face_key); itr != border_faces.end()) {
          border_faces.erase(itr);
        } else {
          border_faces[face_key] = face;
        }
      }
    }
    /* Record the expected minimum number of vertex positions to be received.
     For simplicity we choose a generous upper bound: the total number of
     vertices in the tetrahedral mesh, even though we really only need the
     positions of the vertices on the surface. */
    // TODO(xuchenhan-tri) It might be worthwhile to make largest_index the
    //  largest index that lies on the surface. Then, when we create our meshes,
    //  if we intentionally construct them so that the surface vertices come
    //  first, we will process a very compact representation.
    volume_vertex_counts_[i] = largest_index + 1;

    /* Using a set because the vertices will be nicely ordered. Ideally, we'll
     be extracting a subset of the vertex positions from the input port. We
     optimize cache coherency if we march in a monotonically increasing pattern.
     So, we'll map triangle vertex indices to volume vertex indices in a
     strictly monotonically increasing relationship.  */
    set<int> unique_vertices;
    for (const auto& [face_key, face] : border_faces) {
      unused(face_key);
      for (int j = 0; j < 3; ++j) unique_vertices.insert(face[j]);
    }
    /* This is the *second* documented responsibility of this function: populate
     the mapping from surface to volume so that we can efficiently extract the
     *surface* vertex positions from the *volume* vertex input.  */
    surface_to_volume_vertices_[i].clear();  // just to be safe.
    surface_to_volume_vertices_[i].insert(
        surface_to_volume_vertices_[i].begin(), unique_vertices.begin(),
        unique_vertices.end());

    /* The border faces all include indices into the volume vertices. To turn
     them into surface triangles, they need to include indices into the surface
     vertices. Create the volume index --> surface map to facilitate the
     transformation.  */
    const int surface_vertex_count =
        static_cast<int>(surface_to_volume_vertices_[i].size());
    map<int, int> volume_to_surface;
    for (int j = 0; j < surface_vertex_count; ++j) {
      volume_to_surface[surface_to_volume_vertices_[i][j]] = j;
    }

    /* This is the *first* documented responsibility: create the topology of the
     surface triangle mesh for each volume mesh. Each triangle consists of three
     indices into the set of *surface* vertex positions.  */
    surface_triangles_[i].clear();
    surface_triangles_[i].reserve(border_faces.size());
    for (auto& [face_key, face] : border_faces) {
      unused(face_key);
      surface_triangles_[i].emplace_back(volume_to_surface[face[0]],
                                         volume_to_surface[face[1]],
                                         volume_to_surface[face[2]]);
    }
  }
}

void DeformableVisualizer::SendMeshInit() const {
  experimental_lcmt_deformable_tri_meshes_init message;
  message.num_meshes = static_cast<int>(surface_to_volume_vertices_.size());
  message.meshes.resize(message.num_meshes);
  for (int i = 0; i < message.num_meshes; ++i) {
    auto& mesh = message.meshes[i];
    mesh.name = mesh_names_[i];
    mesh.num_vertices = static_cast<int>(surface_to_volume_vertices_[i].size());
    // TODO(xuchenhan-tri): Consider flatten the message hierarchy by sending 3T
    //  indices instead of T triangles.
    mesh.num_tris = static_cast<int>(surface_triangles_[i].size());
    mesh.tris.resize(mesh.num_tris);
    for (int t = 0; t < message.meshes[i].num_tris; ++t) {
      mesh.tris[t].vertices[0] = surface_triangles_[i][t](0);
      mesh.tris[t].vertices[1] = surface_triangles_[i][t](1);
      mesh.tris[t].vertices[2] = surface_triangles_[i][t](2);
    }
  }

  lcm::Publish(lcm_, "DEFORMABLE_MESHES_INIT", message);
}

EventStatus DeformableVisualizer::PublishMeshInit(
    const Context<double>&) const {
  SendMeshInit();
  return EventStatus::Succeeded();
}

void DeformableVisualizer::PublishMeshUpdate(
    const Context<double>& context) const {
  experimental_lcmt_deformable_tri_meshes_update message;
  message.timestamp =
      static_cast<int64_t>(ExtractDoubleOrThrow(context.get_time()) * 1e6);

  const int num_meshes = static_cast<int>(surface_to_volume_vertices_.size());
  // The volume vertex positions are one flat vector. Such that the position
  // of the i-th volume vertex is in entries j, j + 1, and j + 2, j = 3i.
  const vector<VectorX<double>>& vertex_states =
      vertex_positions_input_port().Eval<vector<VectorX<double>>>(context);
  vector<int> vertex_index_offsets{0};
  message.num_meshes = vertex_states.size();
  message.meshes.resize(message.num_meshes);
  for (int i = 0; i < num_meshes; ++i) {
    if (vertex_states[i].size() < volume_vertex_counts_[i]) {
      throw std::logic_error(fmt::format(
          "For mesh {}, named '{}', The number of given vertex positions "
          "({}) is smaller than the "
          "minimum expected number of positions ({}).",
          i, mesh_names_[i], vertex_states[i].size(),
          volume_vertex_counts_[i]));
    }
    auto& mesh = message.meshes[i];
    mesh.name = mesh_names_[i];
    const int v_count = static_cast<int>(surface_to_volume_vertices_[i].size());
    mesh.num_vertices = v_count;
    mesh.vertices_W.resize(mesh.num_vertices);
    for (int v = 0; v < v_count; ++v) {
      const int state_index = (surface_to_volume_vertices_[i][v]) * 3;
      mesh.vertices_W[v].resize(3);
      for (int d = 0; d < 3; ++d) {
        mesh.vertices_W[v][d] = vertex_states[i](state_index + d);
      }
    }
  }
  lcm::Publish(lcm_, "DEFORMABLE_MESHES_UPDATE", message);
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
