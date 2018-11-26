#include "drake/automotive/maliput/utility/mesh_simplification.h"

#include <algorithm>
#include <queue>

namespace drake {
namespace maliput {
namespace utility {
namespace mesh {

InverseFaceEdgeMap ComputeInverseFaceEdgeMap(
    const std::vector<IndexFace>& faces) {
  InverseFaceEdgeMap inverse_face_edge_map;
  const int faces_count = static_cast<int>(faces.size());
  for (int face_index = 0; face_index < faces_count; ++face_index) {
    const IndexFace& face = faces[face_index];
    const std::vector<IndexFace::Vertex>& face_vertices = face.vertices();
    const int face_vertex_count = static_cast<int>(face_vertices.size());
    for (int start_vertex_index = 0; start_vertex_index < face_vertex_count;
         ++start_vertex_index) {
      int end_vertex_index = (start_vertex_index + 1) % face_vertex_count;
      const DirectedEdgeIndex global_edge{
          face_vertices[start_vertex_index].vertex_index,
          face_vertices[end_vertex_index].vertex_index};
      DRAKE_DEMAND(inverse_face_edge_map.count(global_edge) == 0);
      inverse_face_edge_map[global_edge] = {face_index, start_vertex_index};
    }
  }
  return inverse_face_edge_map;
}

FaceAdjacencyMap ComputeFaceAdjacencyMap(const std::vector<IndexFace>& faces) {
  FaceAdjacencyMap adjacent_faces_map;

  InverseFaceEdgeMap inverse_face_edge_map = ComputeInverseFaceEdgeMap(faces);
  for (const auto& entry : inverse_face_edge_map) {
    const DirectedEdgeIndex& edge_index = entry.first;
    const FaceEdgeIndex& face_edge_index = entry.second;
    if (adjacent_faces_map.count(face_edge_index.face_index) == 0) {
      adjacent_faces_map[face_edge_index.face_index].resize(
          faces[face_edge_index.face_index].vertices().size());
    }
    const DirectedEdgeIndex reversed_edge_index = edge_index.reverse();
    if (inverse_face_edge_map.count(reversed_edge_index) > 0) {
      std::vector<FaceEdgeIndex>& adjacent_face_edges =
          adjacent_faces_map[face_edge_index.face_index];
      adjacent_face_edges[face_edge_index.edge_index] =
          inverse_face_edge_map[reversed_edge_index];
    }
  }
  return adjacent_faces_map;
}

const Vector3<double>& GetMeshFaceVertexPosition(
    const GeoMesh& mesh, const IndexFace::Vertex& vertex) {
  return mesh.vertices().at(vertex.vertex_index)->v().xyz();
}

const Vector3<double>& GetMeshFaceVertexNormal(
    const GeoMesh& mesh, const IndexFace::Vertex& vertex) {
  return mesh.normals().at(vertex.normal_index)->n().xyz();
}

bool IsMeshFaceCoplanarWithPlane(const GeoMesh& mesh, const IndexFace& face,
                                 const Hyperplane3<double>& plane,
                                 double tolerance) {
  const std::vector<IndexFace::Vertex>& face_vertices = face.vertices();
  return DoMeshVerticesLieOnPlane(mesh, face_vertices.begin(),
                                  face_vertices.end(), plane, tolerance);
}

bool IsMeshFacePlanar(const GeoMesh& mesh, const IndexFace& face,
                      double tolerance, Hyperplane3<double>* plane) {
  DRAKE_DEMAND(plane != nullptr);
  const std::vector<IndexFace::Vertex>& face_vertices = face.vertices();
  DRAKE_DEMAND(face_vertices.size() >= 3);
  const Vector3<double>& x0 = GetMeshFaceVertexPosition(mesh, face_vertices[0]);
  const Vector3<double>& n0 = GetMeshFaceVertexNormal(mesh, face_vertices[0]);
  *plane = Hyperplane3<double>(n0.normalized(), x0);
  return DoMeshVerticesLieOnPlane(mesh, face_vertices.begin() + 1,
                                  face_vertices.end(), *plane, tolerance);
}

std::set<int> AggregateAdjacentCoplanarMeshFaces(
    const GeoMesh& mesh, int start_face_index,
    const FaceAdjacencyMap& adjacent_faces_map, double tolerance,
    std::set<int>* visited_faces_indices) {
  DRAKE_DEMAND(0 <= start_face_index);
  const std::vector<IndexFace>& faces = mesh.faces();
  DRAKE_DEMAND(start_face_index < static_cast<int>(faces.size()));
  DRAKE_DEMAND(tolerance > 0.);
  DRAKE_DEMAND(visited_faces_indices != nullptr);
  DRAKE_DEMAND(visited_faces_indices->count(start_face_index) == 0);

  // Traverse adjacent faces, collecting coplanar ones.
  std::set<int> mergeable_faces_indices{start_face_index};
  Hyperplane3<double> start_face_plane;
  if (IsMeshFacePlanar(mesh, faces[start_face_index], tolerance,
                       &start_face_plane)) {
    std::queue<int> faces_indices_to_visit({start_face_index});
    while (!faces_indices_to_visit.empty()) {
      int face_index = faces_indices_to_visit.front();
      const std::vector<FaceEdgeIndex>& adjacent_face_edges =
          adjacent_faces_map.at(face_index);
      const int face_edge_count = adjacent_face_edges.size();
      for (int edge_index = 0; edge_index < face_edge_count; ++edge_index) {
        const int adjacent_face_index =
            adjacent_face_edges[edge_index].face_index;
        if (adjacent_face_index == -1) continue;
        if (mergeable_faces_indices.count(adjacent_face_index) > 0) {
          continue;
        }
        if (visited_faces_indices->count(adjacent_face_index) > 0) {
          continue;
        }
        if (IsMeshFaceCoplanarWithPlane(mesh, faces[adjacent_face_index],
                                        start_face_plane, tolerance)) {
          mergeable_faces_indices.insert(adjacent_face_index);
          faces_indices_to_visit.push(adjacent_face_index);
        }
      }
      visited_faces_indices->insert(face_index);
      faces_indices_to_visit.pop();
    }
  } else {
    visited_faces_indices->insert(start_face_index);
  }
  return mergeable_faces_indices;
}

FaceEdgeIndex FindOuterFaceEdgeIndex(
    const std::set<int>& simply_connected_faces_indices,
    const FaceAdjacencyMap& adjacent_faces_map) {
  // Goes over each face in the simply connected region, looking
  // for the first face edge that's not adjacent to a face within
  // the same region.
  for (int face_index : simply_connected_faces_indices) {
    const std::vector<FaceEdgeIndex>& adjacent_face_edges =
        adjacent_faces_map.at(face_index);
    auto it = std::find_if(
        adjacent_face_edges.begin(), adjacent_face_edges.end(),
        [&simply_connected_faces_indices](const FaceEdgeIndex& face_edge) {
          return (simply_connected_faces_indices.count(face_edge.face_index) ==
                  0);
        });
    if (it != adjacent_face_edges.end()) {
      const int edge_index = std::distance(adjacent_face_edges.begin(), it);
      return {face_index, edge_index};
    }
  }
  return FaceEdgeIndex();
}

std::vector<FaceVertexIndex> ComputeMeshFacesContour(
    const std::set<int>& simply_connected_faces_indices,
    const FaceAdjacencyMap& adjacent_faces_map) {
  std::vector<FaceVertexIndex> faces_contour_indices;
  // Finds first outer edge index for the given simply connected region.
  const FaceEdgeIndex first_outer_face_edge_index = FindOuterFaceEdgeIndex(
      simply_connected_faces_indices, adjacent_faces_map);
  DRAKE_DEMAND(simply_connected_faces_indices.count(
                   first_outer_face_edge_index.face_index) > 0);

  // Follows simply connected region contour by crawling through
  // adjacent faces.
  FaceEdgeIndex outer_face_edge_index = first_outer_face_edge_index;
  do {
    const std::vector<FaceEdgeIndex>& adjacent_outer_face_edges =
        adjacent_faces_map.at(outer_face_edge_index.face_index);
    const FaceEdgeIndex& other_face_edge_index =
        adjacent_outer_face_edges[outer_face_edge_index.edge_index];
    if (simply_connected_faces_indices.count(
            other_face_edge_index.face_index) == 0) {
      faces_contour_indices.push_back(
          {outer_face_edge_index.face_index, outer_face_edge_index.edge_index});
    } else {
      outer_face_edge_index = other_face_edge_index;
    }
    const int outer_face_edge_count =
        adjacent_faces_map.at(outer_face_edge_index.face_index).size();
    outer_face_edge_index.edge_index =
        (outer_face_edge_index.edge_index + 1) % outer_face_edge_count;
  } while (outer_face_edge_index != first_outer_face_edge_index);
  return faces_contour_indices;
}

const IndexFace::Vertex& MeshFaceVertexAt(
    const GeoMesh& mesh, const FaceVertexIndex& face_vertex_index) {
  const std::vector<IndexFace>& faces = mesh.faces();
  const std::vector<IndexFace::Vertex>& face_vertices =
      faces.at(face_vertex_index.face_index).vertices();
  return face_vertices.at(face_vertex_index.vertex_index);
}

template <typename T>
using ParametrizedLine3 = Eigen::ParametrizedLine<T, 3>;

std::vector<FaceVertexIndex> SimplifyMeshFacesContour(
    const GeoMesh& mesh, const std::vector<FaceVertexIndex>& contour_indices,
    double tolerance) {
  const int contour_vertex_count = contour_indices.size();
  if (contour_vertex_count <= 3) return contour_indices;
  std::vector<FaceVertexIndex> simplified_contour_indices;
  ApplyDouglasPeuckerSimplification(
      contour_indices.begin(), contour_indices.end() - 1,
      [&mesh](const FaceVertexIndex& face_vertex_index) {
        return GetMeshFaceVertexPosition(
            mesh, MeshFaceVertexAt(mesh, face_vertex_index));
      },
      [](const Vector3<double>& start_vertex,
         const Vector3<double>& end_vertex) {
        return ParametrizedLine3<double>{
            start_vertex, (end_vertex - start_vertex).normalized()};
      },
      tolerance, std::back_inserter(simplified_contour_indices));
  return simplified_contour_indices;
}

GeoFace MergeMeshFaces(const GeoMesh& mesh,
                       const std::set<int>& mergeable_faces_indices,
                       const FaceAdjacencyMap& adjacent_faces_map,
                       double tolerance) {
  // Computes mergeable faces contour.
  const std::vector<FaceVertexIndex> contour_indices = SimplifyMeshFacesContour(
      mesh,
      ComputeMeshFacesContour(mergeable_faces_indices, adjacent_faces_map),
      tolerance);
  // Builds merged face by collecting all vertices in computed contour.
  GeoFace merged_face;
  const std::vector<const GeoVertex*>& vertices = mesh.vertices();
  const std::vector<const GeoNormal*>& normals = mesh.normals();
  for (const FaceVertexIndex& face_vertex_index : contour_indices) {
    const IndexFace::Vertex& face_vertex =
        MeshFaceVertexAt(mesh, face_vertex_index);
    merged_face.push_vn(*vertices[face_vertex.vertex_index],
                        *normals[face_vertex.normal_index]);
  }
  return merged_face;
}

GeoMesh SimplifyMeshFaces(const GeoMesh& input_mesh, double tolerance) {
  GeoMesh output_mesh;
  std::set<int> visited_faces_indices;
  const FaceAdjacencyMap adjacent_faces_map =
      ComputeFaceAdjacencyMap(input_mesh.faces());
  const int faces_count = static_cast<int>(input_mesh.faces().size());
  for (int face_index = 0; face_index < faces_count; ++face_index) {
    if (visited_faces_indices.count(face_index) > 0) {
      continue;
    }

    const std::set<int> mergeable_faces_indices =
        AggregateAdjacentCoplanarMeshFaces(input_mesh, face_index,
                                           adjacent_faces_map, tolerance,
                                           &visited_faces_indices);

    output_mesh.PushFace(MergeMeshFaces(input_mesh, mergeable_faces_indices,
                                        adjacent_faces_map, tolerance));
  }
  return output_mesh;
}

}  // namespace mesh
}  // namespace utility
}  // namespace maliput
}  // namespace drake
