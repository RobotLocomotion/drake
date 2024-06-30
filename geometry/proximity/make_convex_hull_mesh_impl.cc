#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <fmt/format.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <vtkGLTFImporter.h>  // vtkIOImport
#include <vtkMapper.h>        // vtkCommonCore
#include <vtkMatrix4x4.h>     // vtkCommonMath
#include <vtkPolyData.h>      // vtkCommonDataModel
#include <vtkRenderer.h>      // vtkRenderingCore

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/ssize.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"
#include "drake/geometry/read_obj.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

#include <iostream>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;

namespace fs = std::filesystem;

/* Used for ordering polygon vertices according to their angular distance from
 a reference direction. See OrderPolyVertices(). */
struct VertexScore {
  double angle;
  int index;
};

/* Orders the vertices of a convex, planar polygon into a right-handed winding
 w.r.t. the plane's normal `n`.

 The polygon vertices are drawn from the set of `vertices` and indicated by the
 indices in `v_indices`.

 @param vertices  The set of vertices including the polygon's vertices.
 @param v_indices The indices of the vertices that bound the polygon.
 @param center    A point in the "center" of the polygon.
 @param n         The plane normal.

 @pre The polygon has at least three vertices.
 @pre The indicies in `v_indices` are all valid indices into `vertices`.
 @pre The polygon is convex.
 @pre The vertices and normals are expressed in the same basis.
 @pre the indexed vertices are truly all planar. */
std::vector<int> OrderPolyVertices(const std::vector<Vector3d>& vertices,
                                   const std::vector<int>& v_indices,
                                   const Vector3d& center, const Vector3d& n) {
  DRAKE_DEMAND(v_indices.size() >= 3);

  /* We arbitrarily define v0 to be the first vertex in the polygon. We will
   then order the other vertices such that they wind around the normal starting
   from v0. */

  /* Given the direction v_C0 (direction from center vertex to vertex 0), define
   the angle between v_C0 and v_CI for all vertices. We'll then sort the
   vertices on the angle and get a counter-clockwise winding. */
  const Vector3d v_C0 = (vertices[v_indices[0]] - center).normalized();
  std::vector<VertexScore> scored_vertices;
  scored_vertices.reserve(v_indices.size());
  for (int vi : v_indices) {
    const Vector3d& p_MI = vertices[vi];
    const Vector3d v_CI = (p_MI - center).normalized();
    scored_vertices.push_back(
        {std::atan2((v_C0.cross(v_CI).dot(n)), v_C0.dot(v_CI)), vi});
  }

  /* Sort by increasing angle. */
  std::sort(scored_vertices.begin(), scored_vertices.end(),
            [](const VertexScore& a, const VertexScore& b) {
              return a.angle < b.angle;
            });

  /* Construct the ordered polygon. */
  std::vector<int> ordered;
  ordered.reserve(v_indices.size());
  for (const VertexScore& vs : scored_vertices) {
    ordered.push_back(vs.index);
  }

  return ordered;
}

/* The point cloud defined by VertexReader. */
struct VertexCloud {
  /* The set of points, defined in some frame. */
  std::vector<Vector3d> vertices;
  /* Reports of the cloud is planar. */
  bool is_planar{false};
  /* If `is_planar` is `true`, the normal of the plane (otherwise undefined). */
  Vector3d n;
  /* If `is_planar` a point on the plane contained within the cloud's convex
   hull (otherwise undefined). */
  Vector3d interior_point;
};

/* Returns the scaled vertices from the named obj file.
 @pre `filename` references an obj file. */
void ReadObjVertices(const fs::path filename, double scale,
                     std::vector<Vector3d>* vertices) {
  const auto [tinyobj_vertices, _1, _2] = geometry::internal::ReadObjFile(
      std::string(filename), scale, /* triangulate = */ false);
  *vertices = std::move(*tinyobj_vertices);
}

/* Returns the scaled vertices from the named vtk file.
 @pre `filename` references a vtk file (with a volume mesh). */
void ReadVtkVertices(const fs::path filename, double scale,
                     std::vector<Vector3d>* vertices) {
  const VolumeMesh<double> volume_mesh =
      ReadVtkToVolumeMesh(std::string(filename), scale);

  // It would be nice if we could simply steal the vertices rather than copy.
  *vertices = volume_mesh.vertices();
}

/* Multiplies the position vector p_AQ by the transform T_BA, returning p_BQ. */
Vector3d VtkMultiply(vtkMatrix4x4* T_BA, const Vector3d& p_AQ) {
  double p_in[] = {p_AQ.x(), p_AQ.y(), p_AQ.z(), 1};
  double p_out[4];
  T_BA->MultiplyPoint(p_in, p_out);
  return Vector3d(p_out);
}

/* Returns the scaled vertices from the named glTF file.
 @pre `filename` references a glTF file. */
void ReadGltfVertices(const fs::path filename, double scale,
                      std::vector<Vector3d>* vertices) {
  vtkNew<vtkGLTFImporter> importer;
  importer->SetFileName(filename.c_str());
  importer->Update();

  auto* renderer = importer->GetRenderer();
  DRAKE_DEMAND(renderer != nullptr);

  if (renderer->VisibleActorCount() == 0) {
    throw std::runtime_error(
        fmt::format("MakeConvexHull() found no vertices in the file '{}'.",
                    filename.string()));
  }

  // The relative transform from the file's frame F to the geometry's frame G.
  // (rotation from y-up to z-up). The scale is handled separately.
  const RigidTransformd X_GF(RotationMatrixd::MakeXRotation(M_PI / 2));

  auto* actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor* actor = actors->GetNextActor()) {
    // 1. Extract PolyData from actor.
    auto* poly_data =
        dynamic_cast<vtkPolyData*>(actor->GetMapper()->GetInput());
    DRAKE_DEMAND(poly_data != nullptr);
    // 2. For each vertex, transform it to the file frame (based on the actors
    // user transform), and then into the geometry frame using the scale and
    // rotation.
    vtkMatrix4x4* T_FA = actor->GetUserMatrix();
    for (vtkIdType vi = 0; vi < poly_data->GetNumberOfPoints(); ++vi) {
      const Vector3d p_AV(poly_data->GetPoint(vi));
      const Vector3d p_FV = VtkMultiply(T_FA, p_AV);
      vertices->emplace_back((X_GF * p_FV) * scale);
    }
  }
}

/* Given a set of vertices, attempts to find a plane that reliably spans three
 vertices. The normal is defined as n = a x b / |a x b| where
   a = v[j] - v[0] and
   b = v[k] - v[0] for 0 ≠ j ≠ k.
 @throws if the vertices span a severely degenerate space in R3 (e.g.,
         co-linear or coincident). */
Vector3d FindNormal(const std::vector<Vector3d>& vertices,
                    const fs::path filename) {
  // Note: this isn't an exhaustive search. We assign i = 0 and then
  // sequentially search for j and k. This may fail but possibly succeed for
  // a different value of i. That risk seems small. Any mesh that depends on
  // that subtlety is so close to being degenerate, the user should revisit
  // their mesh.
  int v = 0;
  Vector3d a;
  for (; v < ssize(vertices); ++v) {
    a = vertices[v] - vertices[0];
    if (a.squaredNorm() >= 1e-24) {
      break;
    }
  }

  if (v == ssize(vertices)) {
    throw std::runtime_error(
        fmt::format("MakeConvexHull failed because all vertices in the mesh "
                    "were within a "
                    "sphere with radius 1e-12 for file: {}.",
                    filename.string()));
  }
  a.normalize();

  Vector3d n_candidate;
  // Continue the search through vertices for b.
  for (; v < ssize(vertices); ++v) {
    const Vector3d b = vertices[v] - vertices[0];
    n_candidate = a.cross(b);
    if (b.squaredNorm() >= 1e-24 && n_candidate.squaredNorm() >= 1e-24) {
      break;
    }
  }

  if (v == ssize(vertices)) {
    throw std::runtime_error(fmt::format(
        "MakeConvexHull failed because all vertices in the mesh appear to be "
        "co-linear for file: {}.",
        filename.string()));
  }
  return n_candidate.normalized();
}

/* Simply reads the vertices from an OBJ, VTK or glTF file referred to by name.
 */
VertexCloud ReadVertices(const fs::path filename, double scale) {
  std::string extension = filename.extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });

  VertexCloud cloud;
  if (extension == ".obj") {
    ReadObjVertices(filename, scale, &cloud.vertices);
  } else if (extension == ".vtk") {
    ReadVtkVertices(filename, scale, &cloud.vertices);
  } else if (extension == ".gltf") {
    ReadGltfVertices(filename, scale, &cloud.vertices);
  } else {
    throw std::runtime_error(
        fmt::format("MakeConvexHull only applies to obj, vtk, and gltf "
                    "meshes; given file: {}.",
                    filename.string()));
  }

  if (cloud.vertices.size() < 3) {
    throw std::runtime_error(
        fmt::format("MakeConvexHull() cannot be used on a mesh with fewer "
                    "than three vertices; found {} vertices in file: {}.",
                    cloud.vertices.size(), filename.string()));
  }

  /* Characterizes planarity. */
  cloud.n = FindNormal(cloud.vertices, filename);
  double d = cloud.n.dot(cloud.vertices[0]);
  cloud.interior_point = cloud.vertices[0];
  /* Assume planarity and look for evidence to the contrary. */
  cloud.is_planar = true;
  for (int vi = 1; vi < ssize(cloud.vertices); ++vi) {
    const Vector3d& v = cloud.vertices[vi];
    cloud.interior_point += v;
    const double dist = std::abs(cloud.n.dot(v) - d);
    if (dist > 1e-12) {
      cloud.is_planar = false;
      break;
    }
  }

  if (cloud.is_planar) {
    /* We define the interior point as simply the mean point. */
    cloud.interior_point /= ssize(cloud.vertices);
  }
  return cloud;
}

class ConvexHull {
 public:
  explicit ConvexHull(std::vector<Vector3d> v)
      : ConvexHull(VertexCloud{.vertices = std::move(v)}) {}

  explicit ConvexHull(VertexCloud cloud) : cloud_(std::move(cloud)) {
    if (cloud_.is_planar) {
      /* When the vertices are planar, we introduce a single point off the plane
       (which still projects into the cloud). Later, when we convert qhull's
       result into a mesh, we'll find the one vertex obviously off the plane and
       move it back onto the plane again. */
      cloud_.vertices.push_back(cloud_.interior_point + cloud_.n);
    }

    // This is a cheat that relies on the fact that a vector of Vector3d really
    // is just a vector<double> with three times as many entries. It *does*
    // eliminate a copy.
    static_assert(sizeof(Vector3d) == 3 * sizeof(double));
    qhull_.runQhull(/* inputComment = */ "", kDim, cloud_.vertices.size(),
                    reinterpret_cast<const double*>(cloud_.vertices.data()),
                    /* qhullCommand = */ "");

    if (qhull_.qhullStatus() != 0) {
      throw std::runtime_error(
          fmt::format("MakeConvexHull failed. Qhull terminated with status {} "
                      "and message:\n{}",
                      qhull_.qhullStatus(), qhull_.qhullMessage()));
    }
  }

  std::vector<Vector4d> GetHyperPlanes() const {
    std::vector<Vector4d> hyperplanes;
    hyperplanes.reserve(qhull_.facetCount());
    for (auto& facet : qhull_.facetList()) {
      // QHull doesn't necessarily order the vertices in the winding we want.
      const Vector3d normal(facet.hyperplane().coordinates());
      DRAKE_DEMAND(std::abs(normal.norm()-1.0) < 1.0e-14);
      const double d = -facet.hyperplane().offset();  // distance to origin.
      Vector4d h = (Vector4d() << normal, d).finished();
      DRAKE_DEMAND(d > 0);   // Demand a normal pointing away from the origin.
      //if (d < 0) h = -h;  // Ensure outwards normal.
      hyperplanes.push_back(h);
    }
    return hyperplanes;
  }

  PolygonSurfaceMesh<double> MakePolygonSurfaceMesh(
      const Vector3d& offset) const {
    // The default mapping from qhull to convex hull mesh is simply a copy.
    std::function<Vector3d(double, double, double)> map_hull_vertex =
        [](double x, double y, double z) {
          return Vector3d(x, y, z);
        };

    if (cloud_.is_planar) {
      /* When the vertices are planar, we introduce a single point off the plane
       (which still projects into the cloud). Later, when we convert qhull's
       result into a mesh, we'll find the one vertex obviously off the plane and
       move it back onto the plane again. */
      const double d = cloud_.n.dot(cloud_.interior_point);
      map_hull_vertex = [this, d](double x, double y, double z) {
        const Vector3d p(x, y, z);
        if (cloud_.n.dot(p) - d > 0.75) {
          // The only point that should measurably lie off the plane is the
          // interior point we perturbed. We'll put it back onto the plane.
          return cloud_.interior_point;
        }
        return p;
      };
    }

    // A mapping from qhull vertex ids to indices in the final mesh
    // (vertices_M).
    // Note: `countT` is the qhull type for mesh ids.
    std::map<countT, int> id_to_mesh_index;
    std::vector<Vector3d> vertices_M;
    vertices_M.reserve(qhull_.vertexCount());
    for (auto& vertex : qhull_.vertexList()) {
      const int index = ssize(vertices_M);
      id_to_mesh_index[vertex.id()] = index;
      const auto* p_MV = vertex.point().coordinates();
      vertices_M.push_back(map_hull_vertex(p_MV[0], p_MV[1], p_MV[2]));
    }

    std::vector<int> face_data;
    // Note: 4 * facet count will not generally be enough, but it's a safe
    // starting size.
    face_data.reserve(qhull_.facetCount() * 4);
    for (auto& facet : qhull_.facetList()) {
      auto qhull_vertices = facet.vertices().toStdVector();
      std::vector<int> mesh_indices;
      std::transform(qhull_vertices.cbegin(), qhull_vertices.cend(),
                     std::back_inserter(mesh_indices),
                     [&id_to_mesh_index](auto v) {
                       return id_to_mesh_index[v.id()];
                     });
      // QHull doesn't necessarily order the vertices in the winding we want.
      const Vector3d normal(facet.hyperplane().coordinates());
      const double distance = -facet.hyperplane().offset();
      DRAKE_DEMAND(distance >= 0);  // Normal away from origin.
      const Vector3d center(facet.getCenter().coordinates());
      std::vector<int> ordered_vertices =
          OrderPolyVertices(vertices_M, mesh_indices, center, normal);

      // Now populate the face data.
      face_data.push_back(ssize(ordered_vertices));
      face_data.insert(face_data.end(), ordered_vertices.begin(),
                       ordered_vertices.end());
    }

    // Apply offset.
    for (auto& p : vertices_M) {
      p += offset;
    }

    return PolygonSurfaceMesh(std::move(face_data), std::move(vertices_M));
  }

 private:
  static constexpr int kDim{3};
  VertexCloud cloud_;
  orgQhull::Qhull qhull_;
};

}  // namespace

PolygonSurfaceMesh<double> MakeConvexHull(const std::filesystem::path mesh_file,
                                          double scale, double margin) {
  VertexCloud cloud = ReadVertices(mesh_file, scale);
  const bool is_planar = cloud.is_planar;

  // When margin != 0, we apply an offset so that the origin is contained in the
  // convex hull. This is a mathematical pre-requisite to work with the dual
  // below. We'll remove the offset on the output mesh.
  Vector3d offset = Vector3d::Zero();
  if (!is_planar) { // && margin != 0
    // Compute offset.
    for (const auto& p : cloud.vertices) {
      offset += p;
    }
    offset /= ssize(cloud.vertices);

    // Apply offset.
    for (auto& p : cloud.vertices) {
      p -= offset;
    }
  }

  ConvexHull hull(std::move(cloud));

  // We do not apply margin to planar clouds.
  if (is_planar) { // || margin == 0
    return hull.MakePolygonSurfaceMesh(offset);
  }

  // Vertices in the dual of the (inflated) original geometry.
  const std::vector<Vector4d> h = hull.GetHyperPlanes();
  std::vector<Vector3d> v_dual;
  v_dual.reserve(h.size());
  for (const Vector4d& hi : h) {
    const auto ni = hi.head<3>();  // normal
    const double di = hi(3);       // distance to origin.
    DRAKE_DEMAND(di > 0);
    const Vector3d v = ni / (di + margin);
    v_dual.push_back(v);
  }

  // Convex hull of the dual.
  ConvexHull dual_hull(std::move(v_dual));

  // Vertices of the inflated geometry.
  const std::vector<Vector4d> h_dual = dual_hull.GetHyperPlanes();
  std::vector<Vector3d> v_inflated;
  v_inflated.reserve(h_dual.size());
  for (const Vector4d& hi : h_dual) {
    const auto ni = hi.head<3>();  // normal
    const double di = hi(3);       // distance to origin.
    DRAKE_DEMAND(di > 0);
    const Vector3d v = ni / di;
    v_inflated.push_back(v);
  }

  // Remove offset.
  //for (auto& p : v_inflated) {
  //  p += offset;
  //}

  // Convex hull of the inflated geometry.
  ConvexHull inflated_hull(std::move(v_inflated));

  return inflated_hull.MakePolygonSurfaceMesh(offset);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
