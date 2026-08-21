#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"

#include <algorithm>
#include <map>
#include <memory>
#include <stack>
#include <string>
#include <utility>
#include <vector>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <fmt/format.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <vtkGLTFDocumentLoader.h>  // vtkIOGeometry
#include <vtkMapper.h>              // vtkCommonCore
#include <vtkMatrix4x4.h>           // vtkCommonMath
#include <vtkPolyData.h>            // vtkCommonDataModel
#include <vtkRenderer.h>            // vtkRenderingCore

#include "drake/common/diagnostic_policy.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/fmt_ostream.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"
#include "drake/geometry/read_gltf_to_memory.h"
#include "drake/geometry/read_obj.h"
#include "drake/geometry/vtk_gltf_uri_loader.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/vtk_diagnostic_event_observer.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;
using systems::sensors::internal::VtkDiagnosticEventObserver;

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
  /* Mean vertex point, guaranteed to be within the convex hull. */
  Vector3d interior_point;
};

/* Returns the scaled vertices from the contents of an obj file.
 @pre `obj_source` contains obj geometry data. */
std::vector<Vector3d> ReadObjVertices(const MeshSource& obj_source,
                                      const Vector3d& scale) {
  const auto [tinyobj_vertices_ptr, _1, _2] =
      geometry::internal::ReadObj(obj_source, scale,
                                  /* triangulate = */ false,
                                  /* vertices_only = */ true);
  return std::move(*tinyobj_vertices_ptr);
}

/* Returns the scaled vertices from the contents of a vtk file.
 @pre `vtk_source` contains vtk geometry data. */
std::vector<Vector3d> ReadVtkVertices(const MeshSource& vtk_source,
                                      const Vector3d& scale) {
  const VolumeMesh<double> volume_mesh = ReadVtkToVolumeMesh(vtk_source, scale);

  // It would be nice if we could simply steal the vertices rather than copy.
  return volume_mesh.vertices();
}

/* Multiplies the position vector p_AQ by the transform T_BA, returning p_BQ. */
Vector3d VtkMultiply(vtkMatrix4x4* T_BA, const Vector3d& p_AQ) {
  double p_in[] = {p_AQ.x(), p_AQ.y(), p_AQ.z(), 1};
  double p_out[4];
  T_BA->MultiplyPoint(p_in, p_out);
  return Vector3d(p_out);
}

/* Returns the scaled vertices from the contents of a glTF file.
 @pre `gltf_source` contains glTF geometry data.
 @pre `gltf_source.is_path()` is `true`. */
std::vector<Vector3d> ReadGltfVertices(const MeshSource& gltf_source,
                                       const Vector3d& scale) {
  vtkNew<vtkGLTFDocumentLoader> loader;
  loader->SetLoadAnimation(false);
  loader->SetLoadImages(false);
  loader->SetLoadSkinMatrix(false);
  vtkNew<VtkDiagnosticEventObserver> observer;
  DiagnosticPolicy diagnostic;
  observer->set_diagnostic(&diagnostic);
  loader->AddObserver(vtkCommand::ErrorEvent, observer);
  loader->AddObserver(vtkCommand::WarningEvent, observer);

  vtkNew<VtkGltfUriLoader> uri_loader;
  uri_loader->SetMeshSource(&gltf_source);
  vtkSmartPointer<vtkResourceStream> gltf_stream = uri_loader->MakeGltfStream();
  bool success = true;
  success =
      success && loader->LoadModelMetaDataFromStream(gltf_stream, uri_loader);
  success = success && loader->LoadModelData({});
  success = success && loader->BuildModelVTKGeometry();
  if (!success) {
    // Note: we don't generally expect this exception to be thrown. We expect
    // the event observer will convert VTK errors to thrown exceptions for us.
    // This is here on the off chance that VTK returns failure on any of the
    // calls above without registering an error with the observer.
    throw std::runtime_error(
        fmt::format("Can't compute convex hull for in-memory glTF file '{}'. "
                    "Unknown VTK error.",
                    gltf_source.description()));
  }

  // Now get the vertex data, transformed to the geometry frame G.
  std::shared_ptr<vtkGLTFDocumentLoader::Model> model =
      loader->GetInternalModel();
  DRAKE_DEMAND(model != nullptr);

  // TODO(SeanCurtis-TRI): This should use the same scene logic as found in
  // GltfMeshExtractor::FindTargetRootNodes() to unify the scene/node semantics.
  std::stack<int> nodes;
  for (int node_id : model->Scenes[model->DefaultScene].Nodes) {
    nodes.push(node_id);
  }

  // The relative transform from the file's frame F to the geometry's frame G.
  // (rotation from y-up to z-up). The scale is handled separately.
  const RigidTransformd X_GF(RotationMatrixd::MakeXRotation(M_PI / 2));

  std::vector<Vector3d> vertices;
  while (!nodes.empty()) {
    const int node_id = nodes.top();
    nodes.pop();
    const auto& node = model->Nodes[node_id];

    // Transform of the node in the file.
    vtkMatrix4x4* T_FN = node.GlobalTransform;
    if (node.Mesh >= 0) {
      const auto& mesh = model->Meshes[node.Mesh];
      for (const auto& primitive : mesh.Primitives) {
        vtkSmartPointer<vtkPolyData> point_data = primitive.Geometry;
        if (point_data == nullptr) {
          continue;
        }
        for (vtkIdType vi = 0; vi < point_data->GetNumberOfPoints(); ++vi) {
          const Vector3d p_NV(point_data->GetPoint(vi));
          const Vector3d p_FV = VtkMultiply(T_FN, p_NV);
          vertices.emplace_back(scale.cwiseProduct(X_GF * p_FV));
        }
      }
    }
    for (int child_node_id : node.Children) {
      // This is a *tree* not a graph, so we won't be revisiting nodes.
      nodes.push(child_node_id);
    }
  }
  return vertices;
}

/* Given a set of vertices, attempts to find a plane that reliably spans three
 vertices. The normal is defined as n = a x b / |a x b| where
   a = v[j] - v[0] and
   b = v[k] - v[0] for 0 ≠ j ≠ k.
 @throws if the vertices span a severely degenerate space in R3 (e.g.,
         co-linear or coincident). */
Vector3d FindNormal(const std::vector<Vector3d>& vertices,
                    const std::string_view description) {
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
                    "were within a sphere with radius 1e-12 for geometry: {}.",
                    description));
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
        description));
  }
  return n_candidate.normalized();
}

/* Simply reads the vertices from an OBJ, VTK or glTF file indicated by the
 given `mesh_source`. */
VertexCloud ReadVertices(const MeshSource& mesh_source, const Vector3d& scale) {
  VertexCloud cloud;
  if (mesh_source.extension() == ".obj") {
    cloud.vertices = ReadObjVertices(mesh_source, scale);
  } else if (mesh_source.extension() == ".vtk") {
    cloud.vertices = ReadVtkVertices(mesh_source, scale);
  } else if (mesh_source.extension() == ".gltf") {
    cloud.vertices = ReadGltfVertices(mesh_source, scale);
  } else {
    throw std::runtime_error(
        fmt::format("MakeConvexHull only applies to .obj, .vtk, and .gltf "
                    "meshes; unsupported extension '{}' for geometry data: {}.",
                    mesh_source.extension(), mesh_source.description()));
  }

  if (cloud.vertices.size() < 3) {
    throw std::runtime_error(fmt::format(
        "MakeConvexHull() cannot be used on a mesh with fewer "
        "than three vertices; found {} vertices in geometry data: {}.",
        cloud.vertices.size(), mesh_source.description()));
  }

  /* Characterizes planarity. */
  cloud.n = FindNormal(cloud.vertices, mesh_source.description());
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
    }
  }

  /* We define the interior point as simply the mean point. */
  cloud.interior_point /= ssize(cloud.vertices);

  return cloud;
}

/* Wrapper around qhull that provides construction from either a cloud of
 vertices or half spaces, along with useful utilities. */
class ConvexHull {
 public:
  /* Construct the convex hull for a `cloud` of vertices. */
  explicit ConvexHull(VertexCloud cloud) : cloud_(std::move(cloud)) {
    if (cloud_.is_planar) {
      /* When the vertices are planar, we introduce a single point off the plane
       (which still projects into the cloud). Later, when we convert qhull's
       result into a mesh, we'll find the one vertex obviously off the plane and
       move it back onto the plane again. */
      cloud_.vertices.push_back(cloud_.interior_point + cloud_.n);
    }
    RunQHull();
  }

  /* Makes a new convex hull representing a polytope which is the result of
   moving all faces of `this` convex hull an amount `margin` along their
   normals. */
  ConvexHull MakeInflatedConvexHull(double margin) const {
    // The same hull described by a set of half-spaces.
    std::vector<Vector4d> h = GetHalfSpaces();
    const Vector3d& c = cloud_.interior_point;

    // Construct the hull of the half spaces moved by a "margin" amount.
    return ConvexHull(c, std::move(h), margin);
  }

  /* Makes a polygon surface mesh for `this` convex hull. */
  PolygonSurfaceMesh<double> MakePolygonSurfaceMesh() const {
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
      const Vector3d center(facet.getCenter().coordinates());
      std::vector<int> ordered_vertices =
          OrderPolyVertices(vertices_M, mesh_indices, center, normal);

      // Now populate the face data.
      face_data.push_back(ssize(ordered_vertices));
      face_data.insert(face_data.end(), ordered_vertices.begin(),
                       ordered_vertices.end());
    }

    return PolygonSurfaceMesh(std::move(face_data), std::move(vertices_M));
  }

 private:
  /* Convex hull for a set of vertices `v`.
   @pre Vertices are not planar. */
  explicit ConvexHull(std::vector<Vector3d> v)
      : ConvexHull(VertexCloud{.vertices = std::move(v)}) {}

  /* Constructs the convex hull of a polytope given its H-representation
   (representation as the intersection of half spaces). See GetHalfSpaces()
   for the data format of the half spaces. Half spaces are moved a distance
   `margin` along their (outward) normal, effectively "inflating" the
   resulting convex hull.
   @param c A point inside the H-polytope. While the half-spaces fully
   represent the polytope, an interior point is needed to construct the dual.
   @param h The half-spaces defining the polytope.
   @param margin The margin value, see @ref hydro_margin.
   @pre The intersection of the half spaces forms a closed polytope.
   @pre The polytope includes vertex c. */
  ConvexHull(const Vector3d& c, std::vector<Vector4d> h, double margin) {
    /* We use the result in Proposition 5.6 in the book by [Gallier and
     Quaintance, 2022]. Paraphrased, we can construct the convex hull of the
     dual of an H-Polytope (a polyhedron described by its half spaces) by taking
     the convex hull of the normals scaled by the inverse of the distance to the
     origin. To recover the desired (primal) convex hull, we use the result that
     for a convex set A, the dual of the dual recovers the set A.

     With this result, "inflating" a convex geometry simply entails modifying
     the distance dᵢ of a half space into dᵢ + margin.

     Gallier, Jean, and Jocelyn Quaintance, 2022. "Aspects of convex geometry
     polyhedra, linear programming, shellings, Voronoi diagrams, Delaunay
     triangulations". Available online at:
     https://www.cis.upenn.edu/~jean/gbooks/convexpoly.html */

    // Shift the frame of the half-spaces to the interior point c. In other
    // words, v = 0 is contained within the polytope expressed in this shifted
    // frame. This is a requirement to compute the dual of this hull.
    for (Vector4d& hi : h) {
      auto ni = hi.head<3>();  // normal
      double& di = hi(3);      // distance to origin.
      di -= ni.dot(c);
      DRAKE_DEMAND(di > 0);  // c must be inside the H-polygon.
    }

    // First hull to compute the dual.
    std::vector<Vector3d> v_dual;
    v_dual.reserve(h.size());
    for (const Vector4d& hi : h) {
      const auto ni = hi.head<3>();  // normal
      const double di = hi(3);       // distance to origin.
      DRAKE_DEMAND(di > 0);  // Must be true since we shifted to the origin.
      const Vector3d v = ni / (di + margin);
      v_dual.push_back(v);
    }
    ConvexHull dual_hull(std::move(v_dual));

    // Second hull to compute the dual of the dual.
    const std::vector<Vector4d> h_dual = dual_hull.GetHalfSpaces();
    std::vector<Vector3d> v_inflated;
    v_inflated.reserve(h_dual.size());
    for (const Vector4d& hi : h_dual) {
      const auto ni = hi.head<3>();  // normal
      const double di = hi(3);       // distance to origin.
      DRAKE_DEMAND(di > 0);
      const Vector3d v = ni / di;
      v_inflated.push_back(v + c);  // Shift back to c.
    }
    cloud_ = VertexCloud{.vertices = std::move(v_inflated)};
    /* N.B. While not required for the RunQHull() call below, we make sure to
     leave the ConvexHull object in a valid state. This would allow, for
     instance, to compute a valid PolygonMesh of the dual if so needed for
     debugging or visualization. */
    cloud_.interior_point = c;
    RunQHull();
  }

  /* Returns a vector of half spaces that delimit `this` convex hull. Each half
   space is represented by the equation n̂ᵢ⋅x ≤ dᵢ, where n̂ᵢ is the outward
   normal to the plane delimiting the half space and dᵢ the distance to the
   origin. Each entry in the returned vector corresponds to a half space
   represented as a Vector4d, where the first three entries correspond to the
   normal n̂ᵢ and the last component corresponds to dᵢ. */
  std::vector<Vector4d> GetHalfSpaces() const {
    std::vector<Vector4d> hyperplanes;
    hyperplanes.reserve(qhull_.facetCount());
    for (auto& facet : qhull_.facetList()) {
      // QHull doesn't necessarily order the vertices in the winding we want.
      const Vector3d normal(facet.hyperplane().coordinates());
      DRAKE_DEMAND(std::abs(normal.norm() - 1.0) < 1.0e-14);
      const double d = -facet.hyperplane().offset();  // distance to origin.
      Vector4d h = (Vector4d() << normal, d).finished();
      hyperplanes.push_back(h);
    }
    return hyperplanes;
  }

  /* Helper to run the underlying qhull on the owned vertex cloud.
    @pre member cloud_ is not empty and contains a non-planar cloud. */
  void RunQHull() {
    // This is a cheat that relies on the fact that a vector of Vector3d really
    // is just a vector<double> with three times as many entries. It *does*
    // eliminate a copy.
    static_assert(sizeof(Vector3d) == 3 * sizeof(double));
    qhull_.runQhull(/* inputComment = */ "", kDim, cloud_.vertices.size(),
                    reinterpret_cast<const double*>(cloud_.vertices.data()),
                    /* qhullCommand = */ "");

    if (qhull_.qhullStatus() != 0) {
      throw std::runtime_error(
          fmt::format("RunQHull failed. Qhull terminated with status {} "
                      "and message:\n{}",
                      qhull_.qhullStatus(), qhull_.qhullMessage()));
    }
  }

  static constexpr int kDim{3};
  VertexCloud cloud_;
  // The underlying qhull object.
  orgQhull::Qhull qhull_;
};

}  // namespace

PolygonSurfaceMesh<double> MakeConvexHull(const MeshSource& mesh_source,
                                          const Vector3d& scale,
                                          double margin) {
  DRAKE_THROW_UNLESS(margin >= 0);
  VertexCloud cloud = ReadVertices(mesh_source, scale);

  // Hull of the input cloud of vertices.
  const ConvexHull hull(std::move(cloud));

  // We do not apply margin to planar clouds.
  if (cloud.is_planar || margin == 0) {
    return hull.MakePolygonSurfaceMesh();
  }

  // Construct the hull of the half spaces moved by a "margin" amount.
  const ConvexHull inflated_hull = hull.MakeInflatedConvexHull(margin);

  return inflated_hull.MakePolygonSurfaceMesh();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
