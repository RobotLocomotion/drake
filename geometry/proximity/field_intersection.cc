#include "drake/geometry/proximity/field_intersection.h"

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

// Sets up the equilibrium plane between the two linear functions from the
// two tetrahedra.
template <typename T>
Plane<T> CalcEquilibriumPlane(
    VolumeElementIndex element0,
    const VolumeMeshFieldLinear<double, double>& field0_M,
    VolumeElementIndex element1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN) {
  const Vector3d df0_M = field0_M.EvaluateGradient(element0);
  const Vector3d p_MMo = Vector3d::Zero();
  const double f0_Mo = field0_M.EvaluateCartesian(element0, p_MMo);

  const Vector3d df1_N = field1_N.EvaluateGradient(element1);
  const Vector3<T> df1_M = X_MN.rotation() * df1_N.cast<T>();
  const Vector3<T> p_NMo = X_MN.inverse() * p_MMo.cast<T>();
  const T f1_Mo = field1_N.EvaluateCartesian(element1, p_NMo);

  // Extend the linear field field0 within tetrahedron element0 to the entire
  // space as:
  //      f0(p_MQ) = df0_M.dot(p_MQ) + f0_Mo.             (1)
  // Extend the linear field field1 within tetrahedron element1 to the entire
  // space as:
  //      f1(p_MQ) = df1_M.dot(p_MQ) + f1_Mo.             (2)
  // This is an equation of the equilibrium plane, where the two functions
  // have the same value:
  //   (df0_M - df1_M).dot(p_MQ) + (f0_Mo - f1_Mo) = 0.   (3)
  // Therefore, a vector n_M perpendicular to the equilibrium plane is:
  //             n_M = df0_M - df1_M.                     (4)
  const Vector3<T> n_M = df0_M - df1_M;
  // Using n_M in the plane equation (3), we have the plane equation:
  //   n_M.dot(p_MQ) = f1_Mo - f0_Mo,                     (5)
  // and a possible point on that plane is:
  //            p_MQ = (f1_Mo - f0_Mo)*n_M/n_M.dot(n_M).  (6)
  const Vector3<T> p_MQ = (f1_Mo - f0_Mo) * n_M / n_M.dot(n_M);

  return {n_M, p_MQ};
}

// TODO(DamrongGuoy): Refactor ClipPolygonByHalfSpace() to share between
//  mesh_intersection and field_intersection instead of this hack.
template<typename T>
class SurfaceVolumeIntersectorTester {
 public:
  void ClipPolygonByHalfSpace(const std::vector<Vector3<T>>& polygon_vertices_F,
                              const PosedHalfSpace<double>& H_F,
                              std::vector<Vector3<T>>* output_vertices_F) {
    intersect_.ClipPolygonByHalfSpace(polygon_vertices_F, H_F,
                                      output_vertices_F);
  }
 private:
  SurfaceVolumeIntersector<T> intersect_;
};

template <typename T>
void IntersectPlaneTetrahedron(
    VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M,
    const Plane<T>& plane_M, std::vector<Vector3<T>>* polygon_M) {
  std::vector<SurfaceFace> unused_faces;
  std::vector<SurfaceVertex<T>> vertices_with_centroid_M;
  std::vector<T> unused_surface_e;
  std::unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>
      unused_cut_edges;

  SliceTetWithPlane(
      tetrahedron, field_M, plane_M, math::RigidTransform<T>::Identity(),
      ContactPolygonRepresentation::kCentroidSubdivision, &unused_faces,
      &vertices_with_centroid_M, &unused_surface_e, &unused_cut_edges);

  int num_vertices_exclude_centroid = vertices_with_centroid_M.size() - 1;
  for (int i = 0; i < num_vertices_exclude_centroid; ++i) {
    polygon_M->emplace_back(vertices_with_centroid_M[i].r_MV());
  }
}

template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    VolumeElementIndex element0,
    const VolumeMeshFieldLinear<double, double>& field0_M,
    VolumeElementIndex element1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN,
    Vector3<T>* normal) {
  const Plane<T> equilibrium_plane_M =
      CalcEquilibriumPlane(element0, field0_M, element1, field1_N, X_MN);
  *normal = equilibrium_plane_M.normal();

  // TODO(DamrongGuoy): Refactor this buffer from being a local variable to
  //  being a class variable to decrease heap allocations. Then, return
  //  the const reference ("const std::vector<Vector3<T>>&").
  std::vector<Vector3<T>> polygon_buffer[2];

  // Intersects the equilibrium plane with the tetrahedron element0.
  std::vector<Vector3<T>>* polygon_M = &(polygon_buffer[0]);
  IntersectPlaneTetrahedron(element0, field0_M, equilibrium_plane_M, polygon_M);

  // Intersection of the above polygon with the four halfspaces of the four
  // faces of the tetrahedron element1. Expressed in frame M.
  Vector3<double> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    Vector3d p_NVi = field1_N.mesh()
                         .vertex(field1_N.mesh().element(element1).vertex(i))
                         .r_MV();
    Vector3<T> p_MV = X_MN * p_NVi.cast<T>();
    p_MVs[i] = Vector3<double>(ExtractDoubleOrThrow(p_MV.x()),
                               ExtractDoubleOrThrow(p_MV.y()),
                               ExtractDoubleOrThrow(p_MV.z()));
  }
  const int kFaceVertexLocalIndex[4][3] = {
      {1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  DRAKE_ASSERT(polygon_M == &(polygon_buffer[0]));
  std::vector<Vector3<T>>* in_M = polygon_M;
  std::vector<Vector3<T>>* out_M = &(polygon_buffer[1]);
  for (const auto& face_vertex : kFaceVertexLocalIndex) {
    const Vector3<double>& p_MA = p_MVs[face_vertex[0]];
    const Vector3<double>& p_MB = p_MVs[face_vertex[1]];
    const Vector3<double>& p_MC = p_MVs[face_vertex[2]];
    const Vector3<double> normal_M = (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<double> half_space_M(normal_M, p_MA);
    SurfaceVolumeIntersectorTester<T>().ClipPolygonByHalfSpace(
        *in_M, half_space_M, out_M);
    std::swap(in_M, out_M);
  }
  polygon_M = in_M;

  // TODO(DamrongGuoy): Take care of duplicated vertices or vertices that are
  //  so closed together. Use RemoveDuplicateVertices() of
  //  SurfaceVolumeIntersector.

  return *polygon_M;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
  &IntersectTetrahedra<T>
))


}  // namespace internal
}  // namespace geometry
}  // namespace drake
