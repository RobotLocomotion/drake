#include "drake/geometry/proximity/volume_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
void VolumeMesh<T>::TransformVertices(
    const math::RigidTransform<T>& transform) {
  const math::RigidTransform<T>& X_NM = transform;
  for (Vector3<T>& vertex : vertices_M_) {
    const Vector3<T> p_MV = vertex;
    vertex = X_NM * p_MV;
  }

  // Transform all position dependent quantities.
  const math::RotationMatrix<T>& R_NM = X_NM.rotation();
  for (int i = 0; i < num_elements(); ++i) {
    for (int j = 0; j < 4; ++j) {
      inward_normals_M_[i][j] = R_NM * inward_normals_M_[i][j];
    }

    for (int j = 0; j < 6; ++j) {
      edges_M_[i][j] = R_NM * edges_M_[i][j];
    }
  }
}

template <typename T>
void VolumeMesh<T>::SetAllPositions(const Eigen::Ref<const VectorX<T>>& p_MVs) {
  if (p_MVs.size() != 3 * num_vertices()) {
    throw std::runtime_error(
        fmt::format("SetAllPositions(): Attempting to deform a mesh with {} "
                    "vertices with data for {} DoFs",
                    num_vertices(), p_MVs.size()));
  }
  for (int v = 0, i = 0; v < num_vertices(); ++v, i += 3) {
    vertices_M_[v] = Vector3<T>(p_MVs[i], p_MVs[i + 1], p_MVs[i + 2]);
  }

  ComputePositionDependentQuantities();
}

template <typename T>
void VolumeMesh<T>::ComputePositionDependentQuantities() {
  inward_normals_M_.clear();
  edges_M_.clear();
  inward_normals_M_.reserve(num_elements());
  edges_M_.reserve(num_elements());
  for (int e = 0; e < num_elements(); ++e) {
    const Vector3<T>& a = vertices_M_[elements_[e].vertex(0)];
    const Vector3<T>& b = vertices_M_[elements_[e].vertex(1)];
    const Vector3<T>& c = vertices_M_[elements_[e].vertex(2)];
    const Vector3<T>& d = vertices_M_[elements_[e].vertex(3)];

    const Vector3<T> ab = b - a;
    const Vector3<T> ac = c - a;
    const Vector3<T> ad = d - a;
    const Vector3<T> bc = c - b;
    const Vector3<T> bd = d - b;
    const Vector3<T> cd = d - c;

    edges_M_.emplace_back(std::array<Vector3<T>, 6>{ab, ac, ad, bc, bd, cd});

    // Assume the first three vertices a, b, c define a triangle with its
    // right-handed normal pointing towards the inside of the tetrahedra. The
    // fourth vertex, d, is on the positive side of the plane defined by a,
    // b, c. The faces that wind CCW from inside the element are:
    //  {b d c}  Across from vertex a
    //  {c d a}  Across from vertex b
    //  {a d b}  Across from vertex c
    //  {a b c}  Across from vertex d
    inward_normals_M_.emplace_back(std::array<Vector3<T>, 4>{
        bd.cross(bc).normalized(), (cd).cross(-ac).normalized(),
        (ad).cross(ab).normalized(), (ab).cross(ac).normalized()});
  }
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh);

}  // namespace geometry
}  // namespace drake
