#include "drake/geometry/proximity/volume_mesh.h"

#include "drake/common/default_scalars.h"
#include "drake/math/linear_solve.h"

namespace drake {
namespace geometry {

using drake::math::internal::PartialPivLU;

template <typename T>
VolumeMesh<T>::VolumeMesh(std::vector<VolumeElement>&& elements,
                          std::vector<Vector3<T>>&& vertices)
    : elements_(std::move(elements)), vertices_M_(std::move(vertices)) {
  if (elements_.empty()) {
    throw std::logic_error("A mesh must contain at least one tetrahedron");
  }
  ComputePositionDependentQuantities();
}

template <typename T>
template <typename C>
typename VolumeMesh<T>::template Barycentric<promoted_numerical_t<T, C>>
VolumeMesh<T>::CalcBarycentric(const Vector3<C>& p_MQ, int e) const
  requires scalar_predicate<C>::is_bool
{
  // We have two conditions to satisfy.
  // 1. b₀ + b₁ + b₂ + b₃ = 1
  // 2. b₀*v0 + b₁*v1 + b₂*v2 + b₃*v3 = p_M.
  // Together they create this 4x4 linear system:
  //
  //      | 1  1  1  1 ||b₀|   | 1 |
  //      | |  |  |  | ||b₁| = | | |
  //      | v0 v1 v2 v3||b₂|   |p_M|
  //      | |  |  |  | ||b₃|   | | |
  //
  // q = p_M - v0 = b₀*u0 + b₁*u1 + b₂*u2 + b₃*u3
  //              = 0 + b₁*u1 + b₂*u2 + b₃*u3
  using ReturnType = promoted_numerical_t<T, C>;
  Matrix4<ReturnType> A;
  for (int i = 0; i < 4; ++i) {
    A.col(i) << ReturnType(1.0), vertex(element(e).vertex(i));
  }
  Vector4<ReturnType> b;
  b << ReturnType(1.0), p_MQ;
  const math::LinearSolver<PartialPivLU, Matrix4<ReturnType>> A_lu(A);
  const Vector4<ReturnType> b_Q = A_lu.Solve(b);
  // TODO(DamrongGuoy): Save the inverse of the matrix instead of
  //  calculating it on the fly. We can reduce to 3x3 system too.  See
  //  issue #11653.
  return b_Q;
}

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
      edge_vectors_M_[i][j] = R_NM * edge_vectors_M_[i][j];
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
  edge_vectors_M_.clear();
  inward_normals_M_.reserve(num_elements());
  edge_vectors_M_.reserve(num_elements());
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

    edge_vectors_M_.push_back(
        std::array<Vector3<T>, 6>{ab, ac, ad, bc, bd, cd});

    // Assume the first three vertices a, b, c define a triangle with its
    // right-handed normal pointing towards the inside of the tetrahedra. The
    // fourth vertex, d, is on the positive side of the plane defined by a,
    // b, c. The faces that wind CCW from inside the element are:
    //  {b d c}  Across from vertex a
    //  {a c d}  Across from vertex b
    //  {a d b}  Across from vertex c
    //  {a b c}  Across from vertex d
    //
    // For example, a standard tetrahedron looks like this:
    //
    //              Mz
    //              ┆
    //            d ●
    //              ┆
    //              ┆    c
    //            a ●┄┄┄●┄┄┄ My
    //             ╱
    //          b ●
    //          ╱
    //
    inward_normals_M_.push_back(std::array<Vector3<T>, 4>{
        bd.cross(bc).normalized(), (ac).cross(ad).normalized(),
        (ad).cross(ab).normalized(), (ab).cross(ac).normalized()});
  }
}

template <typename T>
bool VolumeMesh<T>::Equal(const VolumeMesh<T>& mesh,
                          double vertex_tolerance) const {
  if (this == &mesh) return true;

  if (this->num_elements() != mesh.num_elements()) return false;
  if (this->num_vertices() != mesh.num_vertices()) return false;

  // Check tetrahedral elements.
  for (int i = 0; i < this->num_elements(); ++i) {
    if (!this->element(i).Equal(mesh.element(i))) return false;
  }
  // Check vertices.
  for (int i = 0; i < this->num_vertices(); ++i) {
    if ((this->vertex(i) - mesh.vertex(i)).norm() > vertex_tolerance) {
      return false;
    }
  }

  // All checks passed.
  return true;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh);

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&VolumeMesh<T>::template CalcBarycentric<U>));

}  // namespace geometry
}  // namespace drake
