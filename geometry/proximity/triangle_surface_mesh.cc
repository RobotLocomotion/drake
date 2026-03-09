#include "drake/geometry/proximity/triangle_surface_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
void TriangleSurfaceMesh<T>::ReverseFaceWinding() {
  for (auto& f : triangles_) {
    f.ReverseWinding();
  }
  for (auto& n : face_normals_) {
    n = -n;
  }
}

template <typename T>
template <typename C>
typename TriangleSurfaceMesh<T>::template Barycentric<
    promoted_numerical_t<T, C>>
TriangleSurfaceMesh<T>::CalcBarycentric(const Vector3<C>& p_MQ, int t) const
  requires scalar_predicate<C>::is_bool
{
  const Vector3<T>& v0 = vertex(element(t).vertex(0));
  const Vector3<T>& v1 = vertex(element(t).vertex(1));
  const Vector3<T>& v2 = vertex(element(t).vertex(2));
  // Translate the triangle to the origin to simplify calculations;
  // barycentric coordinates stay the same.
  //     u⃗i = v⃗i - v0
  //     p_MR = p_MQ - v0
  //
  // Consider R' on the spanning plane through the origin, u1, u2:
  //     R' = b₀*u0 + b₁*u1 + b₂*u2
  //        = 0 + b₁*u1 + b₂*u2
  //        = b₁*u1 + b₂*u2
  //
  // Solve for b₁, b₂ that give R' "closest" to R in the least square sense:
  //
  //      |      ||b1|
  //      |u⃗1  u⃗2||b2| ~ R'
  //      |      |
  //
  // return Barycentric (1-b₁-b₂, b₁, b₂)
  //
  using ReturnType = promoted_numerical_t<T, C>;
  Eigen::Matrix<ReturnType, 3, 2> A;
  A.col(0) << v1 - v0;
  A.col(1) << v2 - v0;
  Vector2<ReturnType> solution = A.colPivHouseholderQr().solve(p_MQ - v0);

  const ReturnType& b1 = solution(0);
  const ReturnType& b2 = solution(1);
  const ReturnType b0 = T(1.) - b1 - b2;
  return {b0, b1, b2};
}
// TODO(DamrongGuoy): Investigate alternative calculation suggested by
//  Alejandro Castro:
// 1. Starting with the same ui and p_MR.
// 2. Calculate the unit normal vector n to the spanning plane S through
//    the origin, u1, and u2.
//        n = u1.cross(u2).normalize().
// 3. Project p_MR to p_MR' on the plane S,
//        p_MR' = p_MR - (p_MR.dot(n))*n
//
// Now we have p_MR' = b₀*u⃗0 + b₁*u⃗1 + b₂*u⃗2 by barycentric coordinates.
//                   =   0   + b₁*u1 + b₂*u2
//
// 5. Solve for b₁ and b₂.
//        (b₁*u1 + b₂*u2).dot(u1) = p_MR'.dot(u1)
//        (b₁*u1 + b₂*u2).dot(u2) = p_MR'.dot(u2)
//    Therefore, the 2x2 system:
//        |u1.dot(u1)  u2.dot(u1)||b1| = |p_MR'.dot(u1)|
//        |u1.dot(u2)  u2.dot(u2)||b2|   |p_MR'.dot(u2)|
//
// 6. return Barycentric(1-b₁-b₂, b₁, b₂)
//
// Optimization: save n, and the inverse of matrix |uᵢ.dot(uⱼ)| for later.
//

template <typename T>
void TriangleSurfaceMesh<T>::SetAllPositions(
    const Eigen::Ref<const VectorX<T>>& p_MVs) {
  if (p_MVs.size() != 3 * num_vertices()) {
    throw std::runtime_error(
        fmt::format("SetAllPositions(): Attempting to deform a mesh with {} "
                    "vertices with data for {} DoFs",
                    num_vertices(), p_MVs.size()));
  }
  for (int v = 0, i = 0; v < num_vertices(); ++v, i += 3) {
    vertices_M_[v] = Vector3<T>(p_MVs[i], p_MVs[i + 1], p_MVs[i + 2]);
  }
  // Update position dependent quantities after the vertex positions have been
  // updated.
  ComputePositionDependentQuantities();
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class TriangleSurfaceMesh);

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&TriangleSurfaceMesh<T>::template CalcBarycentric<U>));

}  // namespace geometry
}  // namespace drake
