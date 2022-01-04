#include "drake/geometry/proximity/field_intersection.h"

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

template <typename T>
bool CalcEquilibriumPlane(int element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          int element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M) {
  const Vector3d grad_f0_M = field0_M.EvaluateGradient(element0);
  const Vector3d p_MMo = Vector3d::Zero();
  // Value of f₀ at the origin of frame M.
  const double f0_Mo = field0_M.EvaluateCartesian(element0, p_MMo);

  const Vector3d grad_f1_N = field1_N.EvaluateGradient(element1);
  const Vector3<T> grad_f1_M = X_MN.rotation() * grad_f1_N.cast<T>();
  const Vector3<T> p_NMo = X_MN.inverse() * p_MMo.cast<T>();
  // Value of f₁ at the origin of frame M, which is the frame of f₀.
  const T f1_Mo = field1_N.EvaluateCartesian(element1, p_NMo);

  // In frame M, the two linear functions are:
  //      f₀(p_MQ) = grad_f0_M.dot(p_MQ) + f0_Mo.
  //      f₁(p_MQ) = grad_f1_M.dot(p_MQ) + f1_Mo.
  // Their equilibrium plane is:
  //   (grad_f0_M - grad_f1_M).dot(p_MQ) + (f0_Mo - f1_Mo) = 0.   (1)
  // Its perpendicular (but not necessarily unit-length) vector is:
  //           n_M = grad_f0_M - grad_f1_M,
  // which is in the direction of increasing f₀ and decreasing f₁.
  const Vector3<T> n_M = grad_f0_M - grad_f1_M;
  const T magnitude = n_M.norm();
  // TODO(DamrongGuoy): Change the threshold according to some
  //  experiments with respect to use cases, or make it a parameter.
  //  It is not clear to me what is the appropriate tolerance since the
  //  `magnitude` is in unit-field-value per meter. The other idea is to use
  //  non-unit-length normal vector in the Plane class; however, it will change
  //  the Plane API contract (the CalcHeight() will have different meaning).
  if (magnitude <= 0.0) {
    return false;
  }
  const Vector3<T> nhat_M = n_M / magnitude;

  // Using the unit normal vector nhat_M, the plane equation (1) becomes:
  //
  //          nhat_M.dot(p_MQ) + Δ = 0,
  //
  // where Δ = (f0_Mo - f1_Mo)/‖n_M‖. One such p_MQ is:
  //
  //                          p_MQ = -Δ * nhat_M
  //
  const Vector3<T> p_MQ = -((f0_Mo - f1_Mo) / magnitude) * nhat_M;

  *plane_M = Plane<T>(nhat_M, p_MQ, /*already_normalized = */ true);
  return true;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcEquilibriumPlane<T>))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
