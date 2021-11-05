#include "drake/geometry/proximity/field_intersection.h"

#include <unordered_map>

#include "drake/common/eigen_types.h"

#include "drake/common/default_scalars.h"
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
  if (n_M.norm() == 0) {
    return false;
  }

  // Using n_M in the plane equation (3), we have the plane equation:
  //   n_M.dot(p_MQ) = f1_Mo - f0_Mo,                     (5)
  // and a possible point on that plane is:
  //            p_MQ = (f1_Mo - f0_Mo)*n_M/n_M.dot(n_M).  (6)
  const Vector3<T> p_MQ = (f1_Mo - f0_Mo) * n_M / n_M.dot(n_M);

  *plane_M = Plane<T>{n_M, p_MQ};
  return true;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcEquilibriumPlane<T>))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
