#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace internal {

/// Stores the computed contact Jacobians when a point contact model is used.
/// At a given state of the multibody system, there will be `nc` contact pairs.
/// For each penetration pair involving bodies A and B a contact frame C is
/// defined by the rotation matrix `R_WC = [Cx_W, Cy_W, Cz_W]` where
/// `Cz_W = nhat_BA_W` equals the normal vector pointing from body B into body
/// A, expressed in the world frame W. See PenetrationAsPointPair for further
/// details on the definition of each contact pair. Versors `Cx_W` and `Cy_W`
/// constitute a basis of the plane normal to `Cz_W` and are arbitrarily chosen.
/// Below, v denotes the vector of generalized velocities, of size `nv`.
/// @see MultibodyPlant::EvalContactJacobians().
template <class T>
struct ContactJacobians {
  /// Normal contact Jacobian.
  /// `Jn` is a matrix of size `nc x nv` such that `vn = Jn⋅v` is the separation
  /// speed for each contact point, defined to be positive when bodies are
  /// moving away from each other.
  MatrixX<T> Jn;

  /// Tangential contact forces Jacobian.
  /// `Jt` is a matrix of size `2⋅nc x nv` such that `vt = Jt⋅v` concatenates
  /// the tangential components of the relative velocity vector `v_AcBc`
  /// in the frame C of contact, for each pair. That is, for the k-th contact
  /// pair, `vt.segment<2>(2 * ik)` stores the components of `v_AcBc` in the
  /// `Cx` and `Cy` directions.
  MatrixX<T> Jt;

  /// List of contact frames orientation R_WC in the world frame W for each
  /// contact pair.
  std::vector<drake::math::RotationMatrix<T>> R_WC_list;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::ContactJacobians)
