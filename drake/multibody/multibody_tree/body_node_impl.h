#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

/// For internal use only of the MultibodyTree implementation.
/// While all code that is common to any node can be placed in the BodyNode
/// class, %BodyNodeImpl provides compile-time fixed-size BodyNode
/// implementations so that all operations can be perfomed with fixed-size
/// stack-allocated Eigen variables.
/// In particular, most of the across mobilizer code for velocity kinematics
/// lives in this class since the across mobilizer Jacobian matrices `H_FM(q)`
/// (defined such that the across mobilizer spatial velocity relates to the
/// generalized velocities v by `V_FM = H_FM(q) * v`) have a compile-time fixed
/// size. For a more detailed discussion of the role of a BodyNode in a
/// MultibodyTree refer to the class documentation for BodyNode.
template <typename T, int  num_positions, int num_velocities>
class BodyNodeImpl : public BodyNode<T> {
 public:
  // static constexpr int i = 42; discouraged.  See answer in:
  // http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {kNq = num_positions, kNv = num_velocities};

  /// Given a body and its inboard mobilizer in a MultibodyTree this constructor
  /// creates the corresponding %BodyNode. See the BodyNode class documentation
  /// for details on how a BodyNode is defined.
  /// @param[in] parent_node
  ///   A const pointer to the parent BodyNode object in the tree structure of
  ///   the owning MultibodyTree. It can be a `nullptr` only when `body` **is**
  ///   the **world** body, otherwise the parent class constructor will abort.
  /// @param[in] body The body B associated with `this` node.
  /// @param[in] mobilizer The mobilizer associated with this `node`. It can
  ///                      only be a `nullptr` for the **world** body.
  BodyNodeImpl(const internal::BodyNode<T>* parent_node,
               const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(parent_node, body, mobilizer) {}

  // TODO(amcastro-tri): Implement methods for computing velocity kinematics
  // using fixed-size Eigen matrices.

#if 0
  void CalcAcrossMobilizerPointsGeometricJacobianInWorld(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const Frame<T>& frame_B,
      const Eigen::Ref<const Matrix3X<T>>& p_BQi_set,
      EigenPtr<MatrixX<T>> J_WQi) const override {
    DRAKE_DEMAND(J_WQi != nullptr);
    const int num_points = p_BQi_set.cols();
    const int Jnum_rows = 3 * num_points;
    DRAKE_DEMAND(J_WQi->rows() == Jnum_rows);
    DRAKE_DEMAND(J_WQi->cols() ==  kNv);

    // TODO(amcastro-tri): consider refactoring this code in terms of J_PB_W(q)
    // so that we can reuse those computations in the velocity kinematics as
    // well.

    // Compute orientation of F in W:
    const Frame<T>& frame_F = this->get_inboard_frame();
    const Frame<T>& frame_M = this->get_outboard_frame();

    const Isometry3<T> X_PF = frame_F.CalcPoseInBodyFrame(context);
    const Isometry3<T> X_MB = frame_M.CalcPoseInBodyFrame(context).inverse();
    const Matrix3<T>& R_MB = X_MB.linear();
    const Isometry3<T>& X_WP = this->get_X_WP(pc);

    // TODO(amcastro-tri): R_WF is computed also by velocity kinematics.
    // Probably worth caching.
    const Matrix3<T> R_WF = X_WP.linear() * X_PF.linear();

    const Matrix3<T>& R_FM = this->get_X_FM(pc).linear();
    const Vector3<T>& p_MoBo_M = X_MB.translation();

    Eigen::Matrix<T, kNv, 1> v = Eigen::Matrix<T, kNv, 1>::Zero();
    for (int ipoint = 0; ipoint < num_points; ++ipoint) {
      const int Jrow = 3 * ipoint;
      const auto& p_BQi = p_BQi_set.col(ipoint);
      // TODO(amcastro-tri): this rotation by R_MB could be performe by frame_M
      // and cost nothing if M = B.
      const Vector3<T>& p_BoQi_M = R_MB * p_BQi;
      const Vector3<T> p_MoQi_F = R_FM * (p_MoBo_M + p_BoQi_M);

      for (int mobility = 0; mobility < kNv; ++mobility) {
        v(mobility) = 1.0;
        // i-th column of J_PB_W (probably worth caching).
        const SpatialVelocity<T> Ji_FM =
            this->get_mobilizer().CalcAcrossMobilizerSpatialVelocity(
                context, v);
        // V_PBqi_W = V_PFqi_W + V_FMqi_W + V_MBqi_W,
        // with V_PFqi_W = V_MBqi_W = 0 for rigid bodies we have:
        // V_PBqi_W = V_FMqi_W = V_FM_W.Shift(p_MoQi_W) =
        //                     = R_WF * V_FM_F.Shift(p_MoQi_F)
        const SpatialVelocity<T> Ji_PBqi_W = R_WF * Ji_FM.Shift(p_MoQi_F);
        v(mobility) = 0.0;
        J_WQi->col(mobility).template segment<3>(Jrow) =
            Ji_PBqi_W.translational();
      }
    }
  }
#endif
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
