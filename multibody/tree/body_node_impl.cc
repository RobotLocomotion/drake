#include "drake/multibody/tree/body_node_impl.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/planar_mobilizer.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/rpy_ball_mobilizer.h"
#include "drake/multibody/tree/rpy_floating_mobilizer.h"
#include "drake/multibody/tree/screw_mobilizer.h"
#include "drake/multibody/tree/universal_mobilizer.h"
#include "drake/multibody/tree/weld_mobilizer.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, template <typename> class ConcreteMobilizer>
BodyNodeImpl<T, ConcreteMobilizer>::~BodyNodeImpl() = default;

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcPositionKinematicsCache_BaseToTip(
    const FrameBodyPoseCache<T>& frame_body_pose_cache,
    const T* positions,
    PositionKinematicsCache<T>* pc) const {
  // This method must not be called for the "world" body node.
  DRAKE_ASSERT(this->index() != world_mobod_index());
  DRAKE_ASSERT(pc != nullptr);

  // TODO(sherm1) This should be an update rather than generating a whole
  //  new transform.

  // Update mobilizer-specific position dependent kinematics.
  math::RigidTransform<T>& X_FM = this->get_mutable_X_FM(pc);
  X_FM = mobilizer_->calc_X_FM(get_q(positions));

  // Given X_FM that we just put into PositionKinematicsCache (pc), and
  // calculations already done through the parent mobilizer, this
  // calculates into pc: X_PB, X_WB, p_PoBo_W
  // These don't depend on Mobilizer details.
  this->CalcAcrossMobilizerBodyPoses_BaseToTip(frame_body_pose_cache, pc);
}

// TODO(sherm1) Consider combining this with VelocityCache computation
//  so that we don't have to make a separate pass. Or better, get rid of this
//  computation altogether by working in better frames.
template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::
    CalcAcrossNodeJacobianWrtVExpressedInWorld(
        const systems::Context<T>& context,
        const FrameBodyPoseCache<T>& frame_body_pose_cache,
        const PositionKinematicsCache<T>& pc,
        std::vector<Vector6<T>>* H_PB_W_cache) const {
  DRAKE_ASSERT(H_PB_W_cache != nullptr);
  DRAKE_ASSERT(this->index() != world_mobod_index());

  // Nothing to do for a weld mobilizer.
  if constexpr (kNv > 0) {
    // Inboard frame F of this node's mobilizer.
    const Frame<T>& frame_F = this->inboard_frame();
    // Outboard frame M of this node's mobilizer.
    const Frame<T>& frame_M = this->outboard_frame();

    const math::RigidTransform<T>& X_PF =
        frame_F.get_X_BF(frame_body_pose_cache);  // B==P
    const math::RotationMatrix<T>& R_PF = X_PF.rotation();
    const math::RigidTransform<T>& X_MB =
        frame_M.get_X_FB(frame_body_pose_cache);  // F==M

    // Form the rotation matrix relating the world frame W and parent body P.
    const math::RotationMatrix<T>& R_WP = this->get_R_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    const math::RotationMatrix<T> R_WF = R_WP * R_PF;

    // Vector from Mo to Bo expressed in frame F as needed below:
    const math::RotationMatrix<T>& R_FM = this->get_X_FM(pc).rotation();
    const Vector3<T>& p_MB_M = X_MB.translation();
    const Vector3<T> p_MB_F = R_FM * p_MB_M;

    VVector v = VVector::Zero();
    Eigen::Map<HMatrix> H_PB_W = get_mutable_H(H_PB_W_cache);
    // We compute H_FM(q) one column at a time by calling the multiplication by
    // H_FM operation on a vector of generalized velocities which is zero except
    // for its imob-th component, which is one.
    for (int imob = 0; imob < kNv; ++imob) {
      v(imob) = 1.0;
      // Compute the imob-th column of H_FM:
      const SpatialVelocity<T> Himob_FM =
          mobilizer_->calc_V_FM(context, Eigen::Map<const VVector>(v.data()));
      v(imob) = 0.0;
      // V_PB_W = V_PFb_W + V_FMb_W + V_MB_W = V_FMb_W =
      //         = R_WF * V_FM.Shift(p_MoBo_F)
      H_PB_W.col(imob) = (R_WF * Himob_FM.Shift(p_MB_F)).get_coeffs();
    }
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcVelocityKinematicsCache_BaseToTip(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const std::vector<Vector6<T>>& H_PB_W_cache,
    const T* velocities,
    VelocityKinematicsCache<T>* vc) const {
  // This method must not be called for the "world" body node.
  DRAKE_ASSERT(this->index() != world_mobod_index());
  DRAKE_ASSERT(vc != nullptr);

  // Hinge matrix for this node. H_PB_W ∈ ℝ⁶ˣⁿᵐ with nm ∈ [0; 6] the
  // number of mobilities for this node. Therefore, the return is a
  // MatrixUpTo6 since the number of columns generally changes with the
  // node.  It is returned as an Eigen::Map to the memory allocated in the
  // std::vector H_PB_W_cache so that we can work with H_PB_W as with any
  // other Eigen matrix object.
  //  Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
  //      this->GetJacobianFromArray(H_PB_W_cache);
  //  DRAKE_ASSERT(H_PB_W.rows() == 6);
  //  DRAKE_ASSERT(H_PB_W.cols() == this->get_num_mobilizer_velocities());

  // As a guideline for developers, a summary of the computations performed in
  // this method is provided:
  // Notation:
  //  - B body frame associated with this node.
  //  - P ("parent") body frame associated with this node's parent.
  //  - F mobilizer inboard frame attached to body P.
  //  - M mobilizer outboard frame attached to body B.
  // The goal is computing the spatial velocity V_WB of body B measured in the
  // world frame W. The calculation is recursive and assumes the spatial
  // velocity V_WP of the inboard body P is already computed. These spatial
  // velocities are related by the recursive relation:
  //   V_WB = V_WPb + V_PB_W (Eq. 5.6 in Jain (2010), p. 77)              (1)
  // where Pb is a frame aligned with P but with its origin shifted from Po
  // to B's origin Bo. Then V_WPb is the spatial velocity of frame Pb,
  // measured and expressed in the world frame W. Then since V_PB's
  // translational component is also for the point Bo, we can add these
  // spatial velocities. Therefore we need to develop expressions for the two
  // terms (V_WPb and V_PB_W) in Eq. (1).
  //
  // Computation of V_PB_W:
  // Let Mb be a frame aligned rigidly with M but with its origin at Bo.
  // For rigid bodies (which we always have here)
  //   V_PB_W = V_FMb_W                                                   (2)
  // which can be computed from the spatial velocity measured in frame F (as
  // provided by mobilizer's methods)
  //   V_FMb_W = R_WF * V_FMb = R_WF * V_FM.Shift(p_MoBo_F)               (3)
  // arriving to the desired result:
  //   V_PB_W = R_WF * V_FM.Shift(p_MoBo_F)                               (4)
  //
  // V_FM is immediately available from this node's mobilizer with the method
  // CalcAcrossMobilizerSpatialVelocity() which computes M's spatial velocity
  // in F by V_FM = H_FM * vm, where H_FM is the mobilizer's hinge matrix.
  //
  // Computation of V_WPb:
  // This can be computed by a simple shift operation from V_WP:
  //   V_WPb = V_WP.Shift(p_PoBo_W)                                       (5)
  //
  // Note:
  // It is very common to find treatments in which the body frame B is
  // coincident with the outboard frame M, that is B ≡ M, leading to slightly
  // simpler recursive relations (for instance, see Section 3.3.2 in
  // Jain (2010)) where p_MoBo_F = 0 and thus V_PB_W = V_FM_W.

  // Generalized velocities local to this node's mobilizer.
  const Eigen::Map<const VVector> vm = get_v(velocities);

  // =========================================================================
  // Computation of V_PB_W in Eq. (1). See summary at the top of this method.

  // Update V_FM using the operator V_FM = H_FM * vm:
  SpatialVelocity<T>& V_FM = get_mutable_V_FM(vc);
  V_FM = mobilizer_->calc_V_FM(context, vm);

  // Compute V_PB_W = R_WF * V_FM.Shift(p_MoBo_F), Eq. (4).
  // Side note to developers: in operator form for rigid bodies this would be
  //   V_PB_W = R_WF * phiT_MB_F * V_FM
  //          = R_WF * phiT_MB_F * H_FM * vm
  //          = H_PB_W * vm
  // where H_PB_W = R_WF * phiT_MB_F * H_FM.
  SpatialVelocity<T>& V_PB_W = get_mutable_V_PB_W(vc);
  if constexpr (kNv > 0) {
    // Hinge matrix for this node. H_PB_W ∈ ℝ⁶ˣⁿᵛ with nv ∈ [0; 6] the
    // number of mobilities for this node.
    const Eigen::Matrix<T, 6, kNv>& H_PB_W = get_H(H_PB_W_cache);
    V_PB_W.get_coeffs() = H_PB_W * vm;
  } else {
    V_PB_W.get_coeffs().setZero();
  }

  // =========================================================================
  // Computation of V_WPb in Eq. (1). See summary at the top of this method.

  // Shift vector between the parent body P and this node's body B,
  // expressed in the world frame W.
  const Vector3<T>& p_PB_W = this->get_p_PoBo_W(pc);

  // Since we are in a base-to-tip recursion the parent body P's spatial
  // velocity is already available in the cache.
  const SpatialVelocity<T>& V_WP = this->get_V_WP(*vc);

  // =========================================================================
  // Update velocity V_WB of this node's body B in the world frame. Using the
  // recursive Eq. (1). See summary at the top of this method.
  this->get_mutable_V_WB(vc) =
      V_WP.ComposeWithMovingFrameVelocity(p_PB_W, V_PB_W);
}

// Macro used to explicitly instantiate implementations for every mobilizer.
#define EXPLICITLY_INSTANTIATE_IMPLS(T)                        \
  template class BodyNodeImpl<T, PlanarMobilizer>;             \
  template class BodyNodeImpl<T, PrismaticMobilizer>;          \
  template class BodyNodeImpl<T, QuaternionFloatingMobilizer>; \
  template class BodyNodeImpl<T, RevoluteMobilizer>;           \
  template class BodyNodeImpl<T, RpyBallMobilizer>;            \
  template class BodyNodeImpl<T, RpyFloatingMobilizer>;        \
  template class BodyNodeImpl<T, ScrewMobilizer>;              \
  template class BodyNodeImpl<T, UniversalMobilizer>;          \
  template class BodyNodeImpl<T, WeldMobilizer>

// Explicitly instantiates on the supported scalar types.
// These should be kept in sync with the list in default_scalars.h.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);
EXPLICITLY_INSTANTIATE_IMPLS(symbolic::Expression);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
