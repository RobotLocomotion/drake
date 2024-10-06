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
BodyNodeImpl<T, ConcreteMobilizer>::BodyNodeImpl(
    const internal::BodyNode<T>* parent_node, const RigidBody<T>* rigid_body,
    const Mobilizer<T>* mobilizer)
    : BodyNode<T>(parent_node, rigid_body, mobilizer),
      mobilizer_(dynamic_cast<const ConcreteMobilizer<T>*>(mobilizer)) {
  DRAKE_DEMAND(mobilizer_ != nullptr);
}

template <typename T, template <typename> class ConcreteMobilizer>
BodyNodeImpl<T, ConcreteMobilizer>::~BodyNodeImpl() = default;

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcPositionKinematicsCache_BaseToTip(
    const FrameBodyPoseCache<T>& frame_body_pose_cache, const T* positions,
    PositionKinematicsCache<T>* pc) const {
  // This method must not be called for the "world" body node.
  DRAKE_ASSERT(mobod_index() != world_mobod_index());
  DRAKE_ASSERT(pc != nullptr);

  // TODO(sherm1) This should be an update rather than generating a whole
  //  new transform.

  // Update mobilizer-specific position dependent kinematics.
  math::RigidTransform<T>& X_FM = get_mutable_X_FM(pc);
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
  DRAKE_ASSERT(mobod_index() != world_mobod_index());

  // Nothing to do for a weld mobilizer.
  if constexpr (kNv > 0) {
    // Inboard frame F of this node's mobilizer.
    const Frame<T>& frame_F = inboard_frame();
    // Outboard frame M of this node's mobilizer.
    const Frame<T>& frame_M = outboard_frame();

    const math::RigidTransform<T>& X_PF =
        frame_F.get_X_BF(frame_body_pose_cache);  // B==P
    const math::RotationMatrix<T>& R_PF = X_PF.rotation();
    const math::RigidTransform<T>& X_MB =
        frame_M.get_X_FB(frame_body_pose_cache);  // F==M

    // Form the rotation matrix relating the world frame W and parent body P.
    const math::RotationMatrix<T>& R_WP = get_R_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    const math::RotationMatrix<T> R_WF = R_WP * R_PF;

    // Vector from Mo to Bo expressed in frame F as needed below:
    const math::RotationMatrix<T>& R_FM = get_X_FM(pc).rotation();
    const Vector3<T>& p_MB_M = X_MB.translation();
    const Vector3<T> p_MB_F = R_FM * p_MB_M;

    VVector v = VVector::Zero();
    auto H_PB_W = get_mutable_H(H_PB_W_cache);
    // We compute H_FM(q) one column at a time by calling the multiplication by
    // H_FM operation on a vector of generalized velocities which is zero except
    // for its imob-th component, which is one.
    for (int imob = 0; imob < kNv; ++imob) {
      v(imob) = 1.0;
      // Compute the imob-th column of H_FM:
      const SpatialVelocity<T> Himob_FM =
          mobilizer_->calc_V_FM(context, v.data());
      v(imob) = 0.0;
      // V_PB_W = V_PFb_W + V_FMb_W + V_MB_W = V_FMb_W =
      //         = R_WF * V_FM.Shift(p_MoBo_F)
      H_PB_W.col(imob) = (R_WF * Himob_FM.Shift(p_MB_F)).get_coeffs();
    }
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcVelocityKinematicsCache_BaseToTip(
    const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
    const std::vector<Vector6<T>>& H_PB_W_cache, const T* velocities,
    VelocityKinematicsCache<T>* vc) const {
  // This method must not be called for the "world" body node.
  DRAKE_ASSERT(mobod_index() != world_mobod_index());
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
  //  DRAKE_ASSERT(H_PB_W.cols() == mobilizer_->num_velocities();

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
  const T* v_ptr = get_v(velocities);
  const Eigen::Map<const VVector> v(v_ptr);

  // =========================================================================
  // Computation of V_PB_W in Eq. (1). See summary at the top of this method.

  // Update V_FM using the operator V_FM = H_FM * vm:
  SpatialVelocity<T>& V_FM = get_mutable_V_FM(vc);
  V_FM = mobilizer_->calc_V_FM(context, v_ptr);

  // Compute V_PB_W = R_WF * V_FM.Shift(p_MoBo_F), Eq. (4).
  // Side note to developers: in operator form for rigid bodies this would be
  //   V_PB_W = R_WF * phiT_MB_F * V_FM
  //          = R_WF * phiT_MB_F * H_FM * v
  //          = H_PB_W * v
  // where H_PB_W = R_WF * phiT_MB_F * H_FM.
  SpatialVelocity<T>& V_PB_W = get_mutable_V_PB_W(vc);
  if constexpr (kNv > 0) {
    // Hinge matrix for this node. H_PB_W ∈ ℝ⁶ˣⁿᵛ with nv ∈ [0; 6] the
    // number of mobilities for this node.
    const auto H_PB_W = get_H(H_PB_W_cache);
    V_PB_W.get_coeffs() = H_PB_W * v;
  } else {
    V_PB_W.get_coeffs().setZero();
  }

  // =========================================================================
  // Computation of V_WPb in Eq. (1). See summary at the top of this method.

  // Shift vector between the parent body P and this node's body B,
  // expressed in the world frame W.
  const Vector3<T>& p_PB_W = get_p_PoBo_W(pc);

  // Since we are in a base-to-tip recursion the parent body P's spatial
  // velocity is already available in the cache.
  const SpatialVelocity<T>& V_WP = get_V_WP(*vc);

  // =========================================================================
  // Update velocity V_WB of this node's body B in the world frame. Using the
  // recursive Eq. (1). See summary at the top of this method.
  get_mutable_V_WB(vc) = V_WP.ComposeWithMovingFrameVelocity(p_PB_W, V_PB_W);
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcSpatialAcceleration_BaseToTip(
    const systems::Context<T>& context,
    const FrameBodyPoseCache<T>& frame_body_poses_cache,
    const PositionKinematicsCache<T>& pc, const VelocityKinematicsCache<T>* vc,
    const VectorX<T>& mbt_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array_ptr) const {
  // This method must not be called for the "world" body node.
  DRAKE_DEMAND(mobod_index() != world_mobod_index());
  DRAKE_DEMAND(A_WB_array_ptr != nullptr);
  std::vector<SpatialAcceleration<T>>& A_WB_array = *A_WB_array_ptr;

  // As a guideline for developers, a summary of the computations performed in
  // this method is provided:
  // Notation:
  //  - B body frame associated with this node.
  //  - P ("parent") body frame associated with this node's parent.
  //  - F mobilizer inboard frame attached to body P.
  //  - M mobilizer outboard frame attached to body B.
  // The goal is computing the spatial acceleration A_WB of body B measured in
  // the world frame W. The calculation is recursive and assumes the spatial
  // acceleration A_WP of the inboard body P is already computed.
  // The spatial velocities of P and B are related by the recursive relation
  // (computation is performed by CalcVelocityKinematicsCache_BaseToTip():
  //   V_WB = V_WPb + V_PB_W (Eq. 5.6 in Jain (2010), p. 77)
  //        = V_WP.ComposeWithMovingFrameVelocity(p_PB_W, V_PB_W)         (1)
  // where Pb is a frame aligned with P but with its origin shifted from Po
  // to B's origin Bo. Then V_WPb is the spatial velocity of frame Pb,
  // measured and expressed in the world frame W.
  //
  // In the same way the parent body P velocity V_WP can be composed with body
  // B's velocity V_PB in P, the acceleration A_WB can be obtained by
  // composing A_WP with A_PB:
  //  A_WB = A_WP.ComposeWithMovingFrameAcceleration(
  //      p_PB_W, w_WP, V_PB_W, A_PB_W);                                  (2)
  // which includes both centrifugal and coriolis terms. For details on this
  // operation refer to the documentation for
  // SpatialAcceleration::ComposeWithMovingFrameAcceleration().
  //
  // By recursive precondition, this method was already called on all
  // predecessor nodes in the tree and therefore the acceleration A_WP is
  // already available.
  // V_WP (i.e. w_WP) and V_PB_W were computed in the velocity kinematics pass
  // and are therefore available in the VelocityKinematicsCache vc.
  //
  // Therefore, all that is left is computing A_PB_W = DtP(V_PB)_W.
  // The acceleration of B in P is:
  //   A_PB = DtP(V_PB) = DtF(V_FMb) = A_FM.Shift(p_MB, w_FM)             (3)
  // which expressed in the world frame leads to (see note below):
  //   A_PB_W = R_WF * A_FM.Shift(p_MB_F, w_FM)                           (4)
  // where R_WF is the rotation matrix from F to W and A_FM expressed in the
  // inboard frame F is the direct result from
  // Mobilizer::CalcAcrossMobilizerAcceleration().
  //
  // * Note:
  //     The rigid body assumption is made in Eq. (3) in two places:
  //       1. DtP() = DtF() since V_PF = 0.
  //       2. V_PB = V_FMb since V_PB = V_PFb + V_FMb + V_MB but since P is
  //          assumed rigid V_PF = 0 and since B is assumed rigid V_MB = 0.

  // RigidBody for this node. Its BodyFrame is also referred to as B whenever
  // no ambiguity can arise.
  const RigidBody<T>& body_B = body();

  // RigidBody for this node's parent, or the parent body P. Its BodyFrame
  // is also referred to as P whenever no ambiguity can arise.
  const RigidBody<T>& body_P = parent_body();

  // Inboard frame F of this node's mobilizer.
  const Frame<T>& frame_F = inboard_frame();
  DRAKE_ASSERT(frame_F.body().index() == body_P.index());
  // Outboard frame M of this node's mobilizer.
  const Frame<T>& frame_M = outboard_frame();
  DRAKE_ASSERT(frame_M.body().index() == body_B.index());

  // =========================================================================
  // Computation of A_PB = DtP(V_PB), Eq. (4).

  const math::RigidTransform<T>& X_PF =
      frame_F.get_X_BF(frame_body_poses_cache);  // B==P
  const math::RotationMatrix<T>& R_PF = X_PF.rotation();
  const math::RigidTransform<T>& X_MB =
      frame_M.get_X_FB(frame_body_poses_cache);  // F==M

  // Form the rotation matrix relating the world frame W and parent body P.
  // Available since we are called within a base-to-tip recursion.
  const math::RotationMatrix<T>& R_WP = get_R_WP(pc);

  // Orientation (rotation) of frame F with respect to the world frame W.
  // TODO(amcastro-tri): consider caching X_WF since it is also used to
  //  compute H_PB_W.
  const math::RotationMatrix<T> R_WF = R_WP * R_PF;

  // Vector from Mo to Bo expressed in frame F as needed below:
  // TODO(amcastro-tri): consider caching this since it is also used to
  //  compute H_PB_W.
  const math::RotationMatrix<T>& R_FM = get_X_FM(pc).rotation();
  const Vector3<T>& p_MB_M = X_MB.translation();
  const Vector3<T> p_MB_F = R_FM * p_MB_M;

  // Generalized velocities' time derivatives local to this node's mobilizer.
  const Eigen::Map<const VVector> vmdot = get_vvector(mbt_vdot.data());

  // Operator A_FM = H_FM * vmdot + Hdot_FM * vm
  SpatialAcceleration<T> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(context, vmdot);

  // =========================================================================
  // Compose acceleration A_WP of P in W with acceleration A_PB of B in P,
  // Eq. (2)

  // Obtains a const reference to the parent acceleration from A_WB_array.
  const SpatialAcceleration<T>& A_WP = get_A_WP_from_array(A_WB_array);

  // Shift vector between the parent body P and this node's body B,
  // expressed in the world frame W.
  const Vector3<T>& p_PB_W = get_p_PoBo_W(pc);

  if (vc != nullptr) {
    // Since we are in a base-to-tip recursion the parent body P's spatial
    // velocity is already available in the cache.
    const SpatialVelocity<T>& V_WP = get_V_WP(*vc);

    // For body B, only the spatial velocity V_PB_W is already available in
    // the cache. The acceleration A_PB_W was computed above.
    const SpatialVelocity<T>& V_PB_W = get_V_PB_W(*vc);

    // Across mobilizer velocity is available from the velocity kinematics.
    const SpatialVelocity<T>& V_FM = get_V_FM(*vc);

    const SpatialAcceleration<T> A_PB_W =
        R_WF * A_FM.Shift(p_MB_F, V_FM.rotational());  // Eq. (4)

    // Velocities are non-zero.
    get_mutable_A_WB_from_array(&A_WB_array) =
        A_WP.ComposeWithMovingFrameAcceleration(p_PB_W, V_WP.rotational(),
                                                V_PB_W, A_PB_W);
  } else {
    const SpatialAcceleration<T> A_PB_W =  // Eq. (4), with w_FM = 0.
        R_WF * A_FM.ShiftWithZeroAngularVelocity(p_MB_F);
    // Velocities are zero. No need to compute terms that become zero.
    get_mutable_A_WB_from_array(&A_WB_array) =
        A_WP.ShiftWithZeroAngularVelocity(p_PB_W) + A_PB_W;
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcInverseDynamics_TipToBase(
    const systems::Context<T>& context,
    const FrameBodyPoseCache<T>& frame_body_pose_cache,
    const PositionKinematicsCache<T>& pc,
    const std::vector<SpatialInertia<T>>& M_B_W_cache,
    const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
    const std::vector<SpatialAcceleration<T>>& A_WB_array,
    const SpatialForce<T>& Fapplied_Bo_W,
    const Eigen::Ref<const VectorX<T>>& tau_applied,
    std::vector<SpatialForce<T>>* F_BMo_W_array_ptr,
    EigenPtr<VectorX<T>> tau_array) const {
  DRAKE_DEMAND(F_BMo_W_array_ptr != nullptr);
  std::vector<SpatialForce<T>>& F_BMo_W_array = *F_BMo_W_array_ptr;
  DRAKE_DEMAND(tau_applied.size() == kNv || tau_applied.size() == 0);
  DRAKE_DEMAND(tau_array != nullptr);
  DRAKE_DEMAND(tau_array->size() == this->get_parent_tree().num_velocities());

  // As a guideline for developers, a summary of the computations performed in
  // this method is provided:
  // Notation:
  //  - B body frame associated with this node.
  //  - P ("parent") body frame associated with this node's parent.
  //  - F mobilizer inboard frame attached to body P.
  //  - M mobilizer outboard frame attached to body B.
  //  - Mo The origin of the outboard (or mobilized) frame of the mobilizer
  //       attached to body B.
  //  - C within a loop over children, one of body B's children.
  //  - Mc The origin of the outboard (or mobilized) frame of the mobilizer
  //       attached to body C.
  // The goal is computing the spatial force F_BMo_W (on body B applied at its
  // mobilized frame origin Mo) exerted by its inboard mobilizer that is
  // required to produce the spatial acceleration A_WB. The generalized forces
  // are then obtained as the projection of the spatial force F_BMo in the
  // direction of this node's mobilizer motion. That is, the generalized
  // forces correspond to the working components of the spatial force living
  // in the motion sub-space of this node's mobilizer.
  // The calculation is recursive (from tip-to-base) and assumes the spatial
  // force F_CMc on body C at Mc is already computed in F_BMo_W_array_ptr.
  //
  // The spatial force through body B's inboard mobilizer is obtained from a
  // force balance (essentially the F = m * a for rigid bodies, see
  // [Jain 2010, Eq. 2.26, p. 27] for a derivation):
  //   Ftot_BBo_W = M_Bo_W * A_WB + Fb_Bo_W                                (1)
  // where Fb_Bo_W contains the velocity dependent gyroscopic terms,
  // Ftot_BBo_W is the total spatial force on body B, applied at its origin Bo
  // and quantities are expressed in the world frame W (though the
  // expressed-in frame is not needed in a coordinate-free form.)
  //
  // The total spatial force on body B is the combined effect of externally
  // applied spatial forces Fapp_BMo on body B at Mo and spatial forces
  // induced by its inboard and outboard mobilizers. On its mobilized frame M,
  // in coordinate-free form:
  //   Ftot_BMo = Fapp_BMo + F_BMo - Σᵢ(F_CiMo)                           (2)
  // where F_CiMo is the spatial force on the i-th child body Ci due to its
  // inboard mobilizer which, by action/reaction, applies to body B as
  // -F_CiMo, hence the negative sign in the summation above. The applied
  // spatial force Fapp_BMo at Mo is obtained by shifting the applied force
  // Fapp_Bo from Bo to Mo as Fapp_BMo.Shift(p_BoMo).
  // Therefore, spatial force F_BMo due to body B's mobilizer is:
  //   F_BMo = Ftot_BMo + Σᵢ(F_CiMo) - Fapp_BMo                           (3)
  // The projection of this force on the motion sub-space of this node's
  // mobilizer corresponds to the generalized force tau:
  //  tau = H_FMᵀ * F_BMo_F                                               (4)
  // where the spatial force F_BMo must be re-expressed in the inboard frame F
  // before the projection can be performed.

  // This node's body B.
  const RigidBody<T>& body_B = body();

  // Input spatial acceleration for this node's body B.
  const SpatialAcceleration<T>& A_WB = get_A_WB_from_array(A_WB_array);

  // Total spatial force on body B producing acceleration A_WB.
  SpatialForce<T> Ftot_BBo_W;
  CalcBodySpatialForceGivenItsSpatialAcceleration(M_B_W_cache, Fb_Bo_W_cache,
                                                  A_WB, &Ftot_BBo_W);

  // Compute shift vector from Bo to Mo expressed in the world frame W.
  const Frame<T>& frame_M = outboard_frame();
  DRAKE_DEMAND(frame_M.body().index() == body_B.index());
  const math::RigidTransform<T>& X_BM =
      frame_M.get_X_BF(frame_body_pose_cache);  // F==M
  const Vector3<T>& p_BoMo_B = X_BM.translation();
  const math::RigidTransform<T>& X_WB = get_X_WB(pc);
  const math::RotationMatrix<T>& R_WB = X_WB.rotation();
  const Vector3<T> p_BoMo_W = R_WB * p_BoMo_B;

  // Output spatial force that would need to be exerted by this node's
  // mobilizer in order to attain the prescribed acceleration A_WB.
  SpatialForce<T>& F_BMo_W = F_BMo_W_array[mobod_index()];

  // Ensure this method was not called with an Fapplied_Bo_W being an entry
  // into F_BMo_W_array, otherwise we would be overwriting Fapplied_Bo_W.
  DRAKE_DEMAND(&F_BMo_W != &Fapplied_Bo_W);

  // Shift spatial force on B to Mo.
  F_BMo_W = Ftot_BBo_W.Shift(p_BoMo_W);
  for (const BodyNode<T>* child_node : this->child_nodes()) {
    const MobodIndex child_node_index = child_node->mobod_index();

    // Shift vector from Bo to Co, expressed in the world frame W.
    const Vector3<T>& p_BoCo_W = pc.get_p_PoBo_W(child_node_index);

    // p_CoMc_W:
    const Frame<T>& frame_Mc = child_node->outboard_frame();
    const math::RotationMatrix<T>& R_WC =
        pc.get_X_WB(child_node_index).rotation();
    const math::RigidTransform<T>& X_CMc =
        frame_Mc.get_X_BF(frame_body_pose_cache);  // B==C, F==Mc
    const Vector3<T>& p_CoMc_W = R_WC * X_CMc.translation();

    // Shift position vector from child C outboard mobilizer frame Mc to body
    // B outboard mobilizer Mc. p_MoMc_W:
    // Since p_BoMo = p_BoCo + p_CoMc + p_McMo, we have:
    const Vector3<T> p_McMo_W = p_BoMo_W - p_BoCo_W - p_CoMc_W;

    // Spatial force on the child body C at the origin Mc of the outboard
    // mobilizer frame for the child body.
    // A little note for how to read the next line: the frames for
    // F_BMo_W_array are:
    //  - B this node's body.
    //  - Mo body B's inboard frame origin.
    // However, when indexing by child_node_index:
    //  - B becomes C, the child node's body.
    //  - Mo becomes Mc, body C's inboard frame origin.
    const SpatialForce<T>& F_CMc_W = F_BMo_W_array[child_node_index];

    // Shift to this node's mobilizer origin Mo (still, F_CMo is the force
    // acting on the child body C):
    const SpatialForce<T>& F_CMo_W = F_CMc_W.Shift(p_McMo_W);
    // From Eq. (3), this force is added (with positive sign) to the force
    // applied by this body's mobilizer:
    F_BMo_W += F_CMo_W;
  }
  // Add applied forces contribution.
  F_BMo_W -= Fapplied_Bo_W.Shift(p_BoMo_W);

  // Re-express F_BMo_W in the inboard frame F before projecting it onto the
  // sub-space generated by H_FM(q).
  const math::RotationMatrix<T>& R_PF =
      inboard_frame().EvalPoseInBodyFrame(context).rotation();
  const math::RotationMatrix<T>& R_WP = get_R_WP(pc);
  // TODO(amcastro-tri): consider caching R_WF since also used in position and
  //  velocity kinematics.
  const math::RotationMatrix<T> R_WF = R_WP * R_PF;
  const SpatialForce<T> F_BMo_F = R_WF.inverse() * F_BMo_W;

  // Generalized velocities and forces use the same indexing.
  auto tau = mobilizer_->get_mutable_generalized_forces_from_array(tau_array);

  // Demand that tau_applied is not an entry of tau. It would otherwise get
  // overwritten.
  DRAKE_DEMAND(tau.data() != tau_applied.data());

  // The generalized forces on the mobilizer correspond to the active
  // components of the spatial force performing work. Therefore we need to
  // project F_BMo along the directions of motion.
  // Project as: tau = H_FMᵀ(q) * F_BMo_F, Eq. (4).
  mobilizer_->ProjectSpatialForce(context, F_BMo_F, tau);

  // Include the contribution of applied generalized forces.
  if (tau_applied.size() != 0) tau -= tau_applied;
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::
    CalcArticulatedBodyInertiaCache_TipToBase(
        const systems::Context<T>& context,
        const PositionKinematicsCache<T>& pc,
        const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
        const SpatialInertia<T>& M_B_W, const VectorX<T>& diagonal_inertias,
        ArticulatedBodyInertiaCache<T>* abic) const {
  DRAKE_DEMAND(mobod_index() != world_mobod_index());
  DRAKE_DEMAND(abic != nullptr);
  DRAKE_DEMAND(diagonal_inertias.size() ==
               this->get_parent_tree().num_velocities());

  // As a guideline for developers, a summary of the computations performed in
  // this method is provided:
  // Notation:
  //  - B body frame associated with this node.
  //  - P ("parent") body frame associated with this node's parent. This is
  //    not to be confused with the articulated body inertia, which is also
  //    named P.
  //  - C within a loop over children, one of body B's children.
  //  - P_B_W for the articulated body inertia of body B, about Bo, and
  //    expressed in world frame W.
  //  - Pplus_PB_W for the same articulated body inertia P_B_W but projected
  //    across B's inboard mobilizer to frame P so that instead of
  //    F_Bo_W = P_B_W A_WB + Z_Bo_W, we can write
  //    F_Bo_W = Pplus_PB_W Aplus_WB + Zplus_Bo_W where Aplus_WB is defined
  //    in Section 6.2.2, Page 101 of [Jain 2010] and Zplus_Bo_W is defined
  //    in Section 6.3, Page 108 of [Jain 2010].
  //  - Φ(p_PQ) for Jain's rigid body transformation operator. In code,
  //    V_MQ = Φᵀ(p_PQ) V_MP is equivalent to V_MP.Shift(p_PQ).
  //
  // The goal is to populate the articulated body cache with values necessary
  // for computing generalized accelerations in the second pass of the
  // articulated body algorithm. This computation is recursive, and assumes
  // that required articulated body quantities are already computed for all
  // children.
  //
  // We compute the articulated inertia of the current body by summing
  // contributions from all its children with its own spatial inertia. Note
  // that Φ is the is the rigid body shift operator as defined in [Jain 2010]
  // (Φ(P, Q) in Jain's book corresponds to Φ(p_PQ) in the notation used
  // here).
  //   P_B_W = Σᵢ(Φ(p_BCᵢ_W) Pplus_BCᵢ_W Φ(p_BCᵢ_W)ᵀ) + M_B_W
  //         = Σᵢ(Pplus_BCᵢb_W) + M_B_W                                   (1)
  // where Pplus_BCᵢb_W is the articulated body inertia P_Cᵢ_W of the child
  // body Cᵢ, projected across its inboard mobilizer to frame B, shifted to
  // frame B, and expressed in the world frame W.
  //
  // From P_B_W, we can obtain Pplus_PB_W by projecting the articulated body
  // inertia for this node across its mobilizer.
  //   Pplus_PB_W = (I - P_B_W H_PB_W (H_PB_Wᵀ P_B_W H_PB_W)⁻¹ H_PB_Wᵀ)
  //                  P_B_W                                               (2)
  // where H_PB_W is the hinge mapping matrix.
  //
  // A few quantities are required in the second pass. We write them out
  // explicitly so we can cache them and simplify the expression for P_PB_W.
  //   D_B = H_PB_Wᵀ P_B_W H_PB_W                                        (3)
  //   g_PB_W = P_B_W H_PB_W D_B⁻¹                                       (4)
  // where D_B is the articulated body hinge inertia and g_PB_W is the
  // Kalman gain.
  //
  // With D_B, it is possible to view equation (2) in another manner. D_B
  // relates mobility-space forces to mobility-space accelerations. We can
  // view Pplus_PB_W as subtracting the mobilizer space inertia that
  // results from expanding D_B into an articulated body inertia from B's
  // articulated body inertia.
  //
  // In order to reduce the number of computations, we can save the common
  // factor U_B_W = H_PB_Wᵀ P_B_W. We then can write:
  //   D_B = U_B_W H_PB_W                                                  (5)
  // and for g,
  //   g_PB_Wᵀ = (D_B⁻¹)ᵀ H_PB_Wᵀ P_B_Wᵀ
  //           = (D_Bᵀ)⁻¹ H_PB_Wᵀ P_B_W
  //           = D_B⁻¹ U_B_W                                               (6)
  // where we used the fact that both D and P are symmetric. Notice in the
  // last expression for g_PB_Wᵀ we are reusing the common factor U_B_W.
  //
  // Given the articulated body hinge inertia and Kalman gain, we can simplify
  // the equation in (2).
  //   Pplus_PB_W = (I - g_PB_W H_PB_Wᵀ) P_B_W
  //              = P_B_W - g_PB_W H_PB_Wᵀ P_B_W
  //              = P_B_W - g_PB_W * U_B_W                                 (7)

  // Compute articulated body inertia for body using (1).
  ArticulatedBodyInertia<T>& P_B_W = get_mutable_P_B_W(abic);
  P_B_W = ArticulatedBodyInertia<T>(M_B_W);

  // Add articulated body inertia contributions from all children.
  for (const BodyNode<T>* child : this->child_nodes()) {
    const MobodIndex child_node_index = child->mobod_index();
    // Shift vector p_CoBo_W.
    const Vector3<T>& p_BoCo_W = pc.get_p_PoBo_W(child_node_index);
    const Vector3<T> p_CoBo_W = -p_BoCo_W;

    // Pull Pplus_BC_W from cache (which is Pplus_PB_W for child).
    const ArticulatedBodyInertia<T>& Pplus_BC_W =
        abic->get_Pplus_PB_W(child_node_index);

    // Shift Pplus_BC_W to Pplus_BCb_W.
    // This is known to be one of the most expensive operations of ABA and
    // must not be overlooked. Refer to #12435 for details.
    const ArticulatedBodyInertia<T> Pplus_BCb_W = Pplus_BC_W.Shift(p_CoBo_W);

    // Add Pplus_BCb_W contribution to articulated body inertia.
    P_B_W += Pplus_BCb_W;
  }

  ArticulatedBodyInertia<T>& Pplus_PB_W = get_mutable_Pplus_PB_W(abic);
  Pplus_PB_W = P_B_W;

  // We now proceed to compute Pplus_PB_W using Eq. (7):
  //   Pplus_PB_W = P_B_W - g_PB_W * U_B_W
  // For weld joints (with kNv = 0) or locked joints, terms involving the hinge
  // matrix H_PB_W go away and therefore Pplus_PB_W = P_B_W. We check this
  // below.
  if (kNv != 0 && !mobilizer_->is_locked(context)) {
    // Compute common term U_B_W.
    const MatrixUpTo6<T> U_B_W = H_PB_W.transpose() * P_B_W;

    // Compute the articulated body hinge inertia, D_B, using (5).
    MatrixUpTo6<T> D_B(kNv, kNv);
    D_B.template triangularView<Eigen::Lower>() = U_B_W * H_PB_W;

    // Include the effect of additional diagonal inertias. See @ref
    // additional_diagonal_inertias.
    D_B.diagonal() +=
        diagonal_inertias.segment(mobilizer_->velocity_start_in_v(), kNv);

    // Compute the LLT factorization of D_B as llt_D_B.
    math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& llt_D_B =
        get_mutable_llt_D_B(abic);
    this->CalcArticulatedBodyHingeInertiaMatrixFactorization(D_B, &llt_D_B);

    // Compute the Kalman gain, g_PB_W, using (6).
    Matrix6xUpTo6<T>& g_PB_W = get_mutable_g_PB_W(abic);
    g_PB_W = llt_D_B.Solve(U_B_W).transpose();

    // Project P_B_W using (7) to obtain Pplus_PB_W, the articulated body
    // inertia of this body B as felt by body P and expressed in frame W.
    // Symmetrize the computation to reduce floating point errors.
    // TODO(amcastro-tri): Notice that the line below makes the implicit
    //  assumption that g_PB_W * U_B_W is SPD and only the lower triangular
    //  portion is used, see the documentation for ArticulatedBodyInertia's
    //  constructor (checked only during Debug builds). This
    //  *might* result in the accumulation of floating point round off errors
    //  for long kinematic chains. Further investigation is required.
    Pplus_PB_W -= ArticulatedBodyInertia<T>(g_PB_W * U_B_W);
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::
    CalcArticulatedBodyForceCache_TipToBase(
        const systems::Context<T>& context,
        const PositionKinematicsCache<T>& pc, const VelocityKinematicsCache<T>*,
        const SpatialForce<T>& Fb_Bo_W,
        const ArticulatedBodyInertiaCache<T>& abic,
        const SpatialForce<T>& Zb_Bo_W, const SpatialForce<T>& Fapplied_Bo_W,
        const Eigen::Ref<const VectorX<T>>& tau_applied,
        const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
        ArticulatedBodyForceCache<T>* aba_force_cache) const {
  DRAKE_DEMAND(mobod_index() != world_mobod_index());
  DRAKE_DEMAND(aba_force_cache != nullptr);

  // As a guideline for developers, please refer to @ref
  // internal_forward_dynamics for a detailed description of the algorithm and
  // notation in use.

  // Compute the residual spatial force, Z_Bo_W, according to (1).
  SpatialForce<T> Z_Bo_W = Fb_Bo_W - Fapplied_Bo_W;

  // Add residual spatial force contributions from all children.
  for (const BodyNode<T>* child : this->child_nodes()) {
    const MobodIndex child_node_index = child->mobod_index();
    // Shift vector from Co to Bo.
    const Vector3<T>& p_BoCo_W = pc.get_p_PoBo_W(child_node_index);
    const Vector3<T> p_CoBo_W = -p_BoCo_W;

    // Pull Zplus_BC_W from cache (which is Zplus_PB_W for child).
    const SpatialForce<T>& Zplus_BC_W =
        aba_force_cache->get_Zplus_PB_W(child_node_index);

    // Shift Zplus_BC_W to Zplus_BCb_W.
    const SpatialForce<T> Zplus_BCb_W = Zplus_BC_W.Shift(p_CoBo_W);

    // Add Zplus_BCb_W contribution to residual spatial force.
    Z_Bo_W += Zplus_BCb_W;
  }

  get_mutable_Zplus_PB_W(aba_force_cache) = Z_Bo_W + Zb_Bo_W;

  // These terms do not show up for zero mobilities (weld or locked).
  if (kNv != 0 && !mobilizer_->is_locked(context)) {
    // Compute the articulated body inertia innovations generalized force,
    // e_B, according to (4).
    VectorUpTo6<T>& e_B = get_mutable_e_B(aba_force_cache);
    e_B.noalias() = tau_applied - H_PB_W.transpose() * Z_Bo_W.get_coeffs();

    // Get the Kalman gain from cache.
    const Matrix6xUpTo6<T>& g_PB_W = get_g_PB_W(abic);

    // Compute the projected articulated body force bias Zplus_PB_W.
    get_mutable_Zplus_PB_W(aba_force_cache) += SpatialForce<T>(g_PB_W * e_B);
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::
    CalcArticulatedBodyAccelerations_BaseToTip(
        const systems::Context<T>& context,
        const PositionKinematicsCache<T>& pc,
        const ArticulatedBodyInertiaCache<T>& abic,
        const ArticulatedBodyForceCache<T>& aba_force_cache,
        const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
        const SpatialAcceleration<T>& Ab_WB,
        AccelerationKinematicsCache<T>* ac) const {
  DRAKE_THROW_UNLESS(ac != nullptr);

  // As a guideline for developers, please refer to @ref
  // abi_computing_accelerations for a detailed description of the algorithm
  // and the notation in use.

  // Get the spatial acceleration of the parent.
  const SpatialAcceleration<T>& A_WP =
      ac->get_A_WB(mobilizer_->mobod().inboard());

  // Shift vector p_PoBo_W from the parent origin to the body origin.
  const Vector3<T>& p_PoBo_W = get_p_PoBo_W(pc);

  // Rigidly shift the acceleration of the parent node.
  const SpatialAcceleration<T> Aplus_WB = SpatialAcceleration<T>(
      A_WP.rotational(),
      A_WP.translational() + A_WP.rotational().cross(p_PoBo_W));

  SpatialAcceleration<T>& A_WB = get_mutable_A_WB(ac);
  A_WB = Aplus_WB + Ab_WB;

  // These quantities do not contribute when nv = 0 (weld or locked joint). We
  // skip them since Eigen does not allow certain operations on zero-sized
  // objects. It is important to set the generalized accelerations to zero for
  // locked mobilizers.
  if (mobilizer_->is_locked(context)) {
    get_mutable_accelerations(ac).setZero();
  } else if (kNv != 0) {
    // Compute nu_B, the articulated body inertia innovations generalized
    // acceleration.
    const VectorUpTo6<T> nu_B =
        get_llt_D_B(abic).Solve(get_e_B(aba_force_cache));

    // Mutable reference to the generalized acceleration.
    auto vmdot = get_mutable_accelerations(ac);
    const Matrix6xUpTo6<T>& g_PB_W = get_g_PB_W(abic);
    vmdot = nu_B - g_PB_W.transpose() * A_WB.get_coeffs();

    // Update with vmdot term the spatial acceleration of the current body.
    A_WB += SpatialAcceleration<T>(H_PB_W * vmdot);
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcCompositeBodyInertia_TipToBase(
    const SpatialInertia<T>& M_B_W, const PositionKinematicsCache<T>& pc,
    const std::vector<SpatialInertia<T>>& Mc_B_W_all,
    SpatialInertia<T>* Mc_B_W) const {
  DRAKE_DEMAND(mobod_index() != world_mobod_index());
  DRAKE_DEMAND(Mc_B_W != nullptr);

  // Composite body inertia R_B_W for this node B, about its frame's origin
  // Bo, and expressed in the world frame W. Here we adopt the notation used
  // in Jain's book.
  *Mc_B_W = M_B_W;
  // Add composite body inertia contributions from all children.
  for (const BodyNode<T>* child : this->child_nodes()) {
    const MobodIndex child_node_index = child->mobod_index();
    // Shift vector p_CoBo_W.
    const Vector3<T>& p_BoCo_W = pc.get_p_PoBo_W(child_node_index);
    const Vector3<T> p_CoBo_W = -p_BoCo_W;

    // Composite body inertia for outboard child body C, about Co, expressed
    // in W.
    const SpatialInertia<T>& Mc_CCo_W = Mc_B_W_all[child_node_index];

    // Shift to Bo and add it to the composite body inertia of B.
    *Mc_B_W += Mc_CCo_W.Shift(p_CoBo_W);
  }
}

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcSpatialAccelerationBias(
    const systems::Context<T>& context,
    const FrameBodyPoseCache<T>& frame_body_pose_cache,
    const PositionKinematicsCache<T>& pc, const VelocityKinematicsCache<T>& vc,
    SpatialAcceleration<T>* Ab_WB) const {
  DRAKE_THROW_UNLESS(Ab_WB != nullptr);
  // As a guideline for developers, please refer to @ref
  // abi_computing_accelerations for a detailed description and derivation of
  // Ab_WB.

  Ab_WB->SetZero();
  // Inboard frame F and outboard frame M of this node's mobilizer.
  const Frame<T>& frame_F = inboard_frame();
  const Frame<T>& frame_M = outboard_frame();

  // Compute R_PF and X_MB.
  const math::RigidTransform<T>& X_PF =
      frame_F.get_X_BF(frame_body_pose_cache);  // B==P
  const math::RotationMatrix<T>& R_PF = X_PF.rotation();
  const math::RigidTransform<T>& X_MB =
      frame_M.get_X_FB(frame_body_pose_cache);  // F==M

  // Parent position in the world is available from the position kinematics.
  const math::RotationMatrix<T>& R_WP = get_R_WP(pc);

  // TODO(amcastro-tri): consider caching R_WF.
  const math::RotationMatrix<T> R_WF = R_WP * R_PF;

  // Compute shift vector p_MoBo_F.
  const Vector3<T> p_MoBo_F = get_X_FM(pc).rotation() * X_MB.translation();

  // The goal is to compute Ab_WB = Ac_WB + Ab_PB_W, see @ref
  // abi_computing_accelerations.
  // We first compute Ab_PB_W and add Ac_WB at the end.

  // We are ultimately trying to compute Ab_WB = Ac_WB + Ab_PB_W, of which
  // Ab_PB (that we're computing here) is a component. See @ref
  // abi_computing_accelerations. Now, Ab_PB_W is the bias term for the
  // acceleration A_PB_W. That is, it is the acceleration A_PB_W when vmdot is
  // zero. We get Ab_PB_W by shifting and re-expressing Ab_FM, the across
  // mobilizer spatial acceleration bias term.

  // We first compute the acceleration bias Ab_FM = Hdot * vm.
  // Note, A_FM = H_FM(qm) * vmdot + Ab_FM(qm, vm).
  const VectorUpTo6<T> vmdot_zero = VectorUpTo6<T>::Zero(kNv);
  const SpatialAcceleration<T> Ab_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(context, vmdot_zero);

  // Due to the fact that frames P and F are on the same rigid body, we have
  // that V_PF = 0. Therefore, DtP(V_PB) = DtF(V_PB). Since M and B are also
  // on the same rigid body, V_MB = 0. We then recognize that V_PB = V_PFb +
  // V_FMb + V_MB = V_FMb. Together, we get that A_PB = DtF(V_FMb) =
  // A_FM.Shift(p_MoBo, w_FM).
  const Vector3<T> w_FM = get_V_FM(vc).rotational();
  const SpatialAcceleration<T> Ab_PB_W = R_WF * Ab_FM.Shift(p_MoBo_F, w_FM);

  // Spatial velocity of parent is available from the velocity kinematics.
  const SpatialVelocity<T>& V_WP = get_V_WP(vc);
  const Vector3<T>& w_WP = V_WP.rotational();
  const Vector3<T>& v_WP = V_WP.translational();

  // Velocity of this mobilized body in its parent is available from the
  // velocity kinematics.
  const SpatialVelocity<T>& V_PB_W = get_V_PB_W(vc);
  const Vector3<T>& w_PB_W = V_PB_W.rotational();
  const Vector3<T>& v_PB_W = V_PB_W.translational();

  // Mobilized body spatial velocity in W.
  const SpatialVelocity<T>& V_WB = get_V_WB(vc);
  const Vector3<T>& w_WB = V_WB.rotational();
  const Vector3<T>& v_WB = V_WB.translational();

  // Compute Ab_WB according to:
  // Ab_WB =  | w_WB x w_PB_W                 | + Ab_PB_W
  //          | w_WP x (v_WB - v_WP + v_PB_W) |
  // N.B: It is NOT true that v_PB_W = v_WB - v_WP, since you would otherwise
  // be forgetting to shift v_WP to B. We prefer the expression used here
  // since it is cheaper to compute, requiring only a single cross product,
  // see @note in SpatialAcceleration::ComposeWithMovingFrameAcceleration()
  // for a complete derivation.
  *Ab_WB = SpatialAcceleration<T>(
      w_WB.cross(w_PB_W) + Ab_PB_W.rotational(),
      w_WP.cross(v_WB - v_WP + v_PB_W) + Ab_PB_W.translational());
}

// This method computes the total force Ftot_BBo on body B that must be
// applied for it to incur in a spatial acceleration A_WB. Mathematically:
//   Ftot_BBo = M_B_W * A_WB + b_Bo
// where b_Bo contains the velocity dependent gyroscopic terms (see Eq. 2.26,
// p. 27, in A. Jain's book). The above balance is performed at the origin
// Bo of the body frame B, which does not necessarily need to coincide with
// the body center of mass.
// Notes:
//   1. Ftot_BBo = b_Bo when A_WB = 0.
//   2. b_Bo = 0 when w_WB = 0.
//   3. b_Bo.translational() = 0 when Bo = Bcm (p_BoBcm = 0).
//      When Fb_Bo_W_cache is nullptr velocities are considered to be zero.
//      Therefore, from (2), the bias term is assumed to be zero and is not
//      computed.
template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::
    CalcBodySpatialForceGivenItsSpatialAcceleration(
        const std::vector<SpatialInertia<T>>& M_B_W_cache,
        const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
        const SpatialAcceleration<T>& A_WB,
        SpatialForce<T>* Ftot_BBo_W_ptr) const {
  DRAKE_DEMAND(Ftot_BBo_W_ptr != nullptr);

  // Output spatial force applied on mobilized body B, at Bo, measured in W.
  SpatialForce<T>& Ftot_BBo_W = *Ftot_BBo_W_ptr;

  // RigidBody for this node.
  const RigidBody<T>& body_B = body();

  // Mobilized body B spatial inertia about Bo expressed in world W.
  const SpatialInertia<T>& M_B_W = M_B_W_cache[body_B.mobod_index()];

  // Equations of motion for a rigid body written at a generic point Bo not
  // necessarily coincident with the body's center of mass. This corresponds
  // to Eq. 2.26 (p. 27) in A. Jain's book.
  Ftot_BBo_W = M_B_W * A_WB;

  // If velocities are zero, then Fb_Bo_W is zero and does not contribute.
  if (Fb_Bo_W_cache != nullptr) {
    // Dynamic bias for body B.
    const SpatialForce<T>& Fb_Bo_W = (*Fb_Bo_W_cache)[body_B.mobod_index()];
    Ftot_BBo_W += Fb_Bo_W;
  }
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
