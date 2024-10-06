#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/body_node.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

// For internal use only of the MultibodyTree implementation.
// While all code that is common to any node _could_ be placed in the BodyNode
// class, BodyNodeImpl is templatized on the concrete Mobilizer type so
// implementations here can use fixed-size objects and mobilizer-specific
// inline implementations for maximum speed. For a more detailed discussion of
// the role of a BodyNode in a MultibodyTree refer to the class documentation
// for BodyNode.
template <typename T, template <typename> class ConcreteMobilizer>
class BodyNodeImpl final : public BodyNode<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNodeImpl);

  // Inherit sizes from the concrete mobilizer.
  enum : int {
    kNq = ConcreteMobilizer<T>::kNq,
    kNv = ConcreteMobilizer<T>::kNv,
    kNx = ConcreteMobilizer<T>::kNx
  };
  using QVector = typename ConcreteMobilizer<T>::QVector;
  using VVector = typename ConcreteMobilizer<T>::VVector;
  using HMatrix = typename ConcreteMobilizer<T>::HMatrix;

  using BodyNode<T>::mobod_index;
  using BodyNode<T>::inboard_mobod_index;
  using BodyNode<T>::inboard_frame;
  using BodyNode<T>::outboard_frame;
  using BodyNode<T>::body;
  using BodyNode<T>::parent_body;

  // Given a body and its inboard mobilizer in a MultibodyTree this constructor
  // creates the corresponding %BodyNode. See the BodyNode class documentation
  // for details on how a BodyNode is defined.
  // @param[in] parent_node
  //   A const pointer to the parent BodyNode object in the tree structure of
  //   the owning MultibodyTree.
  // @param[in] body The body B associated with `this` node. Can't be World.
  // @param[in] mobilizer The mobilizer associated with this `node`.
  //
  // @note World is handled specially and does not go through this path. There
  //   is a world BodyNode and dummy world Mobilizer 0; see MultibodyTree
  //   CreateJointImplementations() and CreateBodyNode(). The BodyNode
  //   constructor used here will abort if we try this with World.
  BodyNodeImpl(const internal::BodyNode<T>* parent_node,
               const RigidBody<T>* body, const Mobilizer<T>* mobilizer);

  ~BodyNodeImpl() final;

  // TODO(sherm1) Just a warm up -- move the rest of the kernel computations
  //  here also.

  void CalcPositionKinematicsCache_BaseToTip(
      const FrameBodyPoseCache<T>& frame_body_pose_cache, const T* positions,
      PositionKinematicsCache<T>* pc) const final;

  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context,
      const FrameBodyPoseCache<T>& frame_body_pose_cache,
      const PositionKinematicsCache<T>& pc,
      std::vector<Vector6<T>>* H_PB_W_cache) const final;

  void CalcVelocityKinematicsCache_BaseToTip(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const std::vector<Vector6<T>>& H_PB_W_cache, const T* velocities,
      VelocityKinematicsCache<T>* vc) const final;

  void CalcSpatialAcceleration_BaseToTip(
      const systems::Context<T>& context,
      const FrameBodyPoseCache<T>& frame_body_poses_cache,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>* vc, const VectorX<T>& mbt_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array_ptr) const final;

  void CalcInverseDynamics_TipToBase(
      const systems::Context<T>& context,
      const FrameBodyPoseCache<T>& frame_body_pose_cache,
      const PositionKinematicsCache<T>& pc,
      const std::vector<SpatialInertia<T>>& M_B_W_cache,
      const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
      const std::vector<SpatialAcceleration<T>>& A_WB_array,
      const SpatialForce<T>& Fapplied_Bo_W,
      const Eigen::Ref<const VectorX<T>>& tau_applied,
      std::vector<SpatialForce<T>>* F_BMo_W_array_ptr,
      EigenPtr<VectorX<T>> tau_array) const final;

  void CalcArticulatedBodyInertiaCache_TipToBase(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      const SpatialInertia<T>& M_B_W, const VectorX<T>& diagonal_inertias,
      ArticulatedBodyInertiaCache<T>* abic) const final;

  void CalcArticulatedBodyForceCache_TipToBase(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>*, const SpatialForce<T>& Fb_Bo_W,
      const ArticulatedBodyInertiaCache<T>& abic,
      const SpatialForce<T>& Zb_Bo_W, const SpatialForce<T>& Fapplied_Bo_W,
      const Eigen::Ref<const VectorX<T>>& tau_applied,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      ArticulatedBodyForceCache<T>* aba_force_cache) const final;

  void CalcArticulatedBodyAccelerations_BaseToTip(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const ArticulatedBodyInertiaCache<T>& abic,
      const ArticulatedBodyForceCache<T>& aba_force_cache,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      const SpatialAcceleration<T>& Ab_WB,
      AccelerationKinematicsCache<T>* ac) const final;

  void CalcCompositeBodyInertia_TipToBase(
      const SpatialInertia<T>& M_B_W, const PositionKinematicsCache<T>& pc,
      const std::vector<SpatialInertia<T>>& Mc_B_W_all,
      SpatialInertia<T>* Mc_B_W) const final;

  void CalcSpatialAccelerationBias(
      const systems::Context<T>& context,
      const FrameBodyPoseCache<T>& frame_body_pose_cache,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      SpatialAcceleration<T>* Ab_WB) const final;

 private:
  // Given a pointer to the contiguous array of all q's in this system, returns
  // a pointer to the ones for this mobilizer.
  // @pre `positions` is the full set of q's for this system
  const T* get_q(const T* positions) const {
    return &positions[mobilizer().position_start_in_q()];
  }

  // Returns this mobilizer's qs as a fixed-size QVector.
  Eigen::Map<const QVector> get_qvector(const T* positions) const {
    return Eigen::Map<const QVector>(get_q(positions));
  }

  // Given a pointer to the contiguous array of all v's in this system, returns
  // a pointer to the ones for this mobilizer.
  // @pre `velocities` is the full set of v's for this system
  const T* get_v(const T* velocities) const {
    return &velocities[mobilizer().velocity_start_in_v()];
  }

  // Returns this mobilizer's vs as a fixed-size VVector.
  Eigen::Map<const VVector> get_vvector(const T* velocities) const {
    return Eigen::Map<const VVector>(get_v(velocities));
  }

  // Given a complete array of hinge matrices H stored by contiguous columns,
  // returns a const reference to H for this mobilizer, as a 6xnv fixed-size
  // matrix. This matrix is 16-byte aligned because it is composed of
  // columns of Eigen::Vector6 objects which Eigen aligns.
  Eigen::Map<const HMatrix, Eigen::Aligned16> get_H(
      const std::vector<Vector6<T>>& H_cache) const {
    return Eigen::Map<const HMatrix, Eigen::Aligned16>(
        H_cache[mobilizer().velocity_start_in_v()].data());
  }

  // Given a pointer to a mutable complete array of hinge matrices H stored by
  // contiguous columns, returns a mutable reference to H for this mobilizer,
  // as a 6xnv fixed-size matrix. This matrix is 16-byte aligned because it is
  // composed of columns of Eigen::Vector6 objects which Eigen aligns.
  Eigen::Map<HMatrix, Eigen::Aligned16> get_mutable_H(
      std::vector<Vector6<T>>* H_cache) const {
    DRAKE_ASSERT(H_cache != nullptr);
    return Eigen::Map<HMatrix, Eigen::Aligned16>(
        (*H_cache)[mobilizer().velocity_start_in_v()].data());
  }

  const ConcreteMobilizer<T>& mobilizer() const {
    DRAKE_ASSERT(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  // =========================================================================
  // PositionKinematicsCache Accessors and Mutators.

  // Returns a const reference to the pose of the body B associated with this
  // node as measured and expressed in the world frame W.
  const math::RigidTransform<T>& get_X_WB(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(mobod_index());
  }

  // Mutable version of get_X_WB().
  math::RigidTransform<T>& get_mutable_X_WB(
      PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(mobod_index());
  }

  // Returns a const reference to the pose of the parent body P measured and
  // expressed in the world frame W.
  const math::RigidTransform<T>& get_X_WP(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(inboard_mobod_index());
  }

  // Returns a const reference to the rotation matrix `R_WP` that relates the
  // orientation of the world frame W to the parent frame P.
  const math::RotationMatrix<T>& get_R_WP(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_R_WB(inboard_mobod_index());
  }

  // Returns a constant reference to the across-mobilizer pose of the outboard
  // frame M as measured and expressed in the inboard frame F.
  const math::RigidTransform<T>& get_X_FM(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_FM(mobod_index());
  }

  // Returns a mutable reference to the across-mobilizer pose of the outboard
  // frame M as measured and expressed in the inboard frame F.
  math::RigidTransform<T>& get_mutable_X_FM(
      PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_FM(mobod_index());
  }

  // Returns a mutable reference to the pose of body B as measured and expressed
  // in the frame of the parent body P.
  math::RigidTransform<T>& get_mutable_X_PB(
      PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(mobod_index());
  }

  const Vector3<T>& get_p_PoBo_W(const PositionKinematicsCache<T>& pc) const {
    return pc.get_p_PoBo_W(mobod_index());
  }

  Vector3<T>& get_mutable_p_PoBo_W(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_p_PoBo_W(mobod_index());
  }

  SpatialVelocity<T>& get_mutable_V_WB(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_WB(mobod_index());
  }

  SpatialVelocity<T>& get_mutable_V_FM(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_FM(mobod_index());
  }

  SpatialVelocity<T>& get_mutable_V_PB_W(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_PB_W(mobod_index());
  }

  // For the body B associated with this node, return V_WB, B's spatial velocity
  // in the world frame W, expressed in W (for Bo, the body frame's origin).
  const SpatialVelocity<T>& get_V_WB(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(mobod_index());
  }

  // Returns the spatial velocity `V_WP` of the body frame P in the parent node
  // as measured and expressed in the world frame.
  const SpatialVelocity<T>& get_V_WP(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(inboard_mobod_index());
  }

  // Returns a const reference to the across-mobilizer spatial velocity `V_FM`
  // of the outboard frame M in the inboard frame F, expressed in the F frame.
  const SpatialVelocity<T>& get_V_FM(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_FM(mobod_index());
  }

  // Returns a const reference to the spatial velocity `V_PB_W` of `this`
  // node's body B in the parent node's body P, expressed in the world frame W.
  const SpatialVelocity<T>& get_V_PB_W(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_PB_W(mobod_index());
  }

  // =========================================================================
  // AccelerationKinematicsCache Accessors and Mutators.

  // For the body B associated with `this` node, returns A_WB, body B's
  // spatial acceleration in the world frame W, expressed in W
  // (for point Bo, the body's origin).
  const SpatialAcceleration<T>& get_A_WB(
      const AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(mobod_index());
  }

  // Mutable version of get_A_WB().
  SpatialAcceleration<T>& get_mutable_A_WB(
      AccelerationKinematicsCache<T>* ac) const {
    return ac->get_mutable_A_WB(mobod_index());
  }

  // Helper to get an Eigen expression of the vector of generalized velocities
  // from a vector of generalized velocities for the entire parent multibody
  // tree. Useful for the implementation of operator forms where the generalized
  // velocity (or time derivatives of the generalized velocities) is an argument
  // to the operator.
  Eigen::Ref<VectorX<T>> get_mutable_velocities_from_array(
      EigenPtr<VectorX<T>> v) const {
    DRAKE_ASSERT(v != nullptr);
    return v->segment(mobilizer().velocity_start_in_v(), kNv);
  }

  // Returns an Eigen expression of the vector of generalized accelerations
  // for this node's inboard mobilizer from the vector of generalized
  // accelerations for the entire model.
  Eigen::Ref<VectorX<T>> get_mutable_accelerations(
      AccelerationKinematicsCache<T>* ac) const {
    VectorX<T>& vdot = ac->get_mutable_vdot();
    return get_mutable_velocities_from_array(&vdot);
  }

  // =========================================================================
  // ArticulatedBodyInertiaCache Accessors and Mutators.

  // Returns a reference to the articulated body inertia `P_B_W` of the
  // body taken about Bo and expressed in W.
  ArticulatedBodyInertia<T>& get_mutable_P_B_W(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_P_B_W(mobod_index());
  }

  // Returns a reference to the articulated body inertia `Pplus_PB_W`,
  // which can be thought of as the articulated body inertia of parent body P
  // as though it were inertialess, but taken about Bo and expressed in W.
  ArticulatedBodyInertia<T>& get_mutable_Pplus_PB_W(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_Pplus_PB_W(mobod_index());
  }

  // =========================================================================
  // Per Node Array Accessors.
  // Quantities are ordered by MobodIndex unless otherwise specified.

  // Returns a const reference to the spatial acceleration of the body B
  // associated with this node as measured and expressed in the world frame W,
  // given an array of spatial accelerations for the entire MultibodyTree model.
  const SpatialAcceleration<T>& get_A_WB_from_array(
      const std::vector<SpatialAcceleration<T>>& A_WB_array) const {
    return A_WB_array[mobod_index()];
  }

  // Mutable version of get_A_WB_from_array().
  SpatialAcceleration<T>& get_mutable_A_WB_from_array(
      std::vector<SpatialAcceleration<T>>* A_WB_array) const {
    DRAKE_ASSERT(A_WB_array != nullptr);
    return (*A_WB_array)[mobod_index()];
  }

  // Returns a const reference to the spatial acceleration `A_WP` of the body
  // frame P in the parent node as measured and expressed in the world frame,
  // given an array of spatial accelerations for the entire MultibodyTree model.
  const SpatialAcceleration<T>& get_A_WP_from_array(
      const std::vector<SpatialAcceleration<T>>& A_WB_array) const {
    return A_WB_array[inboard_mobod_index()];
  }

  // Returns a const reference to the LLT factorization `llt_D_B` of the
  // articulated body hinge inertia.
  const math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& get_llt_D_B(
      const ArticulatedBodyInertiaCache<T>& abic) const {
    return abic.get_llt_D_B(mobod_index());
  }

  // Mutable version of get_llt_D_B().
  math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& get_mutable_llt_D_B(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_llt_D_B(mobod_index());
  }

  // Returns a const reference to the Kalman gain `g_PB_W` of the body.
  const Matrix6xUpTo6<T>& get_g_PB_W(
      const ArticulatedBodyInertiaCache<T>& abic) const {
    return abic.get_g_PB_W(mobod_index());
  }

  // Mutable version of get_g_PB_W().
  Matrix6xUpTo6<T>& get_mutable_g_PB_W(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_g_PB_W(mobod_index());
  }

  // =========================================================================
  // ArticulatedBodyForceCache Accessors and Mutators.

  // Returns a const reference to the articulated body inertia residual force
  // `Zplus_PB_W` for this body projected across its inboard mobilizer to
  // frame P.
  SpatialForce<T>& get_mutable_Zplus_PB_W(
      ArticulatedBodyForceCache<T>* aba_force_cache) const {
    return aba_force_cache->get_mutable_Zplus_PB_W(mobod_index());
  }

  // Returns a const reference to the Coriolis spatial acceleration `Ab_WB`
  // for this body due to the relative velocities of body B and body P.
  const VectorUpTo6<T>& get_e_B(
      const ArticulatedBodyForceCache<T>& aba_force_cache) const {
    return aba_force_cache.get_e_B(mobod_index());
  }

  // Mutable version of get_e_B().
  VectorUpTo6<T>& get_mutable_e_B(
      ArticulatedBodyForceCache<T>* aba_force_cache) const {
    return aba_force_cache->get_mutable_e_B(mobod_index());
  }

  // Computes the total force Ftot_BBo on body B that must be applied for it to
  // incur in a spatial acceleration A_WB.
  void CalcBodySpatialForceGivenItsSpatialAcceleration(
      const std::vector<SpatialInertia<T>>& M_B_W_cache,
      const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
      const SpatialAcceleration<T>& A_WB,
      SpatialForce<T>* Ftot_BBo_W_ptr) const;

  const ConcreteMobilizer<T>* const mobilizer_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
