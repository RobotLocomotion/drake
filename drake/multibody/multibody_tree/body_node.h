#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

/// For internal use only of the MultibodyTree implementation.
/// This is a base class representing a **node** in the tree structure of a
/// MultibodyTree. A %BodyNode is responsible for providing fast implementations
/// used within the recursive algorithms implemented by MultibodyTree. While
/// %BodyNode provides a general API to be used from withing MultibodyTree,
/// its derived class BodyNodeImpl has compile-time fixed sizes so that all
/// operations can be perfomed with fixed-size stack-allocated Eigen variables.
///
/// A %BodyNode can be thought of as a **computational cell** in the
/// MultibodyTree associated with a given body B and an inboard mobilizer
/// that connects this body B to the rest of the tree. The unique parent body of
/// body B is denoted by P, which in turns has its own %BodyNode associated with
/// it. Associated with each %BodyNode, there will be an inboard frame F
/// attached on body P and an outboard frame M attached to body B. The
/// relationship between frames F and M is dictated by the node's mobilizer
/// providing the pose `X_FM(q)` as a function of the generalized coordinates
/// `q` for that mobilizer.
/// In summary, a %BodyNode is a computational cell encompassing:
/// - A body B in a given MultibodyTree,
/// - the outboard frame M attached to this body B,
/// - the inboard frame F attached to the unique parent body P of body B,
/// - the mobilizer connecting the inboard mobilizer F with the outboard
///   frame M.
///
/// %BodyNode provides fast implementation for convinience methods to be used
/// in MultibodyTree recursive algorithms but that however should not leak into
/// the public API for the Mobilizer class. In this regard, %BodyNode provides
/// an additional separation layer between implementation internals and user's
/// API.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class BodyNode : public MultibodyTreeElement<BodyNode<T>, BodyNodeIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNode)

  /// A node is a computational cell encompassing a Body in a MultibodyTree
  /// and the inboard Mobilizer that connects this body to the tree. Given a
  /// body and its inboard mobilizer in a MultibodyTree this constructor
  /// creates the corresponding %BodyNode. See this class' documentation for
  /// details on how a %BodyNode is defined.
  /// @param[in] body The body B associated with `this` node. It must be a valid
  ///                 pointer.
  /// @param[in] mobilizer The mobilizer associated with this `node`. It can
  ///                      only be a `nullptr` for the **world** body.
  BodyNode(const Body<T>* body, const Mobilizer<T>* mobilizer) :
      body_(body), mobilizer_(mobilizer) {
    DRAKE_DEMAND(body != nullptr);
    DRAKE_DEMAND(!(mobilizer == nullptr && body->get_index() != world_index()));
  }

  /// Returns a constant reference to the body B associated with this node.
  const Body<T>& get_body() const {
    return *body_;
  }

  /// Returns a constant reference to the unique parent body P of the body B
  /// associated with this node.
  const Body<T>& get_parent_body() const {
    DRAKE_ASSERT(get_parent_body_index().is_valid());
    return this->get_parent_tree().get_body(get_parent_body_index());
  }

  /// Returns a constant reference to the mobilizer associated with this node.
  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_ASSERT(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  /// Returns the inboard frame F of this node's mobilizer.
  const Frame<T>& get_inboard_frame() const {
    return get_mobilizer().get_inboard_frame();
  }

  /// Returns the outboard frame M of this node's mobilizer.
  const Frame<T>& get_outboard_frame() const {
    return get_mobilizer().get_outboard_frame();
  }

  /// This method is used by MultibodyTree within a base-to-tip loop to compute
  /// this node's kinematics that only depend on generalized coordinates
  /// positions.
  /// @pre CalcPositionKinematicsCache_BaseToTip() must have already been called
  /// for the parent node.
  void CalcPositionKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {
    // This method should not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != world_index());

    // This computes into the PositionKinematicsCache:
    // - X_PB(qb_P, qm_B, qb_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    // where qb_P are the generalized coordinates associated with body P, qm_B
    // the generalized coordinates associated with this node's mobilizer and
    // qb_B the generalized coordinates associated with body B.
    // It assumes:
    // - Body B already updated the pose `X_BM(qb_B)` of the inboard
    //   mobilizer M.
    // - We are in a base-to-tip recursion and therefore `X_PF(qb_P)` and `X_WP`
    //   are were already updated.
    CalcAcrossMobilizerBodyPoses_BaseToTip(context, pc);

    // TODO(amcastro-tri):
    // Update Body specific kinematics. These include:
    // - p_PB_W: vector from P to B to perform shift operations.
    // - com_W: center of mass.
    // - M_Bo_W: Spatial inertia.

    // TODO(amcastro-tri):
    // With H_FM(qr) already in the cache (computed by
    // Mobilizer::UpdatePositionKinematicsCache()) update the cache
    // entries for H_PB_W, the Jacobian for the SpatialVelocity jump between
    // body B and its parent body P expressed in the world frame W.
  }

  void CalcVelocityKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      VelocityKinematicsCache<T>* vc) const {

    // Body for this node.
    const Body<T>& BodyB = get_body();

    // Body for this node's parent, or the parent body P.
    const Body<T>& BodyP = get_parent_body();

    // Inboard/Outboard frames of this node's mobilizer.
    const Frame<T>& FrameF = get_inboard_frame();
    DRAKE_ASSERT(FrameF.get_body().get_index() == BodyP.get_index());
    const Frame<T>& FrameM = get_outboard_frame();
    DRAKE_ASSERT(FrameM.get_body().get_index() == BodyB.get_index());

    // Generalized velocities local to this node's mobilizer.
    const auto& vm = this->get_mobilizer_velocities(context);

    // Operator V_FM = H_FM * vm
    SpatialVelocity<T> V_FM =
        get_mobilizer().CalcAcrossMobilizerSpatialVelocity(context, vm);

    // P: Parent body frame.
    // F: Mobilizer inboard frame.
    // M: Mobilizer outboard frame.
    const Isometry3<T> X_PF = FrameF.CalcPoseInBodyFrame(context);
    const Isometry3<T> X_MB = FrameM.CalcBodyPoseInThisFrame(context);

    // Pose of the parent body P in world frame W.
    // Available since we are called within a base-to-tip recursion.
    const Isometry3<T>& X_WP = get_X_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    const Matrix3<T> R_WF = X_WP.rotation() * X_PF.rotation();

    // TODO(amcastro-tri): incorporate additional terms for flexible bodies.
    // In terms of spatial velocities, for a RIGID parent body P:
    //    - V_PB = V_FB,  since V_PF = 0.
    // For a RIGID body B, we can use rigid shift operator:
    //    - V_FB = phiT_MB * V_FM = V_FM.Shift(p_MB)
    // And therefore:
    //    - V_PB = V_FB = phiT_MB * V_FM = V_FM.Shift(p_MB)
    // Or, equivalently, in terms of velocity Jacobians:
    //    - V_PB = phiT_MB * H_FM * vm = H_PB * vm, with H_PB = phiT_MB * H_FM.
    //
    // It is convenient now to perform this last computations in the F frame
    // given the available pre-computed quantities.
    // Vector from Mo to Bo expressed in frame F:
    const Vector3<T> p_MB_F =
        /* p_MB_F = R_FM * p_MB_M */
        get_X_FM(pc).rotation() * X_MB.translation();

    // Perform V_PB = phiT_MB * V_FM in the F frame and re-express in the
    // world frame W.
    // In operator form:
    //   V_PB_W = R_WF * phiT_MB_F * V_FM
    //          = R_WF * phiT_MB_F * H_FM * vm
    //          = H_PB_W * vm
    // where H_PB_W = R_WF * phiT_MB_F * H_FM.
    // TODO(amcastro-tri): consider storing V_PB_W into the velocity
    // kinematics cache.
    SpatialVelocity<T> V_PB_W = R_WF * V_FM.Shift(p_MB_F);

    // Shift operator between the parent body P and this node's body B,
    // expressed in the world frame W.
    // TODO(amcastro-tri): consider computing p_PB_W in
    // CalcPositionKinematicsCache_BaseToTip() and saving the result in the
    // position kinematics cache.
    /* p_PB_W = R_WP * p_PB */
    Vector3<T> p_PB_W = get_X_WP(pc).rotation() * get_X_PB(pc).translation();

    // Update velocity V_WB of this body's node in the world frame.
    const SpatialVelocity<T>& V_WP = get_V_WP(*vc);

    // This is Eq. 5.6 in Jain (2010), p. 77.
    // V_WBo = V_WPBo_W + V_PBo_W (where PBo means the point of P coincident
    // with Bo).
    get_mutable_V_WB(vc) = V_WP.Shift(p_PB_W) + V_PB_W;
  }

  /// Returns the topology information for this body node.
  const BodyNodeTopology& get_topology() const { return topology_; }

 protected:
  BodyNodeTopology topology_;
  // Pointers for fast access.
  const Body<T>* body_{nullptr};
  const Mobilizer<T>* mobilizer_{nullptr};

 private:
  // Returns the index to the parent body of the body associated with this node.
  BodyIndex get_parent_body_index() const { return topology_.parent_body;}

  // =========================================================================
  // PositionKinematicsCache Accessors and Mutators.

  // Returns a mutable reference to the pose of the body B associated with this
  // node as measured and expressed in the world frame W.
  Isometry3<T>& get_mutable_X_WB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(topology_.index);
  }

  // Returns a const reference to the pose of the parent body P measured and
  // expressed in the world frame W.
  const Isometry3<T>& get_X_WP(const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(topology_.parent_body_node);
  }

  // Returns a constant reference to the across-mobilizer pose of the outboard
  // frame M as measured and expressed in the inboard frame F.
  const Isometry3<T>& get_X_FM(const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_FM(topology_.index);
  }

  // Returns a const reference to the pose of body B as measured and expressed
  // in the frame of the parent body P.
  const Isometry3<T>& get_X_PB(const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_PB(topology_.index);
  }

  // Returns a mutable reference to the pose of body B as measured and expressed
  // in the frame of the parent body P.
  Isometry3<T>& get_mutable_X_PB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.index);
  }

  // =========================================================================
  // VelocityKinematicsCache Accessors and Mutators.

  const SpatialVelocity<T>& get_V_WB(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(topology_.index);
  }

  SpatialVelocity<T>& get_mutable_V_WB(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_WB(topology_.index);
  }

  /// @returns the spatial velocity `V_WP` of the body P in the parent node as
  /// measured and expressed in the world frame.
  const SpatialVelocity<T>& get_V_WP(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(topology_.parent_body_node);
  }

  // Helper method to be called within a base-to-tip recursion that computes
  // into the PositionKinematicsCache:
  // - X_PB(qf_P, qr_B, qf_B)
  // - X_WB(q(W:P), qf_P, qr_B, qf_B)
  // where `qb_P` are the generalized coordinates associated with body P,
  // `qm_B` the generalized coordinates associated with this node's mobilizer
  // (mobilizing body B) and `qb_B` the generalized coordinates associated with
  // body B.
  // It assumes:
  // - Body B already updated the pose `X_BM(qb_B)` of the inboard mobilizer M.
  // - We are in a base-to-tip recursion and therefore `X_PF(qb_P)` and `X_WP`
  //   are were already updated.
  void CalcAcrossMobilizerBodyPoses_BaseToTip(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {
    // Body for this node.
    const Body<T>& BodyB = get_body();

    // Body for this node's parent, or the parent body P.
    const Body<T>& BodyP = get_parent_body();

    // Inboard/Outboard frames of this node's mobilizer.
    const Frame<T>& FrameF = get_mobilizer().get_inboard_frame();
    DRAKE_ASSERT(FrameF.get_body().get_index() == BodyP.get_index());
    const Frame<T>& FrameM = get_mobilizer().get_outboard_frame();
    DRAKE_ASSERT(FrameM.get_body().get_index() == BodyB.get_index());

    // Input (const):
    // - X_PF(qb_P)
    // - X_MB(qb_B)
    // - X_FM(qm_B)
    // - X_WP(q(W:B)), where q(W:B) includes all positions in the kinematics
    //                 path from body B to the world W.
    const Isometry3<T> X_MB = FrameM.CalcBodyPoseInThisFrame(context);
    const Isometry3<T>& X_FM = get_X_FM(*pc);  // mobilizer.Eval_X_FM(ctx)
    const Isometry3<T>& X_WP = get_X_WP(*pc);  // BodyP.EvalPoseInWorld(ctx)

    // Output (updating a cache entry):
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    Isometry3<T>& X_PB = get_mutable_X_PB(pc);
    Isometry3<T>& X_WB = get_mutable_X_WB(pc);  // BodyB.EvalPoseInWorld(ctx)

    // TODO(amcastro-tri): Consider logic for the common case B = M.
    // In that case X_FB = X_FM as suggested by setting X_MB = Id.
    const Isometry3<T> X_FB = X_FM * X_MB;

    // Given the pose X_FB of body frame B measured in the mobilizer inboard
    // frame F, we can ask frame F (who's parent body is P) for the pose of body
    // B measured in the frame of the parent body P.
    // In the particular case F = B, this method directly returns X_FB.
    // For flexible bodies this gives the chance to frame F to pull its pose
    // from the context.
    X_PB = FrameF.CalcOffsetPoseInBody(context, X_FB);

    X_WB = X_WP * X_PB;
  }

  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_body_node(this->get_index());
  }
};

}  // namespace multibody
}  // namespace drake
