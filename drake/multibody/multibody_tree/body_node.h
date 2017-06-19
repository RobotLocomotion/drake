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

namespace internal {

/// For internal use only of the MultibodyTree implementation.
/// This is a base class representing a **node** in the tree structure of a
/// MultibodyTree. %BodyNode provides implementations for convenience methods to
/// be used in MultibodyTree recursive algorithms but that however should not
/// leak into the public API for the Mobilizer class. In this regard, %BodyNode
/// provides an additional separation layer between implementation internals and
/// user facing API.
///
/// <h4>Tree Structure</h4>
///
/// As a tree data structure, a MultibodyTree can be thought of as collection of
/// %BodyNode objects where each body node has a number of %BodyNode children
/// and a unique parent %BodyNode object.
/// Each %BodyNode is associated with a given body B and an inboard mobilizer
/// that connects this body B to the rest of the tree. The unique parent body of
/// body B is denoted by P, which in turn has its own %BodyNode associated with
/// it. Associated with each %BodyNode is an inboard frame F attached on body P
/// and an outboard frame M attached to body B. The relationship between frames
/// F and M is dictated by the body B's inboard mobilizer providing the pose
/// `X_FM` as a function of the generalized coordinates associated with that
/// mobilizer.
///
/// In addition, body B could be a flexible body, in which case the pose of each
/// frame attached to B would in general be a function of a number of
/// generalized positions associated with body B. In particular, the pose
/// `X_BM` of the outboard frame M will be a function of body B's generalized
/// positions while the pose `X_PF` of the inboard frame F will be a function of
/// parent body P's generalized positions. A RigidBody has no generalized
/// positions associated with it (see RigidBody::get_num_flexible_positions()).
///
/// In summary, there will a %BodyNode for each Body in the MultibodyTree which
/// encompasses:
/// - a body B in a given MultibodyTree,
/// - the outboard frame M attached to this body B,
/// - the inboard frame F attached to the unique parent body P of body B,
/// - the mobilizer connecting the inboard frame F with the outboard frame M.
///
/// <h4>Associated State</h4>
///
/// In the same way a Mobilizer and a Body have a number of generalized
/// positions associated with them, a %BodyNode is associated with the
/// generalized positions of body B and of its inboard mobilizer.
///
/// The relationship between frames F and M is dictated by the body B's inboard
/// mobilizer providing the pose `X_FM(qm_B)` as a function of the generalized
/// coordinates `qm_B` (where `m` refers to "mobilizer" and `_B` refers to the
/// fact this is the unique inboard mobilizer of body B.)
///
/// In addition, body B could be a flexible body, case in which the pose of each
/// frame attached to B would in general be a function of the generalized
/// positions `qb_B` for body B (where `b` refers to "body" and `_B` refers to
/// body B in particular.) In particular, the pose `X_BM(qb_B)` of the outboard
/// frame M will be a function of body B's generalized positions `qb_B` while
/// the pose `X_PF(qb_P)` of the inboard frame F will be a function of parent
/// body P's generalized positions `qb_P`.
///
/// Therefore, the generalized positions associated with a given body node
/// correspond to the concatenation `qn_B = [qm_B, qb_B]`. Similarly,
/// mobilizer's generalized velocities `vm_B` and body generalized velocities
/// `vb_B` are grouped into `vn_B = [vm_B, vb_B]`. [Jain 2010] uses a similar
/// grouping of generalized coordinates when flexible bodies are considered,
/// see Chapter 13.
///
/// - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
///                algorithms. Springer Science & Business Media.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class BodyNode : public MultibodyTreeElement<BodyNode<T>, BodyNodeIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNode)

  /// A node encompasses a Body in a MultibodyTree and the inboard Mobilizer
  /// that connects this body to the rest of tree. Given a body and its inboard
  /// mobilizer in a MultibodyTree this constructor creates the corresponding
  /// %BodyNode. See this class' documentation for details on how a %BodyNode is
  /// defined.
  /// @param[in] body The body B associated with `this` node.
  /// @param[in] mobilizer The mobilizer associated with this `node`. It can
  ///                      only be a `nullptr` `body` **is** the **world** body.
  BodyNode(const Body<T>& body, const Mobilizer<T>* mobilizer) :
      body_(body), mobilizer_(mobilizer) {
    DRAKE_DEMAND(!(mobilizer == nullptr && body.get_index() != world_index()));
  }

  /// Returns a constant reference to the body B associated with this node.
  const Body<T>& get_body() const {
    return body_;
  }

  /// Returns a constant reference to the unique parent body P of the body B
  /// associated with this node. This method aborts in Debug builds if called on
  /// the root node corresponding to the _world_ body.
  const Body<T>& get_parent_body() const {
    DRAKE_ASSERT(get_parent_body_index().is_valid());
    return this->get_parent_tree().get_body(get_parent_body_index());
  }

  /// Returns a constant reference to the mobilizer associated with this node.
  /// This method aborts in Debug builds if called on the root node
  /// corresponding to the _world_ body, for which there is no mobilizer.
  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_DEMAND(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  /// This method is used by MultibodyTree within a base-to-tip loop to compute
  /// this node's kinematics that only depend on generalized positions.
  /// This method aborts in Debug builds when:
  /// - Called on the _root_ node.
  /// - `pc` is nullptr.
  /// @param[in] context The context with the state of the MultibodyTree model.
  /// @param[out] pc A pointer to a valid, non nullptr, kinematics cache.
  /// @pre CalcPositionKinematicsCache_BaseToTip() must have already been called
  /// for the parent node (and, by recursive precondition, all predecessor nodes
  /// in the tree.)
  void CalcPositionKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {
    // This method must not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != world_index());

    DRAKE_ASSERT(pc != nullptr);

    // Update mobilizer' position dependent kinematics.
    CalcAcrossMobilizerPositionKinematicsCache(context, pc);

    // This computes into the PositionKinematicsCache:
    // - X_PB(qb_P, qm_B, qb_B)
    // - X_WB(q(W:P), qb_P, qm_B, qb_B)
    // where qb_P are the generalized coordinates associated with body P, qm_B
    // the generalized coordinates associated with this node's mobilizer and
    // qb_B the generalized coordinates associated with body B. q(W:P) denotes
    // all generalized positions in the kinematics path between the world and
    // the parent body P.
    // It assumes:
    // - Body B already updated the pose `X_BM(qb_B)` of the inboard
    //   mobilizer M.
    // - We are in a base-to-tip recursion and therefore `X_PF(qb_P)` and `X_WP`
    //   have already been updated.
    CalcAcrossMobilizerBodyPoses_BaseToTip(context, pc);

    // TODO(amcastro-tri):
    // Update Body specific kinematics. These include:
    // - p_PB_W: vector from P to B to perform shift operations.
    // - com_W: center of mass.
    // - M_Bo_W: Spatial inertia.

    // TODO(amcastro-tri):
    // With H_FM(qm) already in the cache (computed by
    // Mobilizer::UpdatePositionKinematicsCache()) update the cache
    // entries for H_PB_W, the Jacobian for the SpatialVelocity jump between
    // body B and its parent body P expressed in the world frame W.
  }

  /// Returns the topology information for this body node.
  const BodyNodeTopology& get_topology() const { return topology_; }

 private:
  // Returns the index to the parent body of the body associated with this node.
  // For the root node, corresponding to the world body, this method returns an
  // invalid body index. Attempts to using invalid indexes leads to an exception
  // being thrown in Debug builds.
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

  // Returns a mutable reference to the across-mobilizer pose of the outboard
  // frame M as measured and expressed in the inboard frame F.
  Isometry3<T>& get_mutable_X_FM(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_FM(topology_.index);
  }

  // Returns a mutable reference to the pose of body B as measured and expressed
  // in the frame of the parent body P.
  Isometry3<T>& get_mutable_X_PB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.index);
  }

  // Helper method to be called within a base-to-tip recursion that computes
  // into the PositionKinematicsCache:
  // - X_PB(qb_P, qm_B, qb_B)
  // - X_WB(q(W:P), qb_P, qm_B, qb_B)
  // where qb_P are the generalized coordinates associated with body P, qm_B
  // the generalized coordinates associated with this node's mobilizer, and
  // qb_B the generalized coordinates associated with body B. q(W:P) denotes
  // all generalized positions in the kinematics path between the world and
  // the parent body P.
  // It assumes:
  // - Body B already updated the pose `X_BM(qb_B)` of the inboard
  //   mobilizer M.
  // - We are in a base-to-tip recursion and therefore `X_PF(qb_P)` and `X_WP`
  //   have already been updated.
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

  // Computes position dependent kinematics associated with `this` mobilizer
  // which includes:
  // - X_FM(q): The pose of the outboard frame M as measured and expressed in
  //            the inboard frame F.
  // - H_FM(q): the Jacobian matrix describing the relationship between
  //            generalized velocities v and the spatial velocity `V_FM` by
  //            `V_FM(q, v) = H_FM(q) * v`.
  // - Hdot_FM(q): The time derivative of the Jacobian matrix which allows
  //               computing the spatial acceleration between the F and M
  //               frames as:
  //               `A_FM(q, v, v̇) = H_FM(q) * v̇ + Hdot_FM(q) * v`
  // - N(q): The kinematic coupling matrix describing the relationship between
  //         the rate of change of generalized coordinates and the generalized
  //         velocities by `q̇ = N(q) * v`.
  //
  // This method is used by MultibodyTree to update the position kinematics
  // quantities associated with `this` mobilizer. MultibodyTree will always
  // provide a valid PositionKinematicsCache pointer, otherwise this method
  // aborts in Debug builds.
  void CalcAcrossMobilizerPositionKinematicsCache(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {
    DRAKE_ASSERT(pc != nullptr);
    Isometry3<T>& X_FM = get_mutable_X_FM(pc);
    X_FM = get_mobilizer().CalcAcrossMobilizerTransform(context);
  }

  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_body_node(this->get_index());
  }

  BodyNodeTopology topology_;
  // Pointers for fast access.
  const Body<T>& body_;
  const Mobilizer<T>* mobilizer_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
