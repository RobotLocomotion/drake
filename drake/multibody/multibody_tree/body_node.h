#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/acceleration_kinematics_cache.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"

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
/// In addition, body B could be a flexible body, in which case the pose of each
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
  /// Aborts if called on the root node corresponding to the _world_ body, for
  /// which there is no mobilizer.
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

  /// This method is used by MultibodyTree within a base-to-tip loop to compute
  /// this node's kinematics that depend on the generalized velocities.
  /// This method aborts in Debug builds when:
  /// - Called on the _root_ node.
  /// - `vc` is nullptr.
  /// @param[in] context The context with the state of the MultibodyTree model.
  /// @param[in] pc An already updated position kinematics cache in sync with
  ///               `context`.
  /// @param[out] vc A pointer to a valid, non nullptr, velocity kinematics
  ///                cache.
  /// @pre The position kinematics cache `pc` was already updated to be in sync
  /// with `context` by MultibodyTree::CalcPositionKinematicsCache().
  /// @pre CalcVelocityKinematicsCache_BaseToTip() must have already been called
  /// for the parent node (and, by recursive precondition, all predecessor nodes
  /// in the tree.)
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcVelocityKinematicsCache().
  void CalcVelocityKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      VelocityKinematicsCache<T>* vc) const {
    // This method must not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != world_index());

    DRAKE_ASSERT(vc != nullptr);

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
    // This can be split as:
    //   V_PB_W = V_PFb_W + V_FMb_W + V_MB_W                                (2)
    // where Fb and Mb are frames aligned rigidly with F and M but with their
    // origins at Bo. Assuming body P a rigid body V_PFb_W = 0 and assuming B
    // a rigid body V_MB_W = 0, but that won't be true for flexible bodies.
    // TODO(amcastro-tri): incorporate terms for flexible bodies below.
    // Therefore for rigid bodies V_PB_W = V_FMb_W, which can be computed from
    // the spatial velocity measured in frame F (as provided by mobilizer's
    // methods)
    //   V_FMb_W = R_WF * V_FMb = R_WF * V_FM.Shift(p_MoBo_F)               (3)
    // arriving to the desired result:
    //   V_PB_W = R_WF * V_FM.Shift(p_MoBo_F)                               (4)
    //
    // Computation of V_WPb:
    // This can be computed by a simple shift operation from V_WP:
    //   V_WPb = V_WP.Shift(p_PoBo_W)                                       (5)
    //
    // Note:
    // It is very common to find treatments in which the body frame B is
    // coincident with the outboard frame M, that is B ≡ M, leading to slightly
    // simpler recursive relations (for instance, see Section 3.3.2 in
    // Jain (2010).) where p_MoBo_F = 0 and thus V_PB_W = V_FM_W. Here we relax
    // this restriction in preparation of the more general case considering
    // flexible bodies.

    // Body for this node. Its body frame is also referred to as B whenever no
    // ambiguity can arise.
    const Body<T>& body_B = get_body();

    // Body for this node's parent, or the parent body P. Its body frame is
    // also referred to as P whenever no ambiguity can arise.
    const Body<T>& body_P = get_parent_body();

    // Inboard frame F of this node's mobilizer.
    const Frame<T>& frame_F = get_inboard_frame();
    DRAKE_ASSERT(frame_F.get_body().get_index() == body_P.get_index());
    // Outboard frame M of this node's mobilizer.
    const Frame<T>& frame_M = get_outboard_frame();
    DRAKE_ASSERT(frame_M.get_body().get_index() == body_B.get_index());

    // Generalized velocities local to this node's mobilizer.
    const auto& vm = this->get_mobilizer_velocities(context);

    // =========================================================================
    // Computation of V_PB_W in Eq. (1). See summary at the top of this method.

    // Operator V_FM = H_FM * vm
    SpatialVelocity<T> V_FM =
        get_mobilizer().CalcAcrossMobilizerSpatialVelocity(context, vm);

    const Isometry3<T> X_PF = frame_F.CalcPoseInBodyFrame(context);
    const Isometry3<T> X_MB = frame_M.CalcBodyPoseInThisFrame(context);

    // Pose of the parent body P in world frame W.
    // Available since we are called within a base-to-tip recursion.
    const Isometry3<T>& X_WP = get_X_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    const Matrix3<T> R_WF = X_WP.rotation() * X_PF.rotation();

    // Vector from Mo to Bo expressed in frame F as needed below:
    const Vector3<T> p_MB_F =
        /* p_MB_F = R_FM * p_MB_M */
        get_X_FM(pc).rotation() * X_MB.translation();

    // Compute V_PB_W = R_WF * V_FM.Shift(p_MoBo_F), Eq. (4).
    // Side note to developers: in operator form for rigid bodies this would be
    //   V_PB_W = R_WF * phiT_MB_F * V_FM
    //          = R_WF * phiT_MB_F * H_FM * vm
    //          = H_PB_W * vm
    // where H_PB_W = R_WF * phiT_MB_F * H_FM.
    // TODO(amcastro-tri): consider storing V_PB_W into the velocity
    // kinematics cache.
    SpatialVelocity<T> V_PB_W = R_WF * V_FM.Shift(p_MB_F);

    // =========================================================================
    // Computation of V_WPb in Eq. (1). See summary at the top of this method.

    // Shift vector between the parent body P and this node's body B,
    // expressed in the world frame W.
    // TODO(amcastro-tri): consider computing p_PB_W in
    // CalcPositionKinematicsCache_BaseToTip() and saving the result in the
    // position kinematics cache.
    /* p_PB_W = R_WP * p_PB */
    Vector3<T> p_PB_W = get_X_WP(pc).rotation() * get_X_PB(pc).translation();

    // Since we are in a base-to-tip recursion the parent body P's spatial
    // velocity is already available in the cache.
    const SpatialVelocity<T>& V_WP = get_V_WP(*vc);

    // =========================================================================
    // Update velocity V_WB of this node's body B in the world frame. Using the
    // recursive Eq. (1). See summary at the top of this method.
    get_mutable_V_WB(vc) = V_WP.Shift(p_PB_W) + V_PB_W;
  }

  void CalcAccelerationKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const VectorX<T>& mbt_vdot,
      AccelerationKinematicsCache<T>* ac) const {
    // This method must not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != world_index());

    DRAKE_ASSERT(ac != nullptr);

    // As a guideline for developers, a summary of the computations performed in
    // this method is provided:
    // Notation:
    //  - B body frame associated with this node.
    //  - P ("parent") body frame associated with this node's parent.
    //  - F mobilizer inboard frame attached to body P.
    //  - M mobilizer outboard frame attached to body B.
    // The goal is computing the spatial acceleration A_WB of body B measured in the
    // world frame W. The calculation is recursive and assumes the spatial
    // acceleration A_WP of the inboard body P is already computed.
    // The spatial velocities of P and B are related by the recursive relation
    // (computation is performed by CalcVelocityKinematicsCache_BaseToTip():
    //   V_WB = V_WPB + V_PB_W (Eq. 5.6 in Jain (2010), p. 77)              (1)
    // where V_WPB is the spatial velocity of frame B as if instantaneously
    // moving with frame P, measured and expressed in the world frame W.
    // The acceleration A_WB is defined as d_W(V_WB)/dt, where d_W()/dt denotes
    // the time derivative in the world frame. Taking d_w()/dt in Eq. (1)
    // results in a recursive relation for A_WB:
    //   A_WB = d_W(V_WPB + V_PB_W)/dt = A_WPB + d_W(V_PB_W)/dt             (2)
    // Therefore we need to develop expressions for the two terms on the right
    // hand side of Eq. (2).
    //
    // Computation of d_W(V_WPB)/dt:
    // This simply is the acceleration of frame B in the world frame W as if
    // instantaneously moving with frame P. This is nothing but the shift
    // operation of A_WP from Po to Bo and therefore:
    //   A_WPB = d_W(V_WPB)/dt = A_WP.Shift(p_PoBo_W, w_WP)                 (3)
    //
    // Computation of d_W(V_PB_W)/dt:
    // This can be computed by first shifting the time derivative to be computed
    // in the P frame as:
    //   d_W(V_PB_W)/dt = ShiftTimeDerivative(V_PB_W, A_PB_W, w_WP)         (4)
    // with V_PB_W already available in the VelocityKinematicsCache. The
    // acceleration of B in P is:
    //   A_PB = d_P(V_PB)/dt = d_F(V_FMB)/dt = A_FM.Shift(p_MB, w_FM)       (5)
    // which expressed in the world frame leads to:
    //   A_PB_W = R_WF * A_FM.Shift(p_MB_F, w_FM)                           (6)
    // where A_FM in the inboard frame F is the direct result from
    // Mobilizer::CalcAcrossMobilizerAcceleration().
    //
    // * Note:
    //     The rigid body assumption is made in Eq. (5) in two places:
    //       1. d_P()/dt = d_F()/dt since V_PF = 0.
    //       2. V_PB = V_FMB since V_PB = V_PF + V_FMB + V_MB but since P is
    //          assumed rigid V_PF = 0 and since B is assumed rigid V_MB = 0.

    // Body for this node. It's body frame is also referred to as B whenever no
    // ambiguity can arise.
    const Body<T>& BodyB = get_body();

    // Body for this node's parent, or the parent body P. It's body frame is
    // also referred to as P whenever no ambiguity can arise.
    const Body<T>& BodyP = get_parent_body();

    // Inboard frame F of this node's mobilizer.
    const Frame<T>& FrameF = get_inboard_frame();
    DRAKE_ASSERT(FrameF.get_body().get_index() == BodyP.get_index());
    // Outboard frame M of this node's mobilizer.
    const Frame<T>& FrameM = get_outboard_frame();
    DRAKE_ASSERT(FrameM.get_body().get_index() == BodyB.get_index());

    // =========================================================================
    // Computation of d_W(V_WPB)/dt in Eq. (3).

    // Shift vector between the parent body P and this node's body B,
    // expressed in the world frame W.
    // TODO(amcastro-tri): consider computing p_PB_W in
    // CalcPositionKinematicsCache_BaseToTip() and saving the result in the
    // position kinematics cache.
    /* p_PB_W = R_WP * p_PB */
    Vector3<T> p_PB_W = get_X_WP(pc).rotation() * get_X_PB(pc).translation();

    const SpatialVelocity<T>& V_WP = get_V_WP(vc);

    // Since we are in a base-to-tip recursion the parent body P's spatial
    // acceleration is already available in the cache.
    const SpatialAcceleration<T>& A_WP = get_A_WP(*ac);
    const SpatialAcceleration<T> A_WPB = A_WP.Shift(p_PB_W, V_WP.rotational());

    // =========================================================================
    // Computation of d_W(V_PB_W)/dt in Eq. (4)

    // Generalized velocities' time derivatives local to this node's mobilizer.
    const auto& vmdot = this->get_mobilizer_velocities(mbt_vdot);

    // Operator A_FM = H_FM * vmdot + Hdot_FM * vm
    SpatialAcceleration<T> A_FM =
        get_mobilizer().CalcAcrossMobilizerSpatialAcceleration(context, vmdot);

    (void) A_FM;

#if 0

    // =========================================================================
    // Computation of V_PB_W in Eq. (1). See summary at the top of this method.

    const Isometry3<T> X_PF = FrameF.CalcPoseInBodyFrame(context);
    const Isometry3<T> X_MB = FrameM.CalcBodyPoseInThisFrame(context);

    // Pose of the parent body P in world frame W.
    // Available since we are called within a base-to-tip recursion.
    const Isometry3<T>& X_WP = get_X_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    const Matrix3<T> R_WF = X_WP.rotation() * X_PF.rotation();

    // Vector from Mo to Bo expressed in frame F as needed below:
    const Vector3<T> p_MB_F =
        /* p_MB_F = R_FM * p_MB_M */
        get_X_FM(pc).rotation() * X_MB.translation();

    // Compute V_PB_W = R_WF * V_FM.Shift(p_MoBo_F), Eq. (4).
    // In operator form:
    //   V_PB_W = R_WF * phiT_MB_F * V_FM
    //          = R_WF * phiT_MB_F * H_FM * vm
    //          = H_PB_W * vm
    // where H_PB_W = R_WF * phiT_MB_F * H_FM.
    // TODO(amcastro-tri): consider storing V_PB_W into the velocity
    // kinematics cache.
    SpatialVelocity<T> V_PB_W = R_WF * V_FM.Shift(p_MB_F);
#endif

    // =========================================================================
    // Update acceleration A_WB of this node's body B in the world frame using
    // Eq. (2).
    get_mutable_A_WB(ac) = A_WPB;
  }

  /// Returns the topology information for this body node.
  const BodyNodeTopology& get_topology() const { return topology_; }

 protected:
  /// Returns the inboard frame F of this node's mobilizer.
  /// @throws std::runtime_error if called on the root node corresponding to
  /// the _world_ body.
  const Frame<T>& get_inboard_frame() const {
    return get_mobilizer().get_inboard_frame();
  }

  /// Returns the outboard frame M of this node's mobilizer.
  /// @throws std::runtime_error if called on the root node corresponding to
  /// the _world_ body.
  const Frame<T>& get_outboard_frame() const {
    return get_mobilizer().get_outboard_frame();
  }

 private:
  // Returns the index to the parent body of the body associated with this node.
  // For the root node, corresponding to the world body, this method returns an
  // invalid body index. Attempts to using invalid indexes leads to an exception
  // being thrown in Debug builds.
  BodyIndex get_parent_body_index() const { return topology_.parent_body;}

  // =========================================================================
  // Helpers to access the state.
  // Returns an Eigen expression of the vector of generalized velocities.
  Eigen::VectorBlock<const VectorX<T>> get_mobilizer_velocities(
      const MultibodyTreeContext<T>& context) const {
    return context.get_state_segment(
        topology_.mobilizer_velocities_start,
        topology_.num_mobilizer_velocities);
  }

  // Helper to get an Eigen expression of the vector of generalized velocities
  // from a vector of generalized velocities for the entire parent multibody
  // tree. Useful for the implementation of operator forms where the generalized
  // velocity (or time derivatives of the generalized velocities) is an argument
  // to the operator.
  Eigen::VectorBlock<const VectorX<T>> get_mobilizer_velocities(
      const VectorX<T>& v) const {
    return v.segment(topology_.mobilizer_velocities_start,
                     topology_.num_mobilizer_velocities);
  }

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

  // =========================================================================
  // AccelerationKinematicsCache Accessors and Mutators.

  const SpatialAcceleration<T>& get_A_WB(
      const AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(topology_.index);
  }

  SpatialAcceleration<T>& get_mutable_A_WB(
      AccelerationKinematicsCache<T>* ac) const {
    return ac->get_mutable_A_WB(topology_.index);
  }

  /// @returns the spatial velocity `V_WP` of the body P in the parent node as
  /// measured and expressed in the world frame.
  const SpatialAcceleration<T>& get_A_WP(
      const AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(topology_.parent_body_node);
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
    const Body<T>& body_B = get_body();

    // Body for this node's parent, or the parent body P.
    const Body<T>& body_P = get_parent_body();

    // Inboard/Outboard frames of this node's mobilizer.
    const Frame<T>& frame_F = get_mobilizer().get_inboard_frame();
    DRAKE_ASSERT(frame_F.get_body().get_index() == body_P.get_index());
    const Frame<T>& frame_M = get_mobilizer().get_outboard_frame();
    DRAKE_ASSERT(frame_M.get_body().get_index() == body_B.get_index());

    // Input (const):
    // - X_PF(qb_P)
    // - X_MB(qb_B)
    // - X_FM(qm_B)
    // - X_WP(q(W:B)), where q(W:B) includes all positions in the kinematics
    //                 path from body B to the world W.
    const Isometry3<T> X_MB = frame_M.CalcBodyPoseInThisFrame(context);
    const Isometry3<T>& X_FM = get_X_FM(*pc);  // mobilizer.Eval_X_FM(ctx)
    const Isometry3<T>& X_WP = get_X_WP(*pc);  // body_P.EvalPoseInWorld(ctx)

    // Output (updating a cache entry):
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    Isometry3<T>& X_PB = get_mutable_X_PB(pc);
    Isometry3<T>& X_WB = get_mutable_X_WB(pc);  // body_B.EvalPoseInWorld(ctx)

    // TODO(amcastro-tri): Consider logic for the common case B = M.
    // In that case X_FB = X_FM as suggested by setting X_MB = Id.
    const Isometry3<T> X_FB = X_FM * X_MB;

    // Given the pose X_FB of body frame B measured in the mobilizer inboard
    // frame F, we can ask frame F (who's parent body is P) for the pose of body
    // B measured in the frame of the parent body P.
    // In the particular case F = B, this method directly returns X_FB.
    // For flexible bodies this gives the chance to frame F to pull its pose
    // from the context.
    X_PB = frame_F.CalcOffsetPoseInBody(context, X_FB);

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
