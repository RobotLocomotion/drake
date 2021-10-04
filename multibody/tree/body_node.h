#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/articulated_body_force_cache.h"
#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"

namespace drake {
namespace multibody {

namespace internal {

// For internal use only of the MultibodyTree implementation.
// This is a base class representing a **node** in the tree structure of a
// MultibodyTree. %BodyNode provides implementations for convenience methods to
// be used in MultibodyTree recursive algorithms but that however should not
// leak into the public API for the Mobilizer class. In this regard, %BodyNode
// provides an additional separation layer between implementation internals and
// user facing API.
//
// <h4>Tree Structure</h4>
//
// As a tree data structure, a MultibodyTree can be thought of as collection of
// %BodyNode objects where each body node has a number of %BodyNode children
// and a unique parent %BodyNode object.
// Each %BodyNode is associated with a given body B and an inboard mobilizer
// that connects this body B to the rest of the tree. The unique parent body of
// body B is denoted by P, which in turn has its own %BodyNode associated with
// it. Associated with each %BodyNode is an inboard frame F attached on body P
// and an outboard frame M attached to body B. The relationship between frames
// F and M is dictated by the body B's inboard mobilizer providing the pose
// `X_FM` as a function of the generalized coordinates associated with that
// mobilizer.
//
// In addition, body B could be a flexible body, in which case the pose of each
// frame attached to B would in general be a function of a number of
// generalized positions associated with body B. In particular, the pose
// `X_BM` of the outboard frame M will be a function of body B's generalized
// positions while the pose `X_PF` of the inboard frame F will be a function of
// parent body P's generalized positions. A RigidBody has no generalized
// positions associated with it (see RigidBody::get_num_flexible_positions()).
//
// In summary, there will a %BodyNode for each Body in the MultibodyTree which
// encompasses:
//
// - a body B in a given MultibodyTree,
// - the outboard frame M attached to this body B,
// - the inboard frame F attached to the unique parent body P of body B,
// - the mobilizer connecting the inboard frame F with the outboard frame M.
//
// <h4>Associated State</h4>
//
// In the same way a Mobilizer and a Body have a number of generalized
// positions associated with them, a %BodyNode is associated with the
// generalized positions of body B and of its inboard mobilizer.
//
// The relationship between frames F and M is dictated by the body B's inboard
// mobilizer providing the pose `X_FM(qm_B)` as a function of the generalized
// coordinates `qm_B` (where `m` refers to "mobilizer" and `_B` refers to the
// fact this is the unique inboard mobilizer of body B.)
//
// In addition, body B could be a flexible body, in which case the pose of each
// frame attached to B would in general be a function of the generalized
// positions `qb_B` for body B (where `b` refers to "body" and `_B` refers to
// body B in particular.) In particular, the pose `X_BM(qb_B)` of the outboard
// frame M will be a function of body B's generalized positions `qb_B` while
// the pose `X_PF(qb_P)` of the inboard frame F will be a function of parent
// body P's generalized positions `qb_P`.
//
// Therefore, the generalized positions associated with a given body node
// correspond to the concatenation `qn_B = [qm_B, qb_B]`. Similarly,
// mobilizer's generalized velocities `vm_B` and body generalized velocities
// `vb_B` are grouped into `vn_B = [vm_B, vb_B]`. [Jain 2010] uses a similar
// grouping of generalized coordinates when flexible bodies are considered,
// see Chapter 13.
//
// - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
//                algorithms. Springer Science & Business Media.
//
// @tparam_default_scalar
template <typename T>
class BodyNode : public MultibodyElement<BodyNode, T, BodyNodeIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNode)

  // A node encompasses a Body in a MultibodyTree and the inboard Mobilizer
  // that connects this body to the rest of tree. Given a body and its inboard
  // mobilizer in a MultibodyTree this constructor creates the corresponding
  // %BodyNode. See this class' documentation for details on how a %BodyNode is
  // defined.
  // @param[in] parent_node
  //   A const pointer to the parent BodyNode object in the tree structure of
  //   the owning MultibodyTree. It can be a `nullptr` only when `body` **is**
  //   the **world** body, otherwise this constructor will abort.
  // @param[in] body
  //   The body B associated with `this` node.
  // @param[in] mobilizer
  //   The mobilizer associated with this `node`. It can be a `nullptr` only
  //   when `body` **is** the **world** body, otherwise this method will abort.
  //
  // @note %BodyNode keeps a reference to the parent body, body and mobilizer
  // for this node, which must outlive `this` BodyNode.
  BodyNode(const BodyNode<T>* parent_node,
           const Body<T>* body, const Mobilizer<T>* mobilizer)
      : MultibodyElement<BodyNode, T, BodyNodeIndex>(
            body->model_instance()),
        parent_node_(parent_node),
        body_(body),
        mobilizer_(mobilizer) {
    DRAKE_DEMAND(!(parent_node == nullptr &&
        body->index() != world_index()));
    DRAKE_DEMAND(!(mobilizer == nullptr && body->index() != world_index()));
  }

  // Method to update the list of child body nodes maintained by this node,
  // outboard to this node. Recall a %BodyNode is a tree node within the tree
  // structure of MultibodyTree. Therefore each %BodyNode has a unique parent
  // %BodyNode, supplied at construction, and a set of child nodes, specified
  // via calls to this method.
  // Used by MultibodyTree at creation of a BodyNode during the
  // MultibodyTree::Finalize() method call.
  void add_child_node(const BodyNode<T>* child) {
    children_.push_back(child);
  }

  // Returns a constant reference to the body B associated with this node.
  const Body<T>& body() const {
    DRAKE_ASSERT(body_ != nullptr);
    return *body_;
  }

  // Returns a constant reference to the unique parent body P of the body B
  // associated with this node. This method aborts in Debug builds if called on
  // the root node corresponding to the _world_ body.
  const Body<T>& parent_body() const {
    DRAKE_ASSERT(get_parent_body_index().is_valid());
    return this->get_parent_tree().get_body(get_parent_body_index());
  }

  // Returns a const pointer to the parent (inboard) body node or nullptr if
  // `this` is the world node, which has no inboard parent node.
  const BodyNode<T>* parent_body_node() const {
    return parent_node_;
  }

  // Returns a constant reference to the mobilizer associated with this node.
  // Aborts if called on the root node corresponding to the _world_ body, for
  // which there is no mobilizer.
  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_DEMAND(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  // @name Methods to retrieve BodyNode sizes
  //@{

  // Returns the number of generalized positions for the Mobilizer in `this`
  // node.
  int get_num_mobilizer_positions() const {
    return topology_.num_mobilizer_positions;
  }

  // Returns the number of generalized velocities for the Mobilizer in `this`
  // node.
  int get_num_mobilizer_velocities() const {
    return topology_.num_mobilizer_velocities;
  }

  // Returns the index to the first generalized velocity for this node
  // within the vector v of generalized velocities for the full multibody
  // system.
  int velocity_start() const {
    return topology_.mobilizer_velocities_start_in_v;
  }

  //@}


  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // this node's kinematics that only depend on generalized positions.
  // This method aborts in Debug builds when:
  //
  // - Called on the _root_ node.
  // - `pc` is nullptr.
  //
  // @param[in] context The context with the state of the MultibodyTree model.
  // @param[out] pc A pointer to a valid, non nullptr, kinematics cache.
  // @pre CalcPositionKinematicsCache_BaseToTip() must have already been called
  // for the parent node (and, by recursive precondition, all predecessor nodes
  // in the tree.)
  void CalcPositionKinematicsCache_BaseToTip(
      const systems::Context<T>& context,
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
    // entries for H_PB_W, the hinge matrix for the SpatialVelocity jump between
    // body B and its parent body P expressed in the world frame W.
  }

  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // this node's kinematics that depend on the generalized velocities.
  // This method aborts in Debug builds when:
  // - Called on the _root_ node.
  // - `vc` is nullptr.
  // @param[in] context
  //   The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[in] H_PB_W
  //   The `6 x nm` hinge matrix that relates `V_PB_W` (body B's spatial
  //   velocity in its parent body P, expressed in world W) to this node's `nm`
  //   generalized velocities (or mobilities) `v_B` as `V_PB_W = H_PB_W * v_B`.
  // @param[out] vc
  //   A pointer to a valid, non nullptr, velocity kinematics cache.
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcPositionKinematicsCache().
  // @pre CalcVelocityKinematicsCache_BaseToTip() must have already been called
  // for the parent node (and, by recursive precondition, all predecessor nodes
  // in the tree.)
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcVelocityKinematicsCache().
  void CalcVelocityKinematicsCache_BaseToTip(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      VelocityKinematicsCache<T>* vc) const {
    // This method must not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != world_index());

    DRAKE_ASSERT(vc != nullptr);
    DRAKE_DEMAND(H_PB_W.rows() == 6);
    DRAKE_DEMAND(H_PB_W.cols() == get_num_mobilizer_velocities());

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
    // Jain (2010).) where p_MoBo_F = 0 and thus V_PB_W = V_FM_W. Here we relax
    // this restriction in preparation of the more general case considering
    // flexible bodies.

    // Generalized velocities local to this node's mobilizer.
    const auto& vm = this->get_mobilizer_velocities(context);

    // =========================================================================
    // Computation of V_PB_W in Eq. (1). See summary at the top of this method.

    // Update V_FM using the operator V_FM = H_FM * vm:
    SpatialVelocity<T>& V_FM = get_mutable_V_FM(vc);
    V_FM = get_mobilizer().CalcAcrossMobilizerSpatialVelocity(context, vm);

    // Compute V_PB_W = R_WF * V_FM.Shift(p_MoBo_F), Eq. (4).
    // Side note to developers: in operator form for rigid bodies this would be
    //   V_PB_W = R_WF * phiT_MB_F * V_FM
    //          = R_WF * phiT_MB_F * H_FM * vm
    //          = H_PB_W * vm
    // where H_PB_W = R_WF * phiT_MB_F * H_FM.
    SpatialVelocity<T>& V_PB_W = get_mutable_V_PB_W(vc);
    V_PB_W.get_coeffs() = H_PB_W * vm;

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

  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // this node's kinematics that depend on the generalized accelerations, i.e.
  // the generalized velocities' time derivatives.
  // This method aborts in Debug builds when:
  // - Called on the _root_ node.
  // - `ac` is nullptr.
  // @param[in] context The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[in] vc
  //   An already updated velocity kinematics cache in sync with `context`.
  //   If vc is nullptr, velocities are assumed to be zero and velocity
  //   dependent terms are not computed.
  // @param[in] mbt_vdot
  //   The entire vector of generalized accelerations for the full
  //   MultibodyTree model. It must have a size equal to the number of
  //   generalized velocities in the model. This method assumes the caller,
  //   MultibodyTree<T>::CalcAccelerationKinematicsCache(), provides a vector
  //   of the right size.
  // @param[in,out] A_WB_array_ptr
  //   A pointer to a valid, non nullptr, vector of spatial accelerations
  //   containing the spatial acceleration `A_WB` for each body. On input, it
  //   must contain already pre-computed spatial accelerations for the inboard
  //   bodies to this node's body B, see precondition below.  It must be of
  //   size equal to the number of bodies in the MultibodyTree and ordered by
  //   BodyNodeIndex. The calling MultibodyTree method must guarantee these
  //   conditions are satisfied. This method will abort if the pointer is
  //   null. There is no mechanism to assert that `A_WB_array_ptr` is ordered
  //   by BodyNodeIndex and the correctness of MultibodyTree methods, properly
  //   unit tested, should guarantee this condition.
  //
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcPositionKinematicsCache().
  // @pre The velocity kinematics cache `vc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcVelocityKinematicsCache().
  // @pre CalcAccelerationKinematicsCache_BaseToTip() must have already been
  // called for the parent node (and, by recursive precondition, all
  // predecessor nodes in the tree). Therefore, on input, the argument array
  // `A_WB_array_ptr` must contain already pre-computed spatial accelerations
  // for the inboard bodies to this node's body B.
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcAccelerationKinematicsCache().
  void CalcSpatialAcceleration_BaseToTip(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>* vc,
      const VectorX<T>& mbt_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array_ptr) const {
    // This method must not be called for the "world" body node.
    DRAKE_DEMAND(topology_.body != world_index());
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

    // Body for this node. Its body frame is also referred to as B whenever no
    // ambiguity can arise.
    const Body<T>& body_B = body();

    // Body for this node's parent, or the parent body P. Its body frame is
    // also referred to as P whenever no ambiguity can arise.
    const Body<T>& body_P = parent_body();

    // Inboard frame F of this node's mobilizer.
    const Frame<T>& frame_F = inboard_frame();
    DRAKE_ASSERT(frame_F.body().index() == body_P.index());
    // Outboard frame M of this node's mobilizer.
    const Frame<T>& frame_M = outboard_frame();
    DRAKE_ASSERT(frame_M.body().index() == body_B.index());

    // =========================================================================
    // Computation of A_PB = DtP(V_PB), Eq. (4).

    // TODO(amcastro-tri): consider caching these. Especially true if bodies are
    // flexible. Also used in velocity kinematics.
    const math::RotationMatrix<T> R_PF =
        frame_F.CalcRotationMatrixInBodyFrame(context);
    const math::RigidTransform<T> X_MB =
        frame_M.CalcPoseInBodyFrame(context).inverse();

    // Form the rotation matrix relating the world frame W and parent body P.
    // Available since we are called within a base-to-tip recursion.
    const math::RotationMatrix<T>& R_WP = get_R_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    // TODO(amcastro-tri): consider caching X_WF since it is also used to
    // compute H_PB_W.
    const math::RotationMatrix<T> R_WF = R_WP * R_PF;

    // Vector from Mo to Bo expressed in frame F as needed below:
    // TODO(amcastro-tri): consider caching this since it is also used to
    // compute H_PB_W.
    const math::RotationMatrix<T>& R_FM = get_X_FM(pc).rotation();
    const Vector3<T>& p_MB_M = X_MB.translation();
    const Vector3<T> p_MB_F = R_FM * p_MB_M;

    // Generalized velocities' time derivatives local to this node's mobilizer.
    const auto& vmdot = this->get_mobilizer_velocities(mbt_vdot);

    // Operator A_FM = H_FM * vmdot + Hdot_FM * vm
    SpatialAcceleration<T> A_FM =
        get_mobilizer().CalcAcrossMobilizerSpatialAcceleration(context, vmdot);

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
      const SpatialAcceleration<T> A_PB_W =
          R_WF * A_FM.Shift(p_MB_F);  // Eq. (4), with w_FM = 0.
      // Velocities are zero. No need to compute terms that become zero.
      get_mutable_A_WB_from_array(&A_WB_array) = A_WP.Shift(p_PB_W) + A_PB_W;
    }
  }

  // Computes the generalized forces `tau` for a single BodyNode.
  // This method is used by MultibodyTree within a tip-to-base loop to compute
  // the vector of generalized forces `tau` that would correspond with a known
  // set of spatial accelerations `A_WB` for each body in the MultibodyTree.
  //
  // This method aborts in Debug builds when `F_BMo_W_array` is nullptr.
  //
  // @param[in] context The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[in] M_B_W_cache
  //   An already updated cache storing the spatial inertia M_Bo_W(q) for each
  //   body in the model, in sync with `context`.
  // @param[in] Fb_Bo_W_cache
  //   An already updated cache storing the bias term Fb_Bo_W(q, v) for each
  //   body in the model, in sync with `context`.
  //   If Fb_Bo_W_cache is nullptr, velocities are assumed to be zero (thus
  //   Fb_Bo_W is zero) and velocity dependent terms are not computed.
  // @param[in] A_WB_array
  //   A vector of known spatial accelerations containing the spatial
  //   acceleration `A_WB` for each body in the MultibodyTree model. It must be
  //   of size equal to the number of bodies in the MultibodyTree and ordered
  //   by BodyNodeIndex. The calling MultibodyTree method must guarantee these
  //   conditions are satisfied.
  // @param[in] Fapplied_Bo_W
  //   Externally applied spatial force on this node's body B at the body's
  //   frame origin `Bo`, expressed in the world frame.
  //   `Fapplied_Bo_W` **must** not be an entry into `F_BMo_W_array_ptr`, which
  //   would result in undefined results.
  // @param[in] tau_applied
  //   Externally applied generalized force at this node's mobilizer. It can
  //   have zero size, implying no generalized forces are applied. Otherwise it
  //   must have a size equal to the number of generalized velocities for this
  //   node's mobilizer, see get_num_mobilizer_velocities().
  //   `tau_applied` **must** not be an entry into `tau_array`, which would
  //   result in undefined results.
  // @param[out] F_BMo_W_array_ptr
  //   A pointer to a valid, non nullptr, vector of spatial forces containing,
  //   for each body B, the spatial force `F_BMo_W` corresponding to its
  //   inboard mobilizer reaction forces on body B applied at the origin `Mo`
  //   of the inboard mobilizer, expressed in the world frame W.  It must be of
  //   size equal to the number of bodies in the MultibodyTree and ordered by
  //   BodyNodeIndex. The calling MultibodyTree method must guarantee these
  //   conditions are satisfied. This method will abort if the pointer is null.
  //   To access a mobilizer's reaction force on a given body B, access this
  //   array with the index returned by Body::node_index().
  // @param[out] tau_array
  //   A non-null pointer to the output vector of generalized forces that would
  //   result in body B having spatial acceleration `A_WB`. This method will
  //   abort if the pointer is null. The calling MultibodyTree method must
  //   guarantee the size of the array is the number of generalized velocities
  //   in the model.
  //
  // @note There is no mechanism to assert that either `A_WB_array` nor
  //   `F_BMo_W_array_ptr` are ordered by BodyNodeIndex and the correctness of
  //   MultibodyTree methods, properly unit tested, should guarantee this
  //   condition.
  //
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcPositionKinematicsCache().
  // @pre The velocity kinematics cache `vc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcVelocityKinematicsCache().
  // @pre CalcInverseDynamics_TipToBase() must have already been
  // called for all the child nodes of `this` node (and, by recursive
  // precondition, all successor nodes in the tree.)
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcInverseDynamics().
  void CalcInverseDynamics_TipToBase(
      const systems::Context<T>& context,
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
    DRAKE_DEMAND(
        tau_applied.size() == get_num_mobilizer_velocities() ||
        tau_applied.size() == 0);
    DRAKE_DEMAND(tau_array != nullptr);
    DRAKE_DEMAND(tau_array->size() ==
        this->get_parent_tree().num_velocities());

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
    const Body<T>& body_B = body();

    // Input spatial acceleration for this node's body B.
    const SpatialAcceleration<T>& A_WB = get_A_WB_from_array(A_WB_array);

    // Total spatial force on body B producing acceleration A_WB.
    SpatialForce<T> Ftot_BBo_W;
    CalcBodySpatialForceGivenItsSpatialAcceleration(M_B_W_cache, Fb_Bo_W_cache,
                                                    A_WB, &Ftot_BBo_W);

    // Compute shift vector from Bo to Mo expressed in the world frame W.
    const Frame<T>& frame_M = outboard_frame();
    DRAKE_DEMAND(frame_M.body().index() == body_B.index());
    const math::RigidTransform<T> X_BM = frame_M.CalcPoseInBodyFrame(context);
    const Vector3<T>& p_BoMo_B = X_BM.translation();
    const math::RigidTransform<T>& X_WB = get_X_WB(pc);
    const math::RotationMatrix<T>& R_WB = X_WB.rotation();
    const Vector3<T> p_BoMo_W = R_WB * p_BoMo_B;

    // Output spatial force that would need to be exerted by this node's
    // mobilizer in order to attain the prescribed acceleration A_WB.
    SpatialForce<T>& F_BMo_W = F_BMo_W_array[this->index()];

    // Ensure this method was not called with an Fapplied_Bo_W being an entry
    // into F_BMo_W_array, otherwise we would be overwriting Fapplied_Bo_W.
    DRAKE_DEMAND(&F_BMo_W != &Fapplied_Bo_W);

    // Shift spatial force on B to Mo.
    F_BMo_W = Ftot_BBo_W.Shift(p_BoMo_W);
    for (const BodyNode<T>* child_node : children_) {
      BodyNodeIndex child_node_index = child_node->index();

      // Shift vector from Bo to Co, expressed in the world frame W.
      const Vector3<T>& p_BoCo_W = child_node->get_p_PoBo_W(pc);

      // p_CoMc_W:
      const Frame<T>& frame_Mc = child_node->outboard_frame();
      const math::RotationMatrix<T>& R_WC = child_node->get_X_WB(pc).rotation();
      const math::RigidTransform<T> X_CMc =
          frame_Mc.CalcPoseInBodyFrame(context);
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
    const math::RotationMatrix<T> R_PF =
        inboard_frame().CalcRotationMatrixInBodyFrame(context);
    const math::RotationMatrix<T>& R_WP = get_R_WP(pc);
    // TODO(amcastro-tri): consider caching R_WF since also used in position and
    // velocity kinematics.
    const math::RotationMatrix<T> R_WF = R_WP * R_PF;
    const SpatialForce<T> F_BMo_F = R_WF.inverse() * F_BMo_W;

    // Generalized velocities and forces use the same indexing.
    auto tau = get_mutable_generalized_forces_from_array(tau_array);

    // Demand that tau_applied is not an entry of tau. It would otherwise get
    // overwritten.
    DRAKE_DEMAND(tau.data() != tau_applied.data());

    // The generalized forces on the mobilizer correspond to the active
    // components of the spatial force performing work. Therefore we need to
    // project F_BMo along the directions of motion.
    // Project as: tau = H_FMᵀ(q) * F_BMo_F, Eq. (4).
    get_mobilizer().ProjectSpatialForce(context, F_BMo_F, tau);

    // Include the contribution of applied generalized forces.
    if (tau_applied.size() != 0) tau -= tau_applied;
  }

  // Returns the topology information for this body node.
  const BodyNodeTopology& get_topology() const { return topology_; }

  // Calculates the hinge matrix H_PB_W.
  // @param[in] context
  //   The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[out] H_PB_W
  //   The `6 x nm` hinge matrix that relates `V_PB_W` (body B's spatial
  //   velocity in its parent body P, expressed in world W) to this node's `nm`
  //   generalized velocities (or mobilities) `v_B` as `V_PB_W = H_PB_W * v_B`.
  // @note `H_PB_W` is only a function of the model's generalized positions q.
  //
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcPositionKinematicsCache().
  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      EigenPtr<MatrixX<T>> H_PB_W) const {
    // Checks on the input arguments.
    DRAKE_DEMAND(topology_.body != world_index());
    DRAKE_DEMAND(H_PB_W != nullptr);
    DRAKE_DEMAND(H_PB_W->rows() == 6);
    DRAKE_DEMAND(H_PB_W->cols() == get_num_mobilizer_velocities());

    // Inboard frame F of this node's mobilizer.
    const Frame<T>& frame_F = inboard_frame();
    // Outboard frame M of this node's mobilizer.
    const Frame<T>& frame_M = outboard_frame();

    const math::RotationMatrix<T> R_PF =
        frame_F.CalcRotationMatrixInBodyFrame(context);
    const math::RigidTransform<T> X_MB =
        frame_M.CalcPoseInBodyFrame(context).inverse();

    // Form the rotation matrix relating the world frame W and parent body P.
    const math::RotationMatrix<T>& R_WP = get_R_WP(pc);

    // Orientation (rotation) of frame F with respect to the world frame W.
    const math::RotationMatrix<T> R_WF = R_WP * R_PF;

    // Vector from Mo to Bo expressed in frame F as needed below:
    const math::RotationMatrix<T>& R_FM = get_X_FM(pc).rotation();
    const Vector3<T>& p_MB_M = X_MB.translation();
    const Vector3<T> p_MB_F = R_FM * p_MB_M;

    // Compute the imob-th column in J_PB_W:
    VectorUpTo6<T> v = VectorUpTo6<T>::Zero(get_num_mobilizer_velocities());
    // We compute H_FM(q) one column at a time by calling the multiplication by
    // H_FM operation on a vector of generalized velocities which is zero except
    // for its imob-th component, which is one.
    for (int imob = 0; imob < get_num_mobilizer_velocities(); ++imob) {
      v(imob) = 1.0;
      // Compute the imob-th column of H_FM:
      const SpatialVelocity<T> Himob_FM =
          get_mobilizer().CalcAcrossMobilizerSpatialVelocity(context, v);
      v(imob) = 0.0;
      // V_PB_W = V_PFb_W + V_FMb_W + V_MB_W = V_FMb_W =
      //         = R_WF * V_FM.Shift(p_MoBo_F)
      H_PB_W->col(imob) = (R_WF * Himob_FM.Shift(p_MB_F)).get_coeffs();
    }
  }

  // Helper method to retrieve a Jacobian matrix with respect to generalized
  // velocities v for `this` node from an array storing the columns of a set of
  // Jacobian matrices for each node.  This method is used by MultibodyTree
  // implementations to retrieve per-node Jacobian matrices from a
  // `std::vector` that would usually live in the cache.
  // @param[in] H_array
  //   This array stores a Jacobian matrix `H` for each node in the tree. Each
  //   matrix has size `6 x nm` with `nm` the number of mobilities of the node.
  //   `H_array` stores the columns of these matrices and therefore it consists
  //   of a `std::vector` of vectors in ℝ⁶ with as many entries as the number
  //   of generalized velocities v in the model.
  //   `H_array` must be of size MultibodyTree::num_velocities().
  // @retval H
  //   An Eigen::Map to a matrix of size `6 x nm` corresponding to the Jacobian
  //   matrix for this node.
  Eigen::Map<const MatrixUpTo6<T>> GetJacobianFromArray(
      const std::vector<Vector6<T>>& H_array) const {
    DRAKE_DEMAND(static_cast<int>(H_array.size()) ==
        this->get_parent_tree().num_velocities());
    const int start_index_in_v = get_topology().mobilizer_velocities_start_in_v;
    const int num_velocities = get_topology().num_mobilizer_velocities;
    DRAKE_DEMAND(num_velocities == 0 ||
                 start_index_in_v < this->get_parent_tree().num_velocities());
    // The first column of this node's hinge matrix H_PB_W:
    const T* H_col0 =
        num_velocities == 0 ? nullptr : H_array[start_index_in_v].data();
    // Create an Eigen map to the full H_PB_W for this node:
    return Eigen::Map<const MatrixUpTo6<T>>(H_col0, 6, num_velocities);
  }

  // Mutable version of GetJacobianFromArray().
  Eigen::Map<MatrixUpTo6<T>> GetMutableJacobianFromArray(
      std::vector<Vector6<T>>* H_array) const {
    DRAKE_DEMAND(static_cast<int>(H_array->size()) ==
                 this->get_parent_tree().num_velocities());
    const int start_index_in_v = get_topology().mobilizer_velocities_start_in_v;
    const int num_velocities = get_topology().num_mobilizer_velocities;
    DRAKE_DEMAND(num_velocities == 0 ||
                 start_index_in_v < this->get_parent_tree().num_velocities());
    // The first column of this node's hinge matrix H_PB_W:
    T* H_col0 =
        num_velocities == 0 ? nullptr : (*H_array)[start_index_in_v].data();
    // Create an Eigen map to the full H_PB_W for this node:
    return Eigen::Map<MatrixUpTo6<T>>(H_col0, 6, num_velocities);
  }

  // This method is used by MultibodyTree within a tip-to-base loop to compute
  // this node's articulated body inertia quantities that depend only on the
  // generalized positions.
  //
  // @param[in] context
  //   The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[in] H_PB_W
  //   The `6 x nm` hinge matrix that relates `V_PB_W` (body B's spatial
  //   velocity in its parent body P, expressed in world W) to this node's `nm`
  //   generalized velocities (or mobilities) `v_B` as `V_PB_W = H_PB_W * v_B`.
  // @param[in] reflected_inertia
  //   Vector of scalar reflected inertia values for each degree of freedon.
  //   Used if this body node is the outboard body of a single-dof mobilizer.
  // @param[out] abic
  //   A pointer to a valid, non nullptr, articulated body cache.
  //
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with `context` by MultibodyTree::CalcPositionKinematicsCache().
  // @pre CalcArticulatedBodyInertiaCache_TipToBase() must have already been
  // called for all the child nodes of `this` node (and, by recursive
  // precondition, all successor nodes in the tree.)
  //
  // @throws std::exception when called on the _root_ node or `abic` is
  // nullptr.
  // TODO(amcastro-tri): Consider specialized BodyNodeImpl implementations that
  // exploit the sparsity pattern of H_PB_W even at compile time. Most common
  // cases are:
  // - Revolute: [x y z 0 0 0]
  // - Prismatic: [0 0 0 x y z]
  // - Ball: 3x3 blocks of zeroes.
  void CalcArticulatedBodyInertiaCache_TipToBase(
      const systems::Context<T>&,
      const PositionKinematicsCache<T>& pc,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      const SpatialInertia<T>& M_B_W,
      const VectorX<T>& reflected_inertia,
      ArticulatedBodyInertiaCache<T>* abic) const {
    DRAKE_THROW_UNLESS(topology_.body != world_index());
    DRAKE_THROW_UNLESS(abic != nullptr);
    DRAKE_THROW_UNLESS(reflected_inertia.size() ==
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
    for (const BodyNode<T>* child : children_) {
      // Shift vector p_CoBo_W.
      const Vector3<T>& p_BoCo_W = child->get_p_PoBo_W(pc);
      const Vector3<T> p_CoBo_W = -p_BoCo_W;

      // Pull Pplus_BC_W from cache (which is Pplus_PB_W for child).
      const ArticulatedBodyInertia<T>& Pplus_BC_W
          = child->get_Pplus_PB_W(*abic);

      // Shift Pplus_BC_W to Pplus_BCb_W.
      // This is known to be one of the most expensive operations of ABA and
      // must not be overlooked. Refer to #12435 for details.
      const ArticulatedBodyInertia<T> Pplus_BCb_W = Pplus_BC_W.Shift(p_CoBo_W);

      // Add Pplus_BCb_W contribution to articulated body inertia.
      P_B_W += Pplus_BCb_W;
    }

    // Get the number of mobilizer velocities (number of columns of H_PB_W).
    const int nv = get_num_mobilizer_velocities();

    ArticulatedBodyInertia<T>& Pplus_PB_W = get_mutable_Pplus_PB_W(abic);
    Pplus_PB_W = P_B_W;

    // We now proceed to compute Pplus_PB_W using Eq. (7):
    //   Pplus_PB_W = P_B_W - g_PB_W * U_B_W
    // For weld joints, with nv = 0, terms involving the hinge matrix H_PB_W
    // go away and therefore Pplus_PB_W = P_B_W. We check this below.
    if (nv != 0) {
      // Compute common term U_B_W.
      const MatrixUpTo6<T> U_B_W = H_PB_W.transpose() * P_B_W;

      // Compute the articulated body hinge inertia, D_B, using (5).
      MatrixUpTo6<T> D_B(nv, nv);
      D_B.template triangularView<Eigen::Lower>() = U_B_W * H_PB_W;

      // Add the effect of reflected inertia.
      // See JointActuator::reflected_inertia().
      // Reminder: reflected_inertia is implemented only for revolute or
      // prismatic joints (i.e., single degree-of-freedom joints, hence nv = 1).
      if (nv == 1) {
        D_B(0, 0) += reflected_inertia(this->velocity_start());
      }

      // Compute the LDLT factorization of D_B as ldlt_D_B.
      // TODO(bobbyluig): Test performance against inverse().
      math::LinearSolver<Eigen::LDLT, MatrixUpTo6<T>>& ldlt_D_B =
          get_mutable_ldlt_D_B(abic);
      ldlt_D_B = math::LinearSolver<Eigen::LDLT, MatrixUpTo6<T>>(
          MatrixUpTo6<T>(D_B.template selfadjointView<Eigen::Lower>()));

      // Ensure that D_B is not singular.
      // Singularity means that a non-physical hinge mapping matrix was used or
      // that this articulated body inertia has some non-physical quantities
      // (such as zero moment of inertia along an axis which the hinge mapping
      // matrix permits motion).
      if (ldlt_D_B.eigen_linear_solver().info() != Eigen::Success) {
        std::stringstream message;
        message << "Encountered singular articulated body hinge inertia "
                << "for body node index " << topology_.index << ". "
                << "Please ensure that this body has non-zero inertia "
                << "along all axes of motion.";
        throw std::runtime_error(message.str());
      }

      // Compute the Kalman gain, g_PB_W, using (6).
      Matrix6xUpTo6<T>& g_PB_W = get_mutable_g_PB_W(abic);
      g_PB_W = ldlt_D_B.Solve(U_B_W).transpose();

      // Project P_B_W using (7) to obtain Pplus_PB_W, the articulated body
      // inertia of this body B as felt by body P and expressed in frame W.
      // Symmetrize the computation to reduce floating point errors.
      // TODO(amcastro-tri): Notice that the line below makes the implicit
      // assumption that g_PB_W * U_B_W is SPD and only the lower triangular
      // portion is used, see the documentation for ArticulatedBodyInertia's
      // constructor. This assumption is only asserted during debug builds. This
      // *might* result in the accumulation of floating point round off errors
      // for long kinematic chains. Further investigation is required.
      Pplus_PB_W -= ArticulatedBodyInertia<T>(g_PB_W * U_B_W);
    }
  }

  // This method is used by MultibodyTree within a tip-to-base loop to compute
  // the force bias terms in the articulated body algorithm. Please refer to
  // @ref internal_forward_dynamics
  // "Articulated Body Algorithm Forward Dynamics" for further mathematical
  // background and implementation details.
  //
  // @param[in] context
  //   The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[in] vc
  //   An already updated velocity kinematics cache in sync with `context`.
  //   All velocities are assumed to be zero if vc is nullptr.
  // @param[in] Fb_Bo_W
  //   Force bias for this node's body B, at Bo, expressed in the world frame.
  // @param[in] abic
  //   An already updated articulated body inertia cache in sync with
  //   `context`.
  // @param[in] Zb_Bo_W
  //   Articulated body bias `Zb_Bo_W = Pplus_PB_W * Ab_WB`.
  // @param[in] Fapplied_Bo_W
  //   Externally applied spatial force on this node's body B at the body's
  //   frame origin `Bo`, expressed in the world frame.
  // @param[in] tau_applied
  //   Externally applied generalized force at this node's mobilizer. It must
  //   have a size equal to the number of generalized velocities for this
  //   node's mobilizer, see get_num_mobilizer_velocities().
  // @param[in] H_PB_W
  //   The hinge mapping matrix that relates to the spatial velocity `V_PB_W`
  //   of this node's body B in its parent node body P, expressed in the world
  //   frame W, with this node's generalized velocities (or mobilities) `v_B`
  //   by `V_PB_W = H_PB_W⋅v_B`.
  // @param[out] aba_force_cache
  //   A pointer to a valid, non nullptr, force bias cache.
  //
  // @pre pc, vc, and abic previously computed to be in sync with `context.
  // @pre CalcArticulatedBodyForceCache_TipToBase() must have already been
  // called for all the child nodes of `this` node (and, by recursive
  // precondition, all successor nodes in the tree.)
  //
  // @throws when called on the _root_ node or `aba_force_cache` is
  // nullptr.
  void CalcArticulatedBodyForceCache_TipToBase(
      const systems::Context<T>&,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>*,
      const SpatialForce<T>& Fb_Bo_W,
      const ArticulatedBodyInertiaCache<T>& abic,
      const SpatialForce<T>& Zb_Bo_W,
      const SpatialForce<T>& Fapplied_Bo_W,
      const Eigen::Ref<const VectorX<T>>& tau_applied,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      ArticulatedBodyForceCache<T>* aba_force_cache) const {
    DRAKE_THROW_UNLESS(topology_.body != world_index());
    DRAKE_THROW_UNLESS(aba_force_cache != nullptr);

    // As a guideline for developers, please refer to @ref
    // internal_forward_dynamics for a detailed description of the algorithm and
    // notation inuse.

    // Compute the residual spatial force, Z_Bo_W, according to (1).
    SpatialForce<T> Z_Bo_W = Fb_Bo_W - Fapplied_Bo_W;

    // Add residual spatial force contributions from all children.
    for (const BodyNode<T>* child : children_) {
      // Shift vector from Co to Bo.
      const Vector3<T>& p_BoCo_W = child->get_p_PoBo_W(pc);
      const Vector3<T> p_CoBo_W = -p_BoCo_W;

      // Pull Zplus_BC_W from cache (which is Zplus_PB_W for child).
      const SpatialForce<T>& Zplus_BC_W =
          child->get_Zplus_PB_W(*aba_force_cache);

      // Shift Zplus_BC_W to Zplus_BCb_W.
      const SpatialForce<T> Zplus_BCb_W = Zplus_BC_W.Shift(p_CoBo_W);

      // Add Zplus_BCb_W contribution to residual spatial force.
      Z_Bo_W += Zplus_BCb_W;
    }

    get_mutable_Zplus_PB_W(aba_force_cache) = Z_Bo_W + Zb_Bo_W;

    const int nv = get_num_mobilizer_velocities();

    // These terms do not show up for zero mobilities (weld).
    if (nv != 0) {
      // Compute the articulated body inertia innovations generalized force,
      // e_B,
      // according to (4).
      VectorUpTo6<T>& e_B = get_mutable_e_B(aba_force_cache);
      e_B = tau_applied - H_PB_W.transpose() * Z_Bo_W.get_coeffs();

      // Get the Kalman gain from cache.
      const Matrix6xUpTo6<T>& g_PB_W = get_g_PB_W(abic);

      // Compute the projected articulated body force bias Zplus_PB_W.
      get_mutable_Zplus_PB_W(aba_force_cache) +=
          SpatialForce<T>(g_PB_W * e_B);
    }
  }

  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // the generalized accelerations `vdot` and the spatial accelerations `A_WB`.
  // Please refer to @ref internal_forward_dynamics
  // "Articulated Body Algorithm Forward Dynamics" for further mathematical
  // background and implementation details.
  //
  // @param[in] context
  //   The context with the state of the MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with `context`.
  // @param[in] abic
  //   An already updated articulated body inertia cache in sync with
  //   `context`.
  // @param[in] aba_force_cache
  //   An already updated articulated body algorithm cache in sync with
  //   `context`.
  // @param[in] H_PB_W
  //   The hinge mapping matrix that relates to the spatial velocity `V_PB_W`
  //   of this node's body B in its parent node body P, expressed in the world
  //   frame W, with this node's generalized velocities (or mobilities) `v_B`
  //   by `V_PB_W = H_PB_W⋅v_B`.
  // @param[in] Ab_WB
  //   The spatial acceleration bias term `Ab_WB` as it appears in the
  //   acceleration level motion constraint imposed by body B's mobilizer
  //   `A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B`.
  //   See @ref abi_computing_accelerations for further details.
  // @param[out] ac
  //   A pointer to a valid, non nullptr, acceleration kinematics cache.
  //
  // @pre pc, vc, and abic previously computed to be in sync with `context.
  // @pre CalcArticulatedBodyAccelerations_BaseToTip() must have already been
  // called for the parent node (and, by recursive precondition, all
  // predecessor nodes in the tree.)
  // @throws when called on the _root_ node of `ac` or `vdot` is nullptr.
  void CalcArticulatedBodyAccelerations_BaseToTip(
      const systems::Context<T>& /* context */,
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
    const SpatialAcceleration<T>& A_WP = parent_node_->get_A_WB(*ac);

    // Shift vector p_PoBo_W from the parent origin to the body origin.
    const Vector3<T>& p_PoBo_W = get_p_PoBo_W(pc);

    const int nv = get_num_mobilizer_velocities();

    // Rigidly shift the acceleration of the parent node.
    const SpatialAcceleration<T> Aplus_WB = SpatialAcceleration<T>(
        A_WP.rotational(),
        A_WP.translational() + A_WP.rotational().cross(p_PoBo_W));

    SpatialAcceleration<T>& A_WB = get_mutable_A_WB(ac);
    A_WB = Aplus_WB + Ab_WB;

    // These quantities do not contribute when nv = 0. We skip them since Eigen
    // does not allow certain operations on zero-sized objects.
    if (nv != 0) {
      // Compute nu_B, the articulated body inertia innovations generalized
      // acceleration.
      const VectorUpTo6<T> nu_B =
          get_ldlt_D_B(abic).Solve(get_e_B(aba_force_cache));

      // Mutable reference to the generalized acceleration.
      auto vmdot = get_mutable_accelerations(ac);
      const Matrix6xUpTo6<T>& g_PB_W = get_g_PB_W(abic);
      vmdot = nu_B - g_PB_W.transpose() * A_WB.get_coeffs();

      // Update with vmdot term the spatial acceleration of the current body.
      A_WB += SpatialAcceleration<T>(H_PB_W * vmdot);
    }
  }

  // This method is used by MultibodyTree within a tip-to-base loop to compute
  // the composite body inertia of each body in the system.
  //
  // @param[in] M_B_W Spatial inertia for the body B associated with this node.
  // About B's origin Bo and expressed in the world frame W.
  // @param[in] pc Position kinematics cache.
  // @param[in] Mc_B_W_all Vector storing the composite body inertia for all
  // bodies in the multibody system. It must contain already up-to-date
  // composite body inertias for all the children of `this` node.
  // @param[out] Mc_B_W The composite body inertia for `this` node. It must be
  // non-nullptr.
  // @pre CalcCompositeBodyInertia_TipToBase() must have already been called
  // for the children nodes (and, by recursive precondition, all outboard nodes
  // in the tree.)
  void CalcCompositeBodyInertia_TipToBase(
      const SpatialInertia<T>& M_B_W,
      const PositionKinematicsCache<T>& pc,
      const std::vector<SpatialInertia<T>>& Mc_B_W_all,
      SpatialInertia<T>* Mc_B_W) const {
    DRAKE_THROW_UNLESS(topology_.body != world_index());
    DRAKE_THROW_UNLESS(Mc_B_W != nullptr);

    // Composite body inertia R_B_W for this node B, about its frame's origin
    // Bo, and expressed in the world frame W. Here we adopt the notation used
    // in Jain's book.
    *Mc_B_W = M_B_W;
    // Add composite body inertia contributions from all children.
    for (const BodyNode<T>* child : children_) {
      // Shift vector p_CoBo_W.
      const Vector3<T>& p_BoCo_W = child->get_p_PoBo_W(pc);
      const Vector3<T> p_CoBo_W = -p_BoCo_W;

      // Composite body inertia for outboard child body C, about Co, expressed
      // in W.
      const SpatialInertia<T>& Mc_CCo_W = Mc_B_W_all[child->index()];

      // Shift to Bo and add it to the composite body inertia of B.
      *Mc_B_W += Mc_CCo_W.Shift(p_CoBo_W);
    }
  }

  // Computes the spatial acceleration bias `Ab_WB(q, v)` for `this` node, a
  // function of both configuration q and velocities v. This term appears in
  // the acceleration level motion constraint imposed by body B's mobilizer
  // `A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B`. Refer to
  // @ref abi_computing_accelerations for a detailed description and
  // derivation.
  // @param[in] context The context with the state of the MultibodyTree model.
  // @param[in] pc An already updated position kinematics cache in sync with
  //   `context`.
  // @param[in] vc An already updated velocity kinematics cache in sync with
  //   `context`.
  // @param[out] Ab_WB The spatial acceleration bias for this node, measured
  //   and expressed in the world frame W. Must be non nullptr.
  //
  // @pre pc and vc previously computed to be in sync with `context.
  //
  // @throws when `Ab_WB` is nullptr.
  void CalcSpatialAccelerationBias(const systems::Context<T>& context,
                                   const PositionKinematicsCache<T>& pc,
                                   const VelocityKinematicsCache<T>& vc,
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
    const math::RotationMatrix<T> R_PF =
        frame_F.CalcRotationMatrixInBodyFrame(context);
    const math::RigidTransform<T> X_MB =
        frame_M.CalcPoseInBodyFrame(context).inverse();

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
    const VectorUpTo6<T> vmdot_zero =
        VectorUpTo6<T>::Zero(get_num_mobilizer_velocities());
    const SpatialAcceleration<T> Ab_FM =
        get_mobilizer().CalcAcrossMobilizerSpatialAcceleration(context,
                                                               vmdot_zero);

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

    // Velocity of body in parent is available from the velocity kinematics.
    const SpatialVelocity<T>& V_PB_W = get_V_PB_W(vc);
    const Vector3<T>& w_PB_W = V_PB_W.rotational();
    const Vector3<T>& v_PB_W = V_PB_W.translational();

    // Body spatial velocity in W.
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

 protected:
  // Returns the inboard frame F of this node's mobilizer.
  // @throws std::exception if called on the root node corresponding to
  // the _world_ body.
  const Frame<T>& inboard_frame() const {
    return get_mobilizer().inboard_frame();
  }

  // Returns the outboard frame M of this node's mobilizer.
  // @throws std::exception if called on the root node corresponding to
  // the _world_ body.
  const Frame<T>& outboard_frame() const {
    return get_mobilizer().outboard_frame();
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
      const systems::Context<T>& context) const {
    return this->get_parent_tree().get_state_segment(context,
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
    return v.segment(topology_.mobilizer_velocities_start_in_v,
                     topology_.num_mobilizer_velocities);
  }

  // =========================================================================
  // PositionKinematicsCache Accessors and Mutators.

  // Returns a const reference to the pose of the body B associated with this
  // node as measured and expressed in the world frame W.
  const math::RigidTransform<T>& get_X_WB(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(topology_.index);
  }

  // Mutable version of get_X_WB().
  math::RigidTransform<T>& get_mutable_X_WB(
      PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(topology_.index);
  }

  // Returns a const reference to the pose of the parent body P measured and
  // expressed in the world frame W.
  const math::RigidTransform<T>& get_X_WP(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(topology_.parent_body_node);
  }

  // Returns a const reference to the rotation matrix `R_WP` that relates the
  // orientation of the world frame W to the parent frame P.
  const math::RotationMatrix<T>& get_R_WP(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_R_WB(topology_.parent_body_node);
  }

  // Returns a constant reference to the across-mobilizer pose of the outboard
  // frame M as measured and expressed in the inboard frame F.
  const math::RigidTransform<T>& get_X_FM(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_FM(topology_.index);
  }

  // Returns a mutable reference to the across-mobilizer pose of the outboard
  // frame M as measured and expressed in the inboard frame F.
  math::RigidTransform<T>& get_mutable_X_FM(
      PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_FM(topology_.index);
  }

  // Returns a const reference to the pose of body B as measured and expressed
  // in the frame of the parent body P.
  const math::RigidTransform<T>& get_X_PB(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_PB(topology_.index);
  }

  // Returns a mutable reference to the pose of body B as measured and expressed
  // in the frame of the parent body P.
  math::RigidTransform<T>& get_mutable_X_PB(
      PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.index);
  }

  const Vector3<T>& get_p_PoBo_W(const PositionKinematicsCache<T>& pc) const {
    return pc.get_p_PoBo_W(topology_.index);
  }

  Vector3<T>& get_mutable_p_PoBo_W(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_p_PoBo_W(topology_.index);
  }

  // =========================================================================
  // VelocityKinematicsCache Accessors and Mutators.

  // For the body B associated with this node, return V_WB, B's spatial velocity
  // in the world frame W, expressed in W (for Bo, the body frame's origin).
  const SpatialVelocity<T>& get_V_WB(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(topology_.index);
  }

  // Mutable version of get_V_WB().
  SpatialVelocity<T>& get_mutable_V_WB(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_WB(topology_.index);
  }

  // Returns the spatial velocity `V_WP` of the body frame P in the parent node
  // as measured and expressed in the world frame.
  const SpatialVelocity<T>& get_V_WP(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(topology_.parent_body_node);
  }

  // Returns a const reference to the across-mobilizer spatial velocity `V_FM`
  // of the outboard frame M in the inboard frame F, expressed in the F frame.
  const SpatialVelocity<T>& get_V_FM(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_FM(topology_.index);
  }

  // Mutable version of get_V_FM().
  SpatialVelocity<T>& get_mutable_V_FM(
      VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_FM(topology_.index);
  }

  // Returns a const reference to the spatial velocity `V_PB_W` of `this`
  // node's body B in the parent node's body P, expressed in the world frame W.
  const SpatialVelocity<T>& get_V_PB_W(
      const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_PB_W(topology_.index);
  }

  // Mutable version of get_V_PB_W().
  SpatialVelocity<T>& get_mutable_V_PB_W(
      VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_PB_W(topology_.index);
  }

  // =========================================================================
  // AccelerationKinematicsCache Accessors and Mutators.

  // For the body B associated with `this` node, returns A_WB, body B's
  // spatial acceleration in the world frame W, expressed in W
  // (for point Bo, the body's origin).
  const SpatialAcceleration<T>& get_A_WB(
      const AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(topology_.index);
  }

  // Mutable version of get_A_WB().
  SpatialAcceleration<T>& get_mutable_A_WB(
      AccelerationKinematicsCache<T>* ac) const {
    return ac->get_mutable_A_WB(topology_.index);
  }

  // Returns a const reference to the spatial acceleration `A_WP` of the body
  // frame P in the parent node as measured and expressed in the world frame.
  const SpatialAcceleration<T>& get_A_WP(
      const AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(topology_.parent_body_node);
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

  // Returns a const reference to the articulated body inertia `P_B_W` of the
  // body taken about Bo and expressed in W.
  const ArticulatedBodyInertia<T>& get_P_B_W(
      const ArticulatedBodyInertiaCache<T>& abic) const {
    return abic.get_P_B_W(topology_.index);
  }

  // Mutable version of get_P_B_W().
  ArticulatedBodyInertia<T>& get_mutable_P_B_W(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_P_B_W(topology_.index);
  }

  // Returns a const reference to the articulated body inertia `Pplus_PB_W`,
  // which can be thought of as the articulated body inertia of parent body P
  // as though it were inertialess, but taken about Bo and expressed in W.
  const ArticulatedBodyInertia<T>& get_Pplus_PB_W(
      const ArticulatedBodyInertiaCache<T>& abic) const {
    return abic.get_Pplus_PB_W(topology_.index);
  }

  // Mutable version of get_Pplus_PB_W().
  ArticulatedBodyInertia<T>& get_mutable_Pplus_PB_W(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_Pplus_PB_W(topology_.index);
  }

  // Returns a const reference to the LDLT factorization `ldlt_D_B` of the
  // articulated body hinge inertia.
  const math::LinearSolver<Eigen::LDLT, MatrixUpTo6<T>>& get_ldlt_D_B(
      const ArticulatedBodyInertiaCache<T>& abic) const {
    return abic.get_ldlt_D_B(topology_.index);
  }

  // Mutable version of get_ldlt_D_B().
  math::LinearSolver<Eigen::LDLT, MatrixUpTo6<T>>& get_mutable_ldlt_D_B(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_ldlt_D_B(topology_.index);
  }

  // Returns a const reference to the Kalman gain `g_PB_W` of the body.
  const Matrix6xUpTo6<T>& get_g_PB_W(
      const ArticulatedBodyInertiaCache<T>& abic) const {
    return abic.get_g_PB_W(topology_.index);
  }

  // Mutable version of get_g_PB_W().
  Matrix6xUpTo6<T>& get_mutable_g_PB_W(
      ArticulatedBodyInertiaCache<T>* abic) const {
    return abic->get_mutable_g_PB_W(topology_.index);
  }

  // =========================================================================
  // ArticulatedBodyForceCache Accessors and Mutators.

  // Returns a const reference to the articulated body inertia residual force
  // `Zplus_PB_W` for this body projected across its inboard mobilizer to
  // frame P.
  const SpatialForce<T>& get_Zplus_PB_W(
      const ArticulatedBodyForceCache<T>& aba_force_cache) const {
    return aba_force_cache.get_Zplus_PB_W(topology_.index);
  }

  // Mutable version of get_Zplus_PB_W().
  SpatialForce<T>& get_mutable_Zplus_PB_W(
      ArticulatedBodyForceCache<T>* aba_force_cache) const {
    return aba_force_cache->get_mutable_Zplus_PB_W(topology_.index);
  }

  // Returns a const reference to the Coriolis spatial acceleration `Ab_WB`
  // for this body due to the relative velocities of body B and body P.
  const SpatialAcceleration<T>& get_Ab_WB(
      const ArticulatedBodyForceCache<T>& aba_force_cache) const {
    return aba_force_cache.get_Ab_WB(topology_.index);
  }

  // Mutable version of get_Ab_WB().
  SpatialAcceleration<T>& get_mutable_Ab_WB(
      ArticulatedBodyForceCache<T>* aba_force_cache) const {
    return aba_force_cache->get_mutable_Ab_WB(topology_.index);
  }

  // Returns a const reference to the Coriolis spatial acceleration `Ab_WB`
  // for this body due to the relative velocities of body B and body P.
  const VectorUpTo6<T>& get_e_B(
      const ArticulatedBodyForceCache<T>& aba_force_cache) const {
    return aba_force_cache.get_e_B(topology_.index);
  }

  // Mutable version of get_e_B().
  VectorUpTo6<T>& get_mutable_e_B(
      ArticulatedBodyForceCache<T>* aba_force_cache) const {
    return aba_force_cache->get_mutable_e_B(topology_.index);
  }

  // =========================================================================
  // Per Node Array Accessors.
  // Quantities are ordered by BodyNodeIndex unless otherwise specified.

  // Returns a const reference to the spatial acceleration of the body B
  // associated with this node as measured and expressed in the world frame W,
  // given an array of spatial accelerations for the entire MultibodyTree model.
  const SpatialAcceleration<T>& get_A_WB_from_array(
      const std::vector<SpatialAcceleration<T>>& A_WB_array) const {
    return A_WB_array[topology_.index];
  }

  // Mutable version of get_A_WB_from_array().
  SpatialAcceleration<T>& get_mutable_A_WB_from_array(
      std::vector<SpatialAcceleration<T>>* A_WB_array) const {
    DRAKE_ASSERT(A_WB_array != nullptr);
    return (*A_WB_array)[topology_.index];
  }

  // Returns a const reference to the spatial acceleration `A_WP` of the body
  // frame P in the parent node as measured and expressed in the world frame,
  // given an array of spatial accelerations for the entire MultibodyTree model.
  const SpatialAcceleration<T>& get_A_WP_from_array(
      const std::vector<SpatialAcceleration<T>>& A_WB_array) const {
    return A_WB_array[topology_.parent_body_node];
  }

  // Mutable version of get_A_WP_from_array().
  SpatialAcceleration<T>& get_mutable_A_WP_from_array(
      std::vector<SpatialAcceleration<T>>* A_WB_array) const {
    DRAKE_ASSERT(A_WB_array != nullptr);
    return (*A_WB_array)[topology_.parent_body_node];
  }

  // Helper to get an Eigen expression of the vector of generalized velocities
  // from a vector of generalized velocities for the entire parent multibody
  // tree. Useful for the implementation of operator forms where the generalized
  // velocity (or time derivatives of the generalized velocities) is an argument
  // to the operator.
  Eigen::Ref<VectorX<T>> get_mutable_velocities_from_array(
      EigenPtr<VectorX<T>> v) const {
    DRAKE_ASSERT(v != nullptr);
    return v->segment(topology_.mobilizer_velocities_start_in_v,
                      topology_.num_mobilizer_velocities);
  }

  // Helper to get an Eigen expression of the vector of generalized forces
  // from a vector of generalized forces for the entire parent multibody
  // tree.
  Eigen::Ref<VectorX<T>> get_mutable_generalized_forces_from_array(
      EigenPtr<VectorX<T>> tau) const {
    DRAKE_ASSERT(tau != nullptr);
    return get_mutable_velocities_from_array(tau);
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
      const systems::Context<T>& context,
      PositionKinematicsCache<T>* pc) const {
    // Body for this node.
    const Body<T>& body_B = body();

    // Body for this node's parent, or the parent body P.
    const Body<T>& body_P = parent_body();

    // Inboard/Outboard frames of this node's mobilizer.
    const Frame<T>& frame_F = get_mobilizer().inboard_frame();
    DRAKE_ASSERT(frame_F.body().index() == body_P.index());
    const Frame<T>& frame_M = get_mobilizer().outboard_frame();
    DRAKE_ASSERT(frame_M.body().index() == body_B.index());

    // Input (const):
    // - X_PF(qb_P)
    // - X_MB(qb_B)
    // - X_FM(qm_B)
    // - X_WP(q(W:B)), where q(W:B) includes all positions in the kinematics
    //                 path from body B to the world W.
    const math::RigidTransform<T> X_MB =
        frame_M.CalcPoseInBodyFrame(context).inverse();
    const math::RigidTransform<T>& X_FM =
        get_X_FM(*pc);  // mobilizer.Eval_X_FM(ctx)
    const math::RigidTransform<T>& X_WP =
        get_X_WP(*pc);  // body_P.EvalPoseInWorld(ctx)

    // Output (updating a cache entry):
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    math::RigidTransform<T>& X_PB = get_mutable_X_PB(pc);
    math::RigidTransform<T>& X_WB =
        get_mutable_X_WB(pc);  // body_B.EvalPoseInWorld(ctx)

    // TODO(amcastro-tri): Consider logic for the common case B = M.
    // In that case X_FB = X_FM as suggested by setting X_MB = Id.
    const math::RigidTransform<T> X_FB = X_FM * X_MB;

    // Given the pose X_FB of body frame B measured in the mobilizer inboard
    // frame F, we can ask frame F (who's parent body is P) for the pose of body
    // B measured in the frame of the parent body P.
    // In the particular case F = B, this method directly returns X_FB.
    // For flexible bodies this gives the chance to frame F to pull its pose
    // from the context.
    X_PB = frame_F.CalcOffsetPoseInBody(context, X_FB);

    X_WB = X_WP * X_PB;

    // Compute shift vector p_PoBo_W from the parent origin to the body origin.
    const Vector3<T>& p_PoBo_P = X_PB.translation();
    const math::RotationMatrix<T>& R_WP = X_WP.rotation();
    get_mutable_p_PoBo_W(pc) = R_WP * p_PoBo_P;
  }

  // Computes position dependent kinematics associated with `this` mobilizer
  // which includes:
  // - X_FM(q): The pose of the outboard frame M as measured and expressed in
  //            the inboard frame F.
  // - H_FM(q): the mobilizer hinge matrix that relates the mobilizer's
  //            generalized velocities v to the spatial velocity `V_FM` by
  //            `V_FM(q, v) = H_FM(q) * v`.
  // - Hdot_FM(q): The time derivative of H_FM which is used to computing the
  //               M's spatial acceleration in frame F, expressed in F as:
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
      const systems::Context<T>& context,
      PositionKinematicsCache<T>* pc) const {
    DRAKE_ASSERT(pc != nullptr);
    math::RigidTransform<T>& X_FM = get_mutable_X_FM(pc);
    X_FM = get_mobilizer().CalcAcrossMobilizerTransform(context);
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
  //      When Fb_Bo_W_cache is nullptr velocites are considered to be zero.
  //      Therefore, from (2), the bias term is assumed to be zero and is not
  //      computed.
  void CalcBodySpatialForceGivenItsSpatialAcceleration(
      const std::vector<SpatialInertia<T>>& M_B_W_cache,
      const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
      const SpatialAcceleration<T>& A_WB, SpatialForce<T>* Ftot_BBo_W_ptr)
  const {
    DRAKE_DEMAND(Ftot_BBo_W_ptr != nullptr);
    // TODO(amcastro-tri): add argument for flexible body generalized
    // accelerations and generalized forces.

    // Output spatial force applied on Body B, at Bo, measured in W.
    SpatialForce<T>& Ftot_BBo_W = *Ftot_BBo_W_ptr;

    // Body for this node.
    const Body<T>& body_B = body();

    // Body B spatial inertia about Bo expressed in world W.
    const SpatialInertia<T>& M_B_W = M_B_W_cache[body_B.node_index()];

    // Equations of motion for a rigid body written at a generic point Bo not
    // necessarily coincident with the body's center of mass. This corresponds
    // to Eq. 2.26 (p. 27) in A. Jain's book.
    Ftot_BBo_W = M_B_W * A_WB;

    // If velocities are zero, then Fb_Bo_W is zero and does not contribute.
    if (Fb_Bo_W_cache != nullptr) {
      // Dynamic bias for body B.
      const SpatialForce<T>& Fb_Bo_W = (*Fb_Bo_W_cache)[body_B.node_index()];
      Ftot_BBo_W += Fb_Bo_W;
    }
  }

  // Implementation for MultibodyElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_body_node(this->index());
  }

  BodyNodeTopology topology_;

  const BodyNode<T>* parent_node_{nullptr};
  std::vector<const BodyNode<T>*> children_;

  // Pointers for fast access.
  const Body<T>* body_;
  const Mobilizer<T>* mobilizer_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
