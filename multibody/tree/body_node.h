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
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/rigid_body.h"
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
// In summary, there will a BodyNode for each RigidBody in the MultibodyTree
// which encompasses:
//
// - a body B in a given MultibodyTree,
// - the outboard frame M attached to this body B,
// - the inboard frame F attached to the unique parent body P of body B,
// - the mobilizer connecting the inboard frame F with the outboard frame M.
//
// <h4>Associated State</h4>
//
// In the same way a Mobilizer and a RigidBody have a number of generalized
// positions associated with them, a %BodyNode is associated with the
// generalized positions of body B and of its inboard mobilizer.
//
// The relationship between frames F and M is dictated by the body B's inboard
// mobilizer providing the pose `X_FM(q_B)` as a function of the generalized
// coordinates `q_B` (where `_B` means these are the q's for just the unique
// inboard mobilizer of body B.)
//
// @tparam_default_scalar
template <typename T>
class BodyNode : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNode);

  // A node encompasses a RigidBody in a MultibodyTree and the inboard Mobilizer
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
  //
  // Reference used below:
  // - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
  //                algorithms. Springer Science & Business Media.
  BodyNode(const BodyNode<T>* parent_node, const RigidBody<T>* body,
           const Mobilizer<T>* mobilizer)
      : MultibodyElement<T>(body->model_instance()),
        parent_node_(parent_node),
        body_(body),
        mobilizer_(mobilizer) {
    DRAKE_DEMAND(!(parent_node == nullptr && body->index() != world_index()));
    this->set_is_ephemeral(true);  // BodyNodes are never added by users.
  }

  ~BodyNode() override;

  // The Mobod associated with this BodyNode (and its Mobilizer).
  const SpanningForest::Mobod& mobod() const { return get_mobilizer().mobod(); }

  // The index of the associated Mobod (same as the index of this BodyNode
  // and of its Mobilizer).
  MobodIndex mobod_index() const { return mobod().index(); }

  // Method to update the list of child body nodes maintained by this node,
  // outboard to this node. Recall a %BodyNode is a tree node within the tree
  // structure of MultibodyTree. Therefore each %BodyNode has a unique parent
  // %BodyNode, supplied at construction, and a set of child nodes, specified
  // via calls to this method.
  // Used by MultibodyTree at creation of a BodyNode during the
  // MultibodyTree::Finalize() method call.
  void add_child_node(const BodyNode<T>* child) { children_.push_back(child); }

  MobodIndex inboard_mobod_index() const { return mobod().inboard(); }

  // Returns a constant reference to the body B associated with this node.
  const RigidBody<T>& body() const {
    DRAKE_ASSERT(body_ != nullptr);
    return *body_;
  }

  // Returns a constant reference to the unique parent body P of the body B
  // associated with this node. This method aborts in Debug builds if called on
  // the root node corresponding to the _world_ body.
  // @pre has_parent_tree() is true.
  const RigidBody<T>& parent_body() const {
    DRAKE_ASSERT(get_parent_body_index().is_valid());
    DRAKE_ASSERT(this->has_parent_tree());
    return this->get_parent_tree().get_body(get_parent_body_index());
  }

  // Returns a const pointer to the parent (inboard) body node or nullptr if
  // `this` is the world node, which has no inboard parent node.
  const BodyNode<T>* parent_body_node() const { return parent_node_; }

  // Returns a vector of pointers to the BodyNodes for which this is the
  // inboard node.
  const std::vector<const BodyNode<T>*>& child_nodes() const {
    return children_;
  }

  // Returns a constant reference to the mobilizer associated with this node.
  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_ASSERT(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  // @name Methods to retrieve BodyNode sizes
  //@{

  // Returns the number of generalized positions for the Mobilizer in `this`
  // node.
  int get_num_mobilizer_positions() const { return mobod().nq(); }

  // Returns the number of generalized velocities for the Mobilizer in `this`
  // node.
  int get_num_mobilizer_velocities() const { return mobod().nv(); }

  // Returns the index to the first generalized velocity for this node
  // within the vector v of generalized velocities for the full multibody
  // system.
  int velocity_start_in_v() const { return mobod().v_start(); }
  //@}

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
    DRAKE_ASSERT(this->has_parent_tree());
    const SpanningForest& forest = this->get_parent_tree().forest();
    DRAKE_DEMAND(static_cast<int>(H_array.size()) == forest.num_velocities());
    const int start_index_in_v = mobod().v_start();
    const int nv = mobod().nv();
    DRAKE_DEMAND(nv == 0 || start_index_in_v < forest.num_velocities());
    // The first column of this node's hinge matrix H_PB_W:
    const T* H_col0 = nv == 0 ? nullptr : H_array[start_index_in_v].data();
    // Create an Eigen map to the full H_PB_W for this node:
    return Eigen::Map<const MatrixUpTo6<T>>(H_col0, 6, nv);
  }

  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // this node's kinematics that only depend on generalized positions.
  // Don't call this on the World body.
  //
  // @param[in] frame_body_pose_cache parameterized frame offsets
  // @param[in] positions
  //   The current position coordinates q for the full MultibodyTree model.
  // @param[out] pc A pointer to a valid, non nullptr, kinematics cache.
  // @pre CalcPositionKinematicsCache_BaseToTip() must have already been called
  // for the parent node (and, by recursive precondition, all predecessor nodes
  // in the tree.)
  virtual void CalcPositionKinematicsCache_BaseToTip(
      const FrameBodyPoseCache<T>& frame_body_pose_cache, const T* positions,
      PositionKinematicsCache<T>* pc) const = 0;

  // Calculates the hinge matrix H_PB_W, the `6 x nm` hinge matrix that relates
  // `V_PB_W`(body B's spatial velocity in its parent body P, expressed in world
  // W) to this node's nm generalized velocities (or mobilities) v_B as
  // V_PB_W = H_PB_W * v_B.
  //
  // @param[in] frame_body_pose_cache parameterized frame offsets
  // @param[in] positions
  //   The current position coordinates q for the full MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with positions.
  // @param[out] H_PB_W_cache
  //   The cache entry being calculated; just this node's H_PB is updated.

  // @note `H_PB_W` is only a function of this node's generalized positions q.
  //
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with positions by MultibodyTree::CalcPositionKinematicsCache().
  virtual void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const FrameBodyPoseCache<T>& frame_body_pose_cache, const T* positions,
      const PositionKinematicsCache<T>& pc,
      std::vector<Vector6<T>>* H_PB_W_cache) const = 0;

  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // this node's kinematics that depend on the generalized velocities.
  // Don't call this on World.
  // @param[in] positions
  //   The current position coordinates q for the full MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with positions.
  // @param[in] H_PB_W_cache
  //   Already calculated hinge matrices; for more information see
  //   CalcAcrossNodeJacobianWrtVExpressedInWorld().
  // @param[in] velocities
  //   The current velocity coordinates v for the full MultibodyTree model.
  // @param[out] vc
  //   A pointer to a valid, non nullptr, velocity kinematics cache.
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with positions by MultibodyTree::CalcPositionKinematicsCache().
  // @pre CalcVelocityKinematicsCache_BaseToTip() must have already been called
  // for the parent node (and, by recursive precondition, all predecessor nodes
  // in the tree.)
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcVelocityKinematicsCache().
  virtual void CalcVelocityKinematicsCache_BaseToTip(
      const T* positions, const PositionKinematicsCache<T>& pc,
      const std::vector<Vector6<T>>& H_PB_W_cache, const T* velocities,
      VelocityKinematicsCache<T>* vc) const = 0;

  // The CalcMassMatrix() algorithm invokes this on each body k, serving
  // as the composite body R(k) in the outer loop of Jain's algorithm 9.3.
  // This node must fill in its nv x nv diagonal block in M, and then
  // sweep down to World filling in its diagonal contributions as in the
  // inner loop of algorithm 9.3, using the appropriate
  // CalcMassMatrixOffDiagonalViaWorldHelper().
  virtual void CalcMassMatrixContributionViaWorld_TipToBase(
      const PositionKinematicsCache<T>& pc,
      const std::vector<SpatialInertia<T>>& K_BBo_W_cache,  // composites
      const std::vector<Vector6<T>>& H_PB_W_cache,
      EigenPtr<MatrixX<T>> M) const = 0;

  // There are six functions for calculating the off-diagonal blocks, one for
  // each possible size Bnv of body B(k)'s inboard mobilizer (welds don't
  // contribute here). This allows us to use fixed-size 2d matrices in the
  // implementation as we sweep the inboard bodies. Use the separate dispatcher
  // class CalcMassMatrixOffDiagonalViaWorldDispatcher (defined below) to
  // generate the properly sized call for body B(k).
#define DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(Bnv)           \
  virtual void CalcMassMatrixOffDiagonalBlockViaWorld##Bnv(             \
      int B_start_in_v, const std::vector<Vector6<T>>& H_PB_W_cache,    \
      const Eigen::Matrix<T, 6, Bnv>& Fm_CBo_W, EigenPtr<MatrixX<T>> M) \
      const = 0

  DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(1);
  DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(2);
  DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(3);
  DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(4);
  DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(5);
  DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(6);

#undef DECLARE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD

  // This method is used by MultibodyTree within a base-to-tip loop to compute
  // this node's kinematics that depend on the generalized accelerations, i.e.
  // the generalized velocities' time derivatives.
  // Don't call this on World.
  //
  // @param[in] frame_body_pose_cache parameterized frame offsets
  // @param[in] positions
  //   The current position coordinates q for the full MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with positions.
  // @param[in] velocities
  //   The current velocity coordinates v for the full MultibodyTree model.
  // @param[in] vc
  //   An already updated velocity kinematics cache in sync with velocities.
  //   If vc is nullptr, velocities are assumed to be zero and velocity
  //   dependent terms are not computed.
  // @param[in] accelerations
  //   The entire vector of generalized accelerations for the full
  //   MultibodyTree model. It must have a size equal to the number of
  //   generalized velocities in the model.
  // @param[in,out] A_WB_array
  //   A pointer to a valid, non nullptr, vector of spatial accelerations
  //   containing the spatial acceleration `A_WB` for each body. On input, it
  //   must contain already pre-computed spatial accelerations for the inboard
  //   bodies to this node's body B, see precondition below.  It must be of
  //   size equal to the number of bodies in the MultibodyTree and ordered by
  //   MobodIndex. The calling MultibodyTree method must guarantee these
  //   conditions are satisfied. This method will abort if the pointer is
  //   null. There is no mechanism to assert that `A_WB_array` is ordered
  //   by MobodIndex and the correctness of MultibodyTree methods, properly
  //   unit tested, should guarantee this condition.
  //
  // @pre The position kinematics cache `pc` was already updated to be in sync
  // with positions by MultibodyTree::CalcPositionKinematicsCache().
  // @pre The velocity kinematics cache `vc` was already updated to be in sync
  // with velocities by MultibodyTree::CalcVelocityKinematicsCache().
  // @pre CalcAccelerationKinematicsCache_BaseToTip() must have already been
  // called for the parent node (and, by recursive precondition, all
  // predecessor nodes in the tree). Therefore, on input, the argument array
  // `A_WB_array` must contain already pre-computed spatial accelerations
  // for the inboard bodies to this node's body B.
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcAccelerationKinematicsCache().
  virtual void CalcSpatialAcceleration_BaseToTip(
      const FrameBodyPoseCache<T>& frame_body_poses_cache, const T* positions,
      const PositionKinematicsCache<T>& pc, const T* velocities,
      const VelocityKinematicsCache<T>* vc, const T* accelerations,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const = 0;

  // Computes the generalized forces `tau` for a single BodyNode.
  // This method is used by MultibodyTree within a tip-to-base loop to compute
  // the vector of generalized forces `tau` that would correspond with a known
  // set of spatial accelerations `A_WB` for each body in the MultibodyTree.
  //
  // This method aborts in Debug builds when `F_BMo_W_array` is nullptr.
  //
  // @param[in] frame_body_pose_cache parameterized frame offsets
  // @param[in] positions
  //   The current position coordinates q for the full MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with positions.
  // @param[in] M_B_W_cache precalculated spatial inertias in World
  //   Already up to date cache entries
  // @param[in] Fb_Bo_W_cache velocity-dependent bias terms
  //   Null if we're ignoring velocity, otherwise must be up to date.
  // @param[in] A_WB_array
  //   A vector of known spatial accelerations containing the spatial
  //   acceleration A_WB for each body in the MultibodyTree model. It must be
  //   of size equal to the number of bodies in the MultibodyTree and ordered
  //   by MobodIndex. The calling MultibodyTree method must guarantee these
  //   conditions are satisfied.
  // @param[in] Fapplied_Bo_W_array
  //   Either zero length or num_mobods. All applied spatial forces. May be the
  //   same object as the output F_BMo_W_array in which case the body B entry
  //   will be overwritten on return.
  // @param[in] tau_applied_array
  //   Either zero length or num_velocities. All applied generalized forces. May
  //   be the same object as the output tau_array in which case the entries
  //   for body B's mobilizer will be overwritten on return.
  // @param[out] F_BMo_W_array
  //   A non-null pointer to a vector of spatial forces containing,
  //   for each body B, the spatial force `F_BMo_W` corresponding to its
  //   inboard mobilizer reaction forces on body B applied at the origin `Mo`
  //   of the inboard mobilizer, expressed in the world frame W. Note that
  //   everything outboard of body B must already have been computed!
  //   This can be the same object as Fapplied_Bo_W_array.
  // @param[out] tau_array
  //   A non-null pointer to the output vector of generalized forces that would
  //   result in body B having spatial acceleration `A_WB`. This can be the same
  //   object as tau_applied_array.
  //
  // @pre CalcInverseDynamics_TipToBase() must have already been
  // called for all the child nodes of `this` node (and, by recursive
  // precondition, all successor nodes in the tree.)
  // Unit test coverage for this method is provided, among others, in
  // double_pendulum_test.cc, and by any other unit tests making use of
  // MultibodyTree::CalcInverseDynamics().
  virtual void CalcInverseDynamics_TipToBase(
      const FrameBodyPoseCache<T>& frame_body_pose_cache, const T* positions,
      const PositionKinematicsCache<T>& pc,
      const std::vector<SpatialInertia<T>>& M_B_W_cache,
      const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
      const std::vector<SpatialAcceleration<T>>& A_WB_array,
      const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
      const Eigen::Ref<const VectorX<T>>& tau_applied_array,
      std::vector<SpatialForce<T>>* F_BMo_W_array,
      EigenPtr<VectorX<T>> tau_array) const = 0;

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
  // @param[in] diagonal_inertias
  //   Vector of scalar diagonal inertia values for each degree of freedom.
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
  // @throws if diagonal_inertias.size() does not much the number of generalized
  // velocities in the model.
  // TODO(amcastro-tri): Consider specialized BodyNodeImpl implementations that
  //  exploit the sparsity pattern of H_PB_W even at compile time. Most common
  //  cases are:
  //  - Revolute: [x y z 0 0 0]
  //  - Prismatic: [0 0 0 x y z]
  //  - Ball: 3x3 blocks of zeroes.

  // TODO(sherm1) This function should not take a context.
  virtual void CalcArticulatedBodyInertiaCache_TipToBase(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      const SpatialInertia<T>& M_B_W, const VectorX<T>& diagonal_inertias,
      ArticulatedBodyInertiaCache<T>* abic) const = 0;

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
  // @pre pc, vc, and abic previously computed to be in sync with `context`.
  // @pre CalcArticulatedBodyForceCache_TipToBase() must have already been
  // called for all the child nodes of `this` node (and, by recursive
  // precondition, all successor nodes in the tree.)
  //
  // @throws when called on the _root_ node or `aba_force_cache` is
  // nullptr.

  // TODO(sherm1) This function should not take a context.
  virtual void CalcArticulatedBodyForceCache_TipToBase(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>*, const SpatialForce<T>& Fb_Bo_W,
      const ArticulatedBodyInertiaCache<T>& abic,
      const SpatialForce<T>& Zb_Bo_W, const SpatialForce<T>& Fapplied_Bo_W,
      const Eigen::Ref<const VectorX<T>>& tau_applied,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      ArticulatedBodyForceCache<T>* aba_force_cache) const = 0;

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
  // @pre pc, vc, and abic previously computed to be in sync with `context`.
  // @pre CalcArticulatedBodyAccelerations_BaseToTip() must have already been
  // called for the parent node (and, by recursive precondition, all
  // predecessor nodes in the tree.)
  // @throws when called on the _root_ node of `ac` or `vdot` is nullptr.

  // TODO(sherm1) This function should not take a context.
  virtual void CalcArticulatedBodyAccelerations_BaseToTip(
      const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
      const ArticulatedBodyInertiaCache<T>& abic,
      const ArticulatedBodyForceCache<T>& aba_force_cache,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      const SpatialAcceleration<T>& Ab_WB,
      AccelerationKinematicsCache<T>* ac) const = 0;

  // Computes the spatial acceleration bias `Ab_WB(q, v)` for `this` node, a
  // function of both configuration q and velocities v. This term appears in
  // the acceleration level motion constraint imposed by body B's mobilizer
  // `A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B`. Refer to
  // @ref abi_computing_accelerations for a detailed description and
  // derivation.
  //
  // @param[in] frame_body_pose_cache parameterized frame offsets
  // @param[in] positions
  //   The current position coordinates q for the full MultibodyTree model.
  // @param[in] pc
  //   An already updated position kinematics cache in sync with positions.
  // @param[in] velocities
  //   The current velocity coordinates v for the full MultibodyTree model.
  // @param[in] vc An already updated velocity kinematics cache in sync with
  //   velocities.
  // @param[out] Ab_WB_array The spatial acceleration bias for all nodes,
  //   measured and expressed in the world frame W. Must be non nullptr.
  //
  // @pre pc & vc previously computed to be in sync with positions & velocities.
  // @pre Ab_WB_array is not null.
  virtual void CalcSpatialAccelerationBias(
      const FrameBodyPoseCache<T>& frame_body_pose_cache, const T* positions,
      const PositionKinematicsCache<T>& pc, const T* velocities,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialAcceleration<T>>* Ab_WB_array) const = 0;

  // This method is used by MultibodyTree within a tip-to-base loop to compute
  // the composite body inertia of each body in the system, taken about its
  // own body frame origin, and expressed in the World frame.
  //
  // @param[in] pc Position kinematics cache.
  // @param[in] M_BBo_W_all Spatial inertias for all bodies B, about Bo,
  //   and expressed in World (depends on configuration).
  // @param[in] K_BBo_W_all Vector storing the composite body inertias, taken
  //   about Bo, and expressed in World, for all bodies in the multibody system.
  //   It must contain already up-to-date composite body inertias for all the
  //   children of `this` node.
  //
  // @pre CalcCompositeBodyInertiaWorld_TipToBase() must have already been
  // called for the children nodes (and, by recursive precondition, all outboard
  // nodes in the tree.)
  virtual void CalcCompositeBodyInertiaInWorld_TipToBase(
      const PositionKinematicsCache<T>& pc,
      const std::vector<SpatialInertia<T>>& M_BBo_W_all,
      std::vector<SpatialInertia<T>>* K_BBo_W_all) const;

  // Forms LLT factorization of articulated rigid body's hinge inertia matrix.
  // @param[in] D_B Articulated rigid body hinge matrix.
  // @param[out] llt_D_B Stores the LLT factorization of D_B.
  // @throws an exception if D_B is not positive definite or is near-singular.
  // @pre llt_D_B is not nullptr.
  // TODO(sherm1) This should be in BodyNodeImpl (templatized for particular
  //  mobilizers) but is required by the existing body_node_test.cc.
  void CalcArticulatedBodyHingeInertiaMatrixFactorization(
      const MatrixUpTo6<T>& D_B,
      math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>* llt_D_B) const;

  // Returns the inboard frame F of this node's mobilizer.
  const Frame<T>& inboard_frame() const {
    return get_mobilizer().inboard_frame();
  }

  // Returns the outboard frame M of this node's mobilizer.
  const Frame<T>& outboard_frame() const {
    return get_mobilizer().outboard_frame();
  }

 protected:
  // Returns the index to the parent RigidBody of the RigidBody associated with
  // this node. For the root node, corresponding to the world RigidBody, this
  // method returns an invalid BodyIndex. Attempts to using invalid indexes
  // leads to an exception being thrown in Debug builds.
  BodyIndex get_parent_body_index() const {
    if (mobod().is_world()) return BodyIndex{};
    return get_mobilizer().inboard_frame().body().index();
  }

  // =========================================================================
  // Helpers to access the state.
  // Returns an Eigen expression of the vector of generalized velocities.
  Eigen::VectorBlock<const VectorX<T>> get_mobilizer_velocities(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(this->has_parent_tree());
    const MultibodyTree<T>& tree = this->get_parent_tree();
    return tree.get_state_segment(
        context, tree.num_positions() + mobod().v_start(), mobod().nv());
  }

  // Helper to get an Eigen expression of the vector of generalized velocities
  // from a vector of generalized velocities for the entire parent multibody
  // tree. Useful for the implementation of operator forms where the generalized
  // velocity (or time derivatives of the generalized velocities) is an argument
  // to the operator.
  Eigen::VectorBlock<const VectorX<T>> get_mobilizer_velocities(
      const VectorX<T>& v) const {
    return v.segment(mobod().v_start(), mobod().nv());
  }

 private:
  friend class BodyNodeTester;

  // Implementation for MultibodyElement::DoSetTopology().
  void DoSetTopology() final {
    // BodyNode gets everything it needs at construction.
    DRAKE_DEMAND(body_ != nullptr && mobilizer_ != nullptr);
  }

  const BodyNode<T>* parent_node_{nullptr};
  std::vector<const BodyNode<T>*> children_;

  // Pointers for fast access.
  const RigidBody<T>* body_{nullptr};
  const Mobilizer<T>* mobilizer_{nullptr};
};

// During CalcMassMatrix() (using the Composite Body Algorithm via recursion of
// World-frame quantities), this dispatcher is invoked by the composite body B,
// whose inboard mobilizer has Bnv dofs, on each of the bodies ("parent nodes")
// on its inboard path to World.
template <typename T, int Bnv>
class CalcMassMatrixOffDiagonalViaWorldDispatcher;

#define SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(Bnv)                   \
  template <typename T>                                                    \
  class CalcMassMatrixOffDiagonalViaWorldDispatcher<T, Bnv> {              \
   public:                                                                 \
    static void Dispatch(const BodyNode<T>& parent_node, int B_start_in_v, \
                         const std::vector<Vector6<T>>& H_PB_W_cache,      \
                         const Eigen::Matrix<T, 6, Bnv>& Fm_CPo_W,         \
                         EigenPtr<MatrixX<T>> M) {                         \
      parent_node.CalcMassMatrixOffDiagonalBlockViaWorld##Bnv(             \
          B_start_in_v, H_PB_W_cache, Fm_CPo_W, M);                        \
    }                                                                      \
  }

SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(1);
SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(2);
SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(3);
SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(4);
SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(5);
SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER(6);

#undef SPECIALIZE_MASS_MATRIX_VIA_WORLD_DISPATCHER

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::BodyNode);
