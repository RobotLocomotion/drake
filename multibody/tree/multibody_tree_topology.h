#pragma once

/// @file
/// This file defines the topological structures which represent the logical
/// connectivities between multibody tree elements. For instance, the
/// RigidBodyTopology for a RigidBody will contain the topological information
/// specifying its inboard (or parent) RigidBody in the forest model, and its
/// outboard (or children) rigid bodies, and the level or depth in the forest.
/// All of this information is independent of the particular scalar type T the
/// MultibodyTree and its components are specialized with. All of the data
/// structures defined in this file are meant to be the minimal
/// representation that can store this information. These data structures are
/// used in the following ways:
///
/// - To aid the process of cloning or transmogrifying multibody tree
///   components without having to create maps between the "original" and
///   "cloned" objects.
/// - Each topological Multibody element has a copy (acquired at
///   MultibodyTree::Finalize() stage) of its topology which serves as a
///   key into the Context for that element's state.

#include <algorithm>
#include <optional>
#include <set>
#include <stack>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/ssize.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Store the topological information associated with a RigidBody.
struct RigidBodyTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidBodyTopology);

  // Default construction to invalid configuration.
  RigidBodyTopology() {}

  // Constructs for RigidBody with index `body_index` and corresponding
  // RigidBodyFrame with index `frame_index`.
  RigidBodyTopology(BodyIndex body_index, FrameIndex frame_index)
      : index(body_index), body_frame(frame_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const RigidBodyTopology& other) const;

  // Unique index in the MultibodyPlant.
  BodyIndex index{0};

  // Index to the one and only inboard mobilizer a RigidBody can have.
  MobodIndex inboard_mobilizer{};

  // Within the SpanningForest, the RigidBody immediately inboard of this body;
  // That is, the body at the other side of the inboard_mobilizer at a level
  // one lower (closer to World) in the Forest. We're calling this "parent"
  // here (in the tree sense) but don't confuse it with the parent RigidBody
  // of a Joint. This will remain "invalid" for World.
  BodyIndex parent_body{};

  // Within the SpanningForest, the immediate outboard (or "child") RigidBodies
  // to this body. Bodies appear in child_bodies in the order mobilizers were
  // added to the model, with MultibodyTreeTopology::add_mobilizer().
  std::vector<BodyIndex> child_bodies;

  // Unique index to the frame associated with this RigidBody.
  FrameIndex body_frame{0};

  // Depth level in the SpanningForest, level = 0 for World.
  int level{-1};

  // Index to the mobilized body (BodyNode) modeling this RigidBody in the
  // SpanningForest.
  MobodIndex mobod_index;

  // `true` if this topology corresponds to a floating base RigidBody, meaning
  // it has a 6 dof mobilizer connecting it to World.
  bool is_floating_base{false};

  // `true` if this topology corresponds to a floating RigidBody with rotations
  // parametrized by a quaternion.
  bool has_quaternion_dofs{false};

  int floating_positions_start{-1};
  int floating_velocities_start_in_v{-1};
};

// Data structure to store the topological information associated with a Frame.
struct FrameTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameTopology);

  // Default construction to invalid configuration.
  FrameTopology() {}

  // Constructs a frame topology for a frame with index `frame_index`
  // associated with a RigidBody with index `body_index`.
  FrameTopology(FrameIndex frame_index, BodyIndex body_index)
      : index(frame_index), rigid_body(body_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const FrameTopology& other) const = default;

  // Index in the MultibodyPlant.
  FrameIndex index{0};

  // Index of the RigidBody this physical frame attaches to.
  BodyIndex rigid_body{0};
};

// Data structure to store the topological information associated with a
// Mobilizer object. It stores:
//
// - Indexes to the inboard/outboard frames of this mobilizer.
// - Indexes to the inboard/outboard rigid bodies of this mobilizer.
// - Numbers of dofs admitted by this mobilizer.
// - Indexing information to retrieve entries from the parent MultibodyTree
//   Context.
//
// Additional information on topology classes is given in this file's
// documentation at the top.
struct MobilizerTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MobilizerTopology);

  // Constructs a MobilizerTopology from the corresponding Mobod, plus the
  // connected RigidBodies and the Frame geometric information from the
  // modeled Joint.
  MobilizerTopology(MobodIndex mobilizer_index, FrameIndex in_frame,
                    FrameIndex out_frame, BodyIndex in_body, BodyIndex out_body,
                    const SpanningForest::Mobod& mobod)
      : index(mobilizer_index),
        inboard_frame(in_frame),
        outboard_frame(out_frame),
        inboard_body(in_body),
        outboard_body(out_body),
        num_positions(mobod.nq()),
        positions_start(mobod.q_start()),
        num_velocities(mobod.nv()),
        velocities_start_in_v(mobod.v_start()) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MobilizerTopology& other) const = default;

  // Returns `true` if this Mobilizer connects these Frames.
  bool connects_frames(FrameIndex frame1, FrameIndex frame2) const {
    return (inboard_frame == frame1 && outboard_frame == frame2) ||
           (inboard_frame == frame2 && outboard_frame == frame1);
  }

  // Returns `true` if this Mobilizer connects this RigidBody pair.
  bool connects_rigid_bodies(BodyIndex body1, BodyIndex body2) const {
    return (inboard_body == body1 && outboard_body == body2) ||
           (inboard_body == body2 && outboard_body == body1);
  }

  // Returns `true` if this Mobilizer is a weld.
  bool is_weld_mobilizer() const { return num_velocities == 0; }

  // Unique index in the set of Mobilizers. There will be a corresponding
  // BodyNode (and hence BodyNodeTopology) with the same index.
  MobodIndex index;

  FrameIndex inboard_frame;
  FrameIndex outboard_frame;
  BodyIndex inboard_body;
  BodyIndex outboard_body;

  // Number of generalized coordinates granted by this mobilizer.
  int num_positions{0};
  // First entry in the q partition of the global array of states x = [q v z].
  int positions_start{0};

  // Number of generalized velocities granted by this mobilizer.
  int num_velocities{0};
  // Start index in a vector indexed like the v partition of states x, including
  // generalized accelerations (which are the time derivatives of the
  // generalized velocities) and generalized forces.
  int velocities_start_in_v{0};
};

// Data structure to store the topological information associated with a
// JointActuator.
struct JointActuatorTopology {
  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const JointActuatorTopology& other) const = default;

  // Unique index in the MultibodyTree.
  JointActuatorIndex index{0};
  // For an actuator in a MultibodyTree model, this index corresponds to the
  // first entry in the global array u containing all actuation values for the
  // entire model. Actuator indexes are assigned in the order actuators are
  // added to the model, that is, in the order of JointActuatorIndex.
  int actuator_index_start{-1};
  // The number of dofs actuated by this actuator.
  int num_dofs{-1};
};

// Data structure to store the topological information associated with a tree
// node. A tree node consists of a body and its inboard mobilizer. Note that
// BodyNodes and Mobilizers are indexed identically.
// A body node is in charge of the computations associated to that body and
// mobilizer, especially within a base-to-tip or tip-to-base recursion.
// As the topological entity associated with a tree node, this struct contains
// information regarding parent and child nodes, corresponding parent and child
// Links, etc.
struct BodyNodeTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyNodeTopology);

  // Default construction to invalid configuration.
  BodyNodeTopology() {}

  // Constructor specifying the topological information for a tree node.
  // A tree node is instantiated for each body in the multibody system and it
  // contains, in addition to that particular body, the inboard mobilizer
  // connecting the body to the rest of the tree inwards (i.e. towards the
  // world or root of the tree) from the mobilizer.
  //
  // @param index_in The unique index for `this` body node.
  // @param level_in The level (depth or generation) in the tree structure.
  // @param parent_node_in The parent node, in a tree structure sense, of
  //     `this` node.
  // @param rigid_body_in The index to the body associated with this node.
  // @param parent_rigid_body_in The parent body, in a tree structure sense, to
  //     `rigid_body_in`. In other words, `parent_rigid_body_in` is the body
  //     associated with node `parent_node_in`.
  BodyNodeTopology(MobodIndex index_in, int level_in, MobodIndex parent_node_in,
                   BodyIndex rigid_body_in, BodyIndex parent_rigid_body_in)
      : index(index_in),
        level(level_in),
        parent_body_node(parent_node_in),
        rigid_body(rigid_body_in),
        parent_rigid_body(parent_rigid_body_in) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const BodyNodeTopology& other) const;

  // Returns the number of children to this node.
  int get_num_children() const { return ssize(child_nodes); }

  // Index of this node in the SpanningForest (0 for World). There
  // is an associated Mobilizer with the same index.
  MobodIndex index{};

  // Depth level in the SpanningForest, level = 0 for World.
  int level{-1};

  // The index to the inboard ("parent") BodyNode of this node.
  MobodIndex parent_body_node;

  BodyIndex rigid_body;         // This node's RigidBody B.
  BodyIndex parent_rigid_body;  // This node's parent RigidBody P.

  // The list of outboard ("child") body nodes to this node.
  std::vector<MobodIndex> child_nodes;

  // Start and number of dofs for this node's mobilizer.
  int num_mobilizer_positions{0};
  int mobilizer_positions_start{0};
  int num_mobilizer_velocities{0};

  // Start index in a vector containing only generalized velocities.
  // It is also a valid index into a vector of generalized accelerations (which
  // are the time derivatives of the generalized velocities).
  int mobilizer_velocities_start_in_v{0};
};

// Data structure to store the topological information associated with an
// entire SpanningForest of a MultibodyPlant.
class MultibodyTreeTopology {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  // Default constructor creates an empty, invalid topology. The minimum valid
  // topology for a MultibodyPlant contains at least the
  // RigidBodyTopology for World.
  MultibodyTreeTopology() {}

  ~MultibodyTreeTopology();

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MultibodyTreeTopology& other) const;

  // Returns the number of RigidBody elements in the MultibodyPlant. This
  // includes the World RigidBody and therefore the minimum number of rigid
  // bodies after MultibodyTree::Finalize() will always be one, not zero.
  int num_rigid_bodies() const { return ssize(rigid_bodies_); }

  // Returns the number of physical frames in the multibody tree.
  int num_frames() const { return ssize(frames_); }

  // Returns the number of Mobilizers. This is always the same as the number
  // of BodyNodes and the number of mobilized bodies in the SpanningForest.
  int num_mobilizers() const { return ssize(mobilizers_); }

  // Returns the number of BodyNodes. These are generated 1:1 from the
  // MobilizedBodies in the SpanningForest. If we combined welded Links,
  // there will be more RigidBodies than BodyNodes.
  int num_mobods() const { return ssize(body_nodes_); }

  // Returns the number of joint actuators in the topology.
  int num_joint_actuators() const { return ssize(joint_actuators_); }

  // Returns the number of levels in the forest topology.
  int forest_height() const { return forest_height_; }

  // Returns a constant reference to the corresponding FrameTopology given the
  // FrameIndex.
  const FrameTopology& get_frame(FrameIndex index) const {
    DRAKE_ASSERT(index < num_frames());
    return frames_[index];
  }

  // Returns a constant reference to the corresponding RigidBodyTopology given a
  // BodyIndex.
  const RigidBodyTopology& get_rigid_body(BodyIndex index) const {
    DRAKE_ASSERT(index < num_rigid_bodies());
    return rigid_bodies_[index];
  }

  // Mutable version of get_rigid_body().
  RigidBodyTopology& get_mutable_rigid_body(BodyIndex index) {
    DRAKE_ASSERT(index < num_rigid_bodies());
    return rigid_bodies_[index];
  }

  // Returns a constant reference to the corresponding MobilizerTopology given a
  // MobodIndex.
  const MobilizerTopology& get_mobilizer(MobodIndex index) const {
    DRAKE_ASSERT(index < num_mobilizers());
    return mobilizers_[index];
  }

  // Returns a constant reference to the corresponding JointActuatorTopology
  // given a JointActuatorIndex.
  const JointActuatorTopology& get_joint_actuator(
      JointActuatorIndex index) const;

  // Returns a constant reference to the corresponding BodyNodeTopology given
  // a MobodIndex.
  const BodyNodeTopology& get_body_node(MobodIndex index) const {
    DRAKE_ASSERT(index < num_mobods());
    return body_nodes_[index];
  }

  // Returns the number of trees in the "forest" topology of the entire system.
  // We refer to as "tree" a subgraph in the topology having a tree structure
  // and whose base node connects to the world. The world does not belong to any
  // tree. In other words, the number of trees in the topology corresponds to
  // the number of children of the world body node (also called "base nodes").
  int num_trees() const { return ssize(num_tree_velocities_); }

  // Returns the number of generalized velocities for the t-th tree.
  // @pre t.is_valid() is true and t < num_trees().
  int num_tree_velocities(TreeIndex t) const {
    DRAKE_ASSERT(t < num_trees());
    return num_tree_velocities_[t];
  }

  // For the t-th tree, this method returns the index of the first generalized
  // velocity in the vector of generalized velocities for the entire model.
  // Starting at this index, the num_tree_velocities(t) velocities for the t-th
  // tree are contiguous in the vector of generalized velocities for the full
  // model. tree_velocities_start(t) always returns a valid index to an entry in
  // the vector of generalized velocities for the full model, even if the t-th
  // tree has no generalized velocities. In such case however,
  // num_tree_velocities(t) will be zero.
  int tree_velocities_start_in_v(TreeIndex t) const {
    DRAKE_ASSERT(t < num_trees());
    return tree_velocities_start_in_v_[t];
  }

  // Returns the tree index for the b-th rigid body. The tree index for the
  // world rigid body, BodyIndex(0), is invalid. Check with
  // TreeIndex::is_valid().
  // @pre Index b is valid and b < num_rigid_bodies().
  TreeIndex body_to_tree_index(BodyIndex b) const {
    DRAKE_ASSERT(b < num_rigid_bodies());
    return rigid_body_to_tree_index_[b];
  }

  // Returns the tree index for the v-th velocity.
  // @pre 0 <= v and v < num_velocities().
  TreeIndex velocity_to_tree_index(int v) const {
    DRAKE_ASSERT(0 <= v && v < num_velocities());
    return velocity_to_tree_index_[v];
  }

  // Given a tree index, returns `true` if that tree has any degrees of freedom
  // (ignoring joint locking). An invalid tree index is treated as World's
  // "tree", which has no dofs.
  bool tree_has_dofs(TreeIndex tree_index) const {
    if (!tree_index.is_valid()) return false;  // World doesn't have a Tree.
    return num_tree_velocities(tree_index) > 0;
  }

  // Creates and adds a new RigidBodyTopology to this MultibodyTreeTopology. The
  // RigidBodyTopology will be assigned a new, unique BodyIndex and FrameIndex
  // values.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns a std::pair<BodyIndex, FrameIndex> containing the indexes
  // assigned to the new RigidBodyTopology.
  std::pair<BodyIndex, FrameIndex> add_rigid_body();

  // Creates and adds a new FrameTopology, associated with the given
  // body_index, to this MultibodyTreeTopology.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The FrameIndex assigned to the new FrameTopology.
  FrameIndex add_frame(BodyIndex body_index);

  // Creates and adds a new MobilizerTopology connecting the inboard and
  // outboard multibody frames identified by indexes `in_frame` and
  // `out_frame`, respectively. The created topology will correspond to that of
  // a Mobilizer with `num_positions` and `num_velocities`.
  //
  // @throws std::exception if either `in_frame` or `out_frame` do not
  // index frame topologies in `this` %MultibodyTreeTopology.
  // @throws std::exception if `in_frame == out_frame`.
  // @throws std::exception if `in_frame` and `out_frame` already are
  // connected by another mobilizer. More than one mobilizer between two frames
  // is not allowed.
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The MobodIndex assigned to the new MobilizerTopology.
  MobodIndex add_mobilizer(const SpanningForest::Mobod& mobod,
                           FrameIndex in_frame, FrameIndex out_frame);

  void add_world_mobilizer(const SpanningForest::Mobod& world_mobod,
                           FrameIndex world_body_frame);

  // Creates and adds a new JointActuatorTopology for a joint with `num_dofs`
  // degrees of freedom.
  // @param[in] num_dofs
  //   The number of joint dofs actuated by this actuator.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The JointActuatorIndex assigned to the new JointActuatorTopology.
  JointActuatorIndex add_joint_actuator(int num_dofs);

  // Removes `actuator_index` from the list of joint actuators. The
  // `actuator_index_start` will be modified if necessary for other actuators.
  // @throws std::exception if called post-Finalize.
  // @throws std::exception if the actuator with the index `actuator_index` has
  // already been removed.
  void RemoveJointActuator(JointActuatorIndex actuator_index);

  // This method must be called by MultibodyTree::Finalize() after all
  // topological elements in the plant (rigid bodies, joints, constraints) were
  // added and a suitable SpanningForest built.
  //
  // We extract the necessary topological information from the Forest, i.e. how
  // rigid bodies and joints are interconnected, and use that information to
  // build BodyNodeTopology objects directly in depth-first order and extract
  // Tree structure.
  //
  // If the finalize stage is successful, the `this` topology is validated,
  // meaning it is up-to-date after this call. No more multibody elements can be
  // added after a call to Finalize().
  //
  // @throws std::exception If users attempt to call this method on an
  //         already finalized topology.
  // @see is_valid()
  void Finalize(const LinkJointGraph& graph);

  // Returns `true` if Finalize() was already called on `this` topology.
  // @see Finalize()
  bool is_valid() const { return is_valid_; }

  // Returns the total number of generalized positions in the model.
  int num_positions() const { return num_positions_; }

  // Returns the total number of generalized velocities in the model.
  int num_velocities() const { return num_velocities_; }

  // Returns the total size of the state vector in the model.
  int num_states() const { return num_states_; }

  // Returns the total number of actuated joint dofs in the model.
  int num_actuated_dofs() const { return num_actuated_dofs_; }

 private:
  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the frames with indexes `frame` and `frame2`.
  bool IsThereAMobilizerBetweenFrames(FrameIndex frame1,
                                      FrameIndex frame2) const;

  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the Links with indexes `body1` and `body2`.
  bool IsThereAMobilizerBetweenRigidBodies(BodyIndex body1,
                                           BodyIndex body2) const;

  // Helper method to be used within Finalize() to obtain the topological
  // information that describes the multibody system as a "forest" of trees.
  void ExtractForestInfo(const LinkJointGraph& graph);

  // is_valid is set to `true` after a successful Finalize().
  bool is_valid_{false};
  // Number of levels in the full Forest topology. After Finalize()
  // there will be at least one level (level = 0) with the world body.
  int forest_height_{-1};

  // Topological elements:
  std::vector<FrameTopology> frames_;
  std::vector<RigidBodyTopology> rigid_bodies_;
  std::vector<MobilizerTopology> mobilizers_;
  std::vector<std::optional<JointActuatorTopology>> joint_actuators_;
  std::vector<BodyNodeTopology> body_nodes_;

  // Total number of generalized positions and velocities in the MultibodyTree
  // model.
  int num_positions_{0};
  int num_velocities_{0};
  int num_states_{0};
  int num_actuated_dofs_{0};

  // Number of generalized velocities for the t-th tree.
  std::vector<int> num_tree_velocities_;
  // Given the generalized velocities vector v for the entire model, the vector
  // vt = {v(m) s.t. m ∈ [mₛ, mₑ)}, with mₛ = tree_velocities_start_in_v_[t] and
  // iₑ = tree_velocities_start_in_v_[t] + num_tree_velocities_[t], are the
  // generalized velocities for the t-th tree.
  std::vector<int> tree_velocities_start_in_v_;
  // t = velocity_to_tree_index_[m] is the tree index to which the m-th velocity
  // (within the v vector) belongs.
  std::vector<TreeIndex> velocity_to_tree_index_;
  // t = rigid_body_to_tree_index_[b] is the index of the tree to which the b-th
  // RigidBody belongs.
  std::vector<TreeIndex> rigid_body_to_tree_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
