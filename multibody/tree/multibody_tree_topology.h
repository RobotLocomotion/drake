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

  // Unique index in the MultibodyTree.
  BodyIndex index{0};

  // Unique index to the one and only inboard mobilizer a body can have.
  // By default this is initialized to "invalid" so that we can detect
  // graph loops within add_mobilizer().
  // This will remain "invalid" for the world body.
  MobilizerIndex inboard_mobilizer{};

  // Within the tree structure of a MultibodyTree, the immediate inboard (or
  // "parent") body connected by the Mobilizer indexed by `inboard_mobilizer`.
  // By default this is initialized to "invalid" so that we can assert
  // (from within add_mobilizer()) that each body can have only one parent
  // body. Also, this will remain "invalid" for the world body.
  BodyIndex parent_body{};

  // Within the tree structure of a MultibodyTree, the immediate outboard (or
  // "child") bodies to this Body. Bodies appear in child_bodies in the order
  // mobilizers were added to the model, with
  // MultibodyTreeTopology::add_mobilizer().
  std::vector<BodyIndex> child_bodies;

  // Unique index to the frame associated with this RigidBody.
  FrameIndex body_frame{0};

  // Depth level in the MultibodyTree, level = 0 for the world.
  // Initialized to an invalid negative value so that we can detect at
  // Finalize() when a user forgets to connect a body with a mobilizer.
  int level{-1};

  // Index to the mobilized body (BodyNode) modeling this RigidBody.
  MobodIndex mobod_index;

  // `true` if this topology corresponds to a floating RigidBody.
  bool is_floating{false};

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
  bool operator==(const FrameTopology& other) const;

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

  // Default construction to invalid configuration.
  MobilizerTopology() {}

  // Constructs a %MobilizerTopology by specifying the index `mobilizer_index`
  // for `this` new topology, the indexes to the inboard and outboard frames
  // the Mobilizer will connect, given by `in_frame` and `out_frame`
  // respectively, and similarly the inboard and outboard bodies being
  // connected, given by `in_body` and `out_body`, respectively.  The
  // constructed topology will correspond to that of a Mobilizer with
  // `num_positions_in` generalized positions and `num_velocities_in`
  // generalized velocities.
  MobilizerTopology(MobilizerIndex mobilizer_index, FrameIndex in_frame,
                    FrameIndex out_frame, BodyIndex in_body, BodyIndex out_body,
                    int num_positions_in, int num_velocities_in)
      : index(mobilizer_index),
        inboard_frame(in_frame),
        outboard_frame(out_frame),
        inboard_body(in_body),
        outboard_body(out_body),
        num_positions(num_positions_in),
        num_velocities(num_velocities_in) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MobilizerTopology& other) const;

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

  // Unique index in the set of mobilizers.
  MobilizerIndex index;

  FrameIndex inboard_frame;
  FrameIndex outboard_frame;
  BodyIndex inboard_body;
  BodyIndex outboard_body;

  // Index to the mobilized body (BodyNode) modeling this Mobilizer.
  MobodIndex mobod_index;

  // Mobilizer indexing info: Set at Finalize() time.
  // Number of generalized coordinates granted by this mobilizer.
  int num_positions{0};
  // First entry in the q partition of the global array of states x = [q v z].
  int positions_start{0};

  // Number of generalized velocities granted by this mobilizer.
  int num_velocities{0};
  // First entry in the global array of states, `x = [q v z]`, for the parent
  // MultibodyTree.
  int velocities_start_in_state{0};

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
  bool operator==(const JointActuatorTopology& other) const;

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
// node. A tree node essentially consists of a body and its inboard mobilizer.
// A body node is in charge of the computations associated to that body and
// mobilizer, especially within a base-to-tip or tip-to-base recursion.
// As the topological entity associated with a tree node (and specifically a
// MultibodyTree node), this struct contains information regarding parent and
// child nodes, parent and child bodies, etc.
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
                   BodyIndex rigid_body_in, BodyIndex parent_rigid_body_in,
                   MobilizerIndex mobilizer_in)
      : index(index_in),
        level(level_in),
        parent_body_node(parent_node_in),
        rigid_body(rigid_body_in),
        parent_rigid_body(parent_rigid_body_in),
        mobilizer(mobilizer_in) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const BodyNodeTopology& other) const;

  // Unique index of this node in the MultibodyTree.
  MobodIndex index{};

  // Depth level in the MultibodyTree, level = 0 for the world.
  int level{-1};

  // The index to the inboard ("parent") BodyNode of this node.
  MobodIndex parent_body_node;

  BodyIndex rigid_body;         // This node's RigidBody B.
  BodyIndex parent_rigid_body;  // This node's parent RigidBody P.

  MobilizerIndex mobilizer;  // The mobilizer connecting bodies P and B.

  // The list of outboard ("child") body nodes to this node.
  std::vector<MobodIndex> child_nodes;

  // Returns the number of children to this node.
  int get_num_children() const { return ssize(child_nodes); }

  // Start and number of dofs for this node's mobilizer.
  int num_mobilizer_positions{0};
  int mobilizer_positions_start{0};
  int num_mobilizer_velocities{0};
  int mobilizer_velocities_start_in_state{0};

  // Start index in a vector containing only generalized velocities.
  // It is also a valid index into a vector of generalized accelerations (which
  // are the time derivatives of the generalized velocities).
  int mobilizer_velocities_start_in_v{0};
};

// Data structure to store the topological information associated with an
// entire MultibodyTree.
class MultibodyTreeTopology {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  // Default constructor creates an empty, invalid topology. The minimum valid
  // topology for a minimum valid MultibodyTree contains at least the
  // RigidBodyTopology for the world. The topology for the _world_ body does not
  // get added until MultibodyTree construction, which creates a _world_ body
  // and adds it to the tree.
  MultibodyTreeTopology() {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MultibodyTreeTopology& other) const;

  // Returns the number of RigidBody elements in the MultibodyPlant. This
  // includes the World RigidBody and therefore the minimum number of rigid
  // bodies after MultibodyTree::Finalize() will always be one, not zero.
  int num_rigid_bodies() const { return ssize(rigid_bodies_); }

  // Returns the number of physical frames in the multibody tree.
  int num_frames() const { return ssize(frames_); }

  // Returns the number of mobilizers in the multibody tree. Since the "world"
  // body does not have a mobilizer, the number of mobilizers will always equal
  // the number of mobilized bodies minus one.
  int num_mobilizers() const { return ssize(mobilizers_); }

  // Returns the number of mobilized bodies (BodyNodes). Currently this is
  // restricted to being equal to the number of user-supplied RigidBody objects.
  // TODO(sherm1) Relax this restriction -- the number of mobilized bodies can
  //  differ from the number of user-provided links.
  int num_mobods() const { return ssize(body_nodes_); }

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
  // MobilizerIndex.
  const MobilizerTopology& get_mobilizer(MobilizerIndex index) const {
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
  // @returns The MobilizerIndex assigned to the new MobilizerTopology.
  MobilizerIndex add_mobilizer(FrameIndex in_frame, FrameIndex out_frame,
                               int num_positions, int num_velocities);

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
  // added and before any computations are performed.
  // It essentially compiles all the necessary "topological information", i.e.
  // how bodies, joints and, any other elements connect with each other, and
  // performs all the required pre-processing to perform computations at a
  // later stage. This preprocessing includes:
  //
  // - sorting in DFT order for fast recursions through the tree,
  // - computation of state sizes and of pool sizes within cache entries,
  // - computation of index maps to retrieve either state or cache entries for
  //   each multibody element.
  //
  // If the finalize stage is successful, the `this` topology is validated,
  // meaning it is up-to-date after this call.
  // No more multibody tree elements can be added after a call to Finalize().
  //
  // @throws std::exception If users attempt to call this method on an
  //         already finalized topology.
  // @see is_valid()
  void Finalize();

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

  // Given a node in `this` topology, specified by its MobodIndex `from`,
  // this method computes the kinematic path formed by all the nodes in the
  // tree that connect `from` with the root (corresponding to the world).
  //
  // @param[in] from
  //   A node in the tree topology to which the path to the root (world) is to
  //   be computed.
  // @param[out] path_to_world
  //   A std::vector of body node indexes that on output will contain the path
  //   to the root of the tree. Forward iteration (from element 0 to element
  //   size()-1) of `path_to_world` will traverse all nodes in the tree
  //   starting at the root along the path to `from`. That is, forward
  //   iteration starts with the root of the tree at `path_to_world[0]` and
  //   ends with `from` at `path_to_world.back()`.
  //   On input, `path_to_world` must be a valid pointer. On output this vector
  //   will be resized, only if needed, to store as many elements as the level
  //   (BodyNodeTopology::level) of body node `from` plus one (so that we can
  //   include the root node in the path).
  void GetKinematicPathToWorld(MobodIndex from,
                               std::vector<MobodIndex>* path_to_world) const;

  // Returns `true` if the body with index `body_index` is anchored to the
  // world.
  // A body is said to be "anchored" if its kinematics path to the world only
  // contains weld mobilizers.
  // The complexity of this operation is O(depth), where "depth" refers to the
  // depth in the tree of the body node associated with `body_index`.
  bool IsBodyAnchored(BodyIndex body_index) const;

  // This method partitions the tree topology into sub-graphs such that two
  // bodies are in the same sub-graph if there is a path between them which
  // includes only welded-mobilizer.
  // Each sub-graph of welded bodies is represented as a set of body indices.
  // By definition, these sub-graphs will be disconnected by any non-weld
  // mobilizers that may be inboard or outboard of any given body. The first
  // sub-graph will have all of the bodies welded to the world; all
  // subsequent sub-graphs will be in no particular order.
  // A few more notes:
  //
  // - Each body in the topology is included in one set and one set only.
  // - The maximum size of the list equals the number of bodies in the topology
  //   (num_rigid_bodies()). That would be a topology with no weld mobilizers.
  // - The world body is also included in a welded-bodies set, and this set is
  //   element zero in the returned vector.
  // - The minimum size of the list is one. This corresponds to a topology with
  //   all bodies welded to the world.
  std::vector<std::set<BodyIndex>> CreateListOfWeldedBodies() const;

  // Computes the number of generalized velocities in the tree composed of the
  // nodes outboard of `base`, excluding the generalized velocities of `base`.
  // Note: This method returns 0 if base is the most distal body in a multibody
  // tree or if base's children are all welded to it and they are the most
  // distal bodies in the tree.
  // @pre Body nodes were already created.
  int CalcNumberOfOutboardVelocitiesExcludingBase(
      const BodyNodeTopology& base) const {
    return CalcNumberOfOutboardVelocities(base) - base.num_mobilizer_velocities;
  }

  // Returns all bodies that are transitively outboard of the given bodies. In
  // other words, returns the union of all bodies in the subtrees with the given
  // bodies as roots. The result is sorted in increasing body index order.
  // @pre Finalize() is called.
  // @pre body_index is valid and is less than the number of bodies.
  std::vector<BodyIndex> GetTransitiveOutboardBodies(
      std::vector<BodyIndex> body_indexes) const;

 private:
  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the frames with indexes `frame` and `frame2`.
  bool IsThereAMobilizerBetweenFrames(FrameIndex frame1,
                                      FrameIndex frame2) const;

  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the bodies with indexes `body2` and `body2`.
  bool IsThereAMobilizerBetweenRigidBodies(BodyIndex body1,
                                           BodyIndex body2) const;

  // Recursive helper method for CreateListOfWeldedBodies().
  // This method scans the children of body with parent_index. If a child is
  // welded to body with parent_index, it gets added to the parent's body welded
  // body, parent_welded_body. Otherwise a new welded body is created for the
  // child body and gets added to the list of all welded bodies, welded_bodies.
  void CreateListOfWeldedBodiesRecurse(
      BodyIndex parent_index, std::set<BodyIndex>* parent_welded_body,
      std::vector<std::set<BodyIndex>>* welded_bodies) const;

  // This traverses the tree of nodes outboard of `base` and applies `operation`
  // on each of them, starting with `base`. The traversal is performed in depth
  // first order.
  // @pre Body nodes were already created and therefore they are indexed in
  // depth first order.
  void TraverseOutboardNodes(
      const BodyNodeTopology& base,
      std::function<void(const BodyNodeTopology&)> operation) const;

  // Computes the number of generalized velocities in the tree composed of the
  // nodes outboard of `base`, including the generalized velocities of `base`.
  // @pre Body nodes were already created.
  int CalcNumberOfOutboardVelocities(const BodyNodeTopology& base) const;

  // Helper method to be used within Finalize() to obtain the topological
  // information that describes the multibody system as a "forest" of trees.
  void ExtractForestInfo();

  // is_valid is set to `true` after a successful Finalize().
  bool is_valid_{false};
  // Number of levels (or generations) in the forest topology. After Finalize()
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
