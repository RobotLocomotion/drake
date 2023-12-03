#pragma once

/// @file
/// This file defines the topological structures which represent the logical
/// connectivities between multibody tree elements. For instance, the
/// BodyTopology for a Body will contain the topological information  specifying
/// its inboard (or parent) body in the parent tree, and its outboard (or
/// children) bodies, and the level or depth in the MultibodyTree.
/// All of this information is independent of the particular scalar type T the
/// MultibodyTree and its components are specialized with.
/// All of the data structures defined in this file are meant to be the most
/// minimalist representation that can store this information.
/// These data structures are used in the following ways:
///
/// - To aid the process of cloning or transmogrifying multibody tree
///   components without having to create maps between the "original" and
///   "cloned" objects. That process is tedious and error prone.
/// - Each Multibody tree element has a copy (acquired at
///   MultibodyTree::Finalize() stage) of its topology which serves as a
///   key into the Context for that element's state.
/// - The topology is also stored in the Context so that the Multibody tree's
///   topology can be validated against the stored topology in debug builds.

#include <algorithm>
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

// Data structure to store the topological information associated with a Body.
struct BodyTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyTopology);

  // Default construction to invalid configuration.
  BodyTopology() {}

  // Constructs a body topology struct with index `body_index` and a body frame
  // with index `frame_index`.
  BodyTopology(BodyIndex body_index, FrameIndex frame_index) :
      index(body_index), body_frame(frame_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const BodyTopology& other) const {
    if (index != other.index) return false;
    if (inboard_mobilizer.is_valid() != other.inboard_mobilizer.is_valid())
      return false;
    if (inboard_mobilizer.is_valid() &&
        inboard_mobilizer != other.inboard_mobilizer)
      return false;
    if (parent_body.is_valid() != other.parent_body.is_valid()) return false;
    if (parent_body.is_valid() && parent_body != other.parent_body)
      return false;
    if (child_bodies != other.child_bodies) return false;
    if (body_frame != other.body_frame) return false;
    if (level != other.level) return false;
    if (mobod_index != other.mobod_index) return false;
    if (is_floating != other.is_floating) return false;
    if (has_quaternion_dofs != other.has_quaternion_dofs) return false;
    if (floating_positions_start != other.floating_positions_start)
      return false;
    if (floating_velocities_start_in_v != other.floating_velocities_start_in_v)
      return false;
    return true;
  }

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

  // Unique index to the frame associated with this body.
  FrameIndex body_frame{0};

  // Depth level in the MultibodyTree, level = 0 for the world.
  // Initialized to an invalid negative value so that we can detect at
  // Finalize() when a user forgets to connect a body with a mobilizer.
  int level{-1};

  // Index to the mobilized body (BodyNode) modeling this Body.
  MobodIndex mobod_index;

  // `true` if this topology corresponds to a floating body in space.
  bool is_floating{false};

  // `true` if this topology corresponds to a floating body with rotations
  // parametrized by a quaternion.
  bool has_quaternion_dofs{false};

  int floating_positions_start{-1};
  int floating_velocities_start_in_v{-1};
};

// Data structure to store the topological information associated with a
// Frame.
struct FrameTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameTopology);

  // Default construction to invalid configuration.
  FrameTopology() {}

  // Constructs a frame topology for a frame with index `frame_index`
  // associated with a body with index `body_index`.
  FrameTopology(FrameIndex frame_index, BodyIndex body_index) :
      index(frame_index), body(body_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const FrameTopology& other) const {
    if (index != other.index) return false;
    if (body != other.body) return false;
    return true;
  }

  // Unique index in the MultibodyTree.
  FrameIndex index{0};

  // Unique index of the body this physical frame attaches to.
  BodyIndex body{0};
};

// Data structure to store the topological information associated with a
// Mobilizer object. It stores:
//
// - Indexes to the inboard/outboard frames of this mobilizer.
// - Indexes to the inboard/outboard bodies of this mobilizer.
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
  MobilizerTopology(
      MobilizerIndex mobilizer_index,
      FrameIndex in_frame, FrameIndex out_frame,
      BodyIndex in_body, BodyIndex out_body,
      int num_positions_in, int num_velocities_in) :
      index(mobilizer_index),
      inboard_frame(in_frame), outboard_frame(out_frame),
      inboard_body(in_body), outboard_body(out_body),
      num_positions(num_positions_in), num_velocities(num_velocities_in) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MobilizerTopology& other) const {
    if (index != other.index) return false;

    if (inboard_frame != other.inboard_frame) return false;
    if (outboard_frame != other.outboard_frame) return false;
    if (inboard_body != other.inboard_body) return false;
    if (outboard_body != other.outboard_body) return false;

    if (mobod_index != other.mobod_index) return false;

    if (num_positions != other.num_positions) return false;
    if (positions_start != other.positions_start) return false;
    if (num_velocities != other.num_velocities) return false;
    if (velocities_start_in_state != other.velocities_start_in_state)
      return false;
    if (velocities_start_in_v != other.velocities_start_in_v) return false;

    return true;
  }

  // Returns `true` if this %MobilizerTopology connects frames identified by
  // indexes `frame1` and `frame2`.
  bool connects_frames(FrameIndex frame1, FrameIndex frame2) const {
    return (inboard_frame == frame1 && outboard_frame == frame2) ||
           (inboard_frame == frame2 && outboard_frame == frame1);
  }

  // Returns `true` if this %MobilizerTopology connects bodies identified by
  // indexes `body1` and `body2`.
  bool connects_bodies(BodyIndex body1, BodyIndex body2) const {
    return (inboard_body == body1 && outboard_body == body2) ||
           (inboard_body == body2 && outboard_body == body1);
  }

  // Returns `true` if this mobilizer topology corresponds to that of a weld
  // mobilizer.
  bool is_weld_mobilizer() const {
    return num_velocities == 0;
  }

  // Unique index in the set of mobilizers.
  MobilizerIndex index;
  // Index to the inboard frame.
  FrameIndex inboard_frame;
  // Index to the outboard frame.
  FrameIndex outboard_frame;
  // Index to the inboard body.
  BodyIndex inboard_body;
  // Index to the outboard body.
  BodyIndex outboard_body;
  // Index to the mobilized body (BodyNode) modeling this Mobilizer.
  MobodIndex mobod_index;

  // Mobilizer indexing info: Set at Finalize() time.
  // Number of generalized coordinates granted by this mobilizer.
  int num_positions{0};
  // First entry in the global array of states, `x = [q v z]`, for the parent
  // MultibodyTree.
  int positions_start{0};
  // Number of generalized velocities granted by this mobilizer.
  int num_velocities{0};
  // First entry in the global array of states, `x = [q v z]`, for the parent
  // MultibodyTree.
  int velocities_start_in_state{0};

  // Start index in a vector containing only generalized velocities.
  // It is also a valid index into a vector of generalized accelerations (which
  // are the time derivatives of the generalized velocities) and into a vector
  // of generalized forces.
  int velocities_start_in_v{0};
};

// Data structure to store the topological information associated with a
// ForceElement.
struct ForceElementTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ForceElementTopology);

  // Default construction to an invalid configuration. This only exists to
  // satisfy demands of working with various container classes.
  ForceElementTopology() {}

  // Constructs a force element topology with index `force_element_index`.
  explicit ForceElementTopology(ForceElementIndex force_element_index) :
      index(force_element_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const ForceElementTopology& other) const {
    if (index != other.index) return false;
    return true;
  }

  // Unique index in the MultibodyTree.
  ForceElementIndex index{0};
};

// Data structure to store the topological information associated with a
// JointActuator.
struct JointActuatorTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointActuatorTopology);

  // Default construction to an invalid configuration. This only exists to
  // satisfy demands of working with various container classes.
  JointActuatorTopology() {}

  // Constructs a joint actuator topology with index `joint_actuator_index`.
  JointActuatorTopology(
      JointActuatorIndex joint_actuator_index,
      int start_index, int ndofs) :
      index(joint_actuator_index),
      actuator_index_start(start_index),
      num_dofs(ndofs) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const JointActuatorTopology& other) const {
    if (index != other.index) return false;
    if (actuator_index_start != other.actuator_index_start) return false;
    if (num_dofs != other.num_dofs) return false;
    return true;
  }

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
  //                       `this` node.
  // @param body_in The index to the body associated with this node.
  // @param parent_body_in The parent body, in a tree structure sense, to
  //                       `body_in`. In other words, `parent_body_in` is the
  //                       body associated with node `parent_node_in`.
  // @param mobilizer_in The index to the mobilizer associated with this node.
  BodyNodeTopology(
      MobodIndex index_in, int level_in,
      MobodIndex parent_node_in,
      BodyIndex body_in, BodyIndex parent_body_in, MobilizerIndex mobilizer_in)
      : index(index_in), level(level_in),
      parent_body_node(parent_node_in),
      body(body_in), parent_body(parent_body_in), mobilizer(mobilizer_in) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const BodyNodeTopology& other) const {
    if (index != other.index) return false;
    if (level != other.level) return false;

    if (parent_body_node.is_valid() !=
        other.parent_body_node.is_valid()) return false;
    if (parent_body_node.is_valid() &&
        parent_body_node != other.parent_body_node) return false;

    if (body != other.body) return false;

    if (parent_body.is_valid() != other.parent_body.is_valid()) return false;
    if (parent_body.is_valid() &&
        parent_body != other.parent_body) return false;

    if (mobilizer.is_valid() != other.mobilizer.is_valid()) return false;
    if (mobilizer.is_valid() && mobilizer != other.mobilizer) return false;

    if (child_nodes != other.child_nodes) return false;

    if (num_mobilizer_positions != other.num_mobilizer_positions)
      return false;
    if (mobilizer_positions_start != other.mobilizer_positions_start)
      return false;
    if (num_mobilizer_velocities != other.num_mobilizer_velocities)
      return false;
    if (mobilizer_velocities_start_in_state !=
        other.mobilizer_velocities_start_in_state)
      return false;
    if (mobilizer_velocities_start_in_v !=
        other.mobilizer_velocities_start_in_v)
      return false;

    return true;
  }

  // Unique index of this node in the MultibodyTree.
  MobodIndex index{};

  // Depth level in the MultibodyTree, level = 0 for the world.
  int level{-1};

  // The unique index to the parent BodyNode of this node.
  MobodIndex parent_body_node;

  BodyIndex body;         // This node's body B.
  BodyIndex parent_body;  // This node's parent body P.

  MobilizerIndex mobilizer;  // The mobilizer connecting bodies P and B.

  // The list of child body nodes to this node.
  std::vector<MobodIndex> child_nodes;

  // Returns the number of children to this node.
  int get_num_children() const { return ssize(child_nodes);}


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
  // BodyTopology for the world. The topology for the _world_ body does not get
  // added until MultibodyTree construction, which creates a _world_ body
  // and adds it to the tree.
  MultibodyTreeTopology() {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MultibodyTreeTopology& other) const {
    if (is_valid_ != other.is_valid_) return false;
    if (forest_height_ != other.forest_height_) return false;

    if (num_positions_ != other.num_positions_) return false;
    if (num_velocities_ != other.num_velocities_) return false;
    if (num_states_ != other.num_states_) return false;

    if (bodies_ != other.bodies_) return false;
    if (frames_ != other.frames_) return false;
    if (mobilizers_ != other.mobilizers_) return false;
    if (force_elements_ != other.force_elements_) return false;
    if (joint_actuators_ != other.joint_actuators_) return false;
    if (body_nodes_ != other.body_nodes_) return false;

    return true;
  }

  // Returns the number of bodies in the multibody tree. This includes the
  // "world" body and therefore the minimum number of bodies after
  // MultibodyTree::Finalize() will always be one, not zero.
  int num_bodies() const { return ssize(bodies_); }

  // Returns the number of physical frames in the multibody tree.
  int num_frames() const {
    return ssize(frames_);
  }

  // Returns the number of mobilizers in the multibody tree. Since the "world"
  // body does not have a mobilizer, the number of mobilizers will always equal
  // the number of mobilized bodies minus one.
  int num_mobilizers() const {
    return ssize(mobilizers_);
  }

  // Returns the number of mobilized bodies (BodyNodes). Currently this is
  // restricted to being equal to the number of user-supplied Body objects.
  // TODO(sherm1) Relax this restriction -- the number of mobilized bodies can
  //  differ from the number of user-provided links.
  int num_mobods() const {
    return ssize(body_nodes_);
  }

  // Returns the number of force elements in the topology.
  int num_force_elements() const {
    return ssize(force_elements_);
  }

  // Returns the number of joint actuators in the topology.
  int num_joint_actuators() const {
    return ssize(joint_actuators_);
  }

  // Returns the number of levels in the forest topology.
  int forest_height() const {
    return forest_height_;
  }

  // Returns a constant reference to the corresponding FrameTopology given the
  // FrameIndex.
  const FrameTopology& get_frame(FrameIndex index) const {
    DRAKE_ASSERT(index < num_frames());
    return frames_[index];
  }

  // Returns a constant reference to the corresponding BodyTopology given a
  // BodyIndex.
  const BodyTopology& get_body(BodyIndex index) const {
    DRAKE_ASSERT(index < num_bodies());
    return bodies_[index];
  }

  // Mutable version of get_body().
  BodyTopology& get_mutable_body(BodyIndex index) {
    DRAKE_ASSERT(index < num_bodies());
    return bodies_[index];
  }

  // Returns a constant reference to the corresponding BodyTopology given a
  // BodyIndex.
  const MobilizerTopology& get_mobilizer(MobilizerIndex index) const {
    DRAKE_ASSERT(index < num_mobilizers());
    return mobilizers_[index];
  }

  // Returns a constant reference to the corresponding JointActuatorTopology
  // given a JointActuatorIndex.
  const JointActuatorTopology& get_joint_actuator(
      JointActuatorIndex index) const {
    DRAKE_ASSERT(index < num_joint_actuators());
    return joint_actuators_[index];
  }

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
  int num_trees() const {
    return ssize(num_tree_velocities_);
  }

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

  // Returns the tree index for the b-th body. The tree index for the world
  // body, BodyIndex(0), is invalid. Check with TreeIndex::is_valid().
  // @pre Index b is valid and b < num_bodies().
  TreeIndex body_to_tree_index(BodyIndex b) const {
    DRAKE_ASSERT(b < num_bodies());
    return body_to_tree_index_[b];
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

  // Creates and adds a new BodyTopology to this MultibodyTreeTopology.
  // The BodyTopology will be assigned a new, unique BodyIndex and FrameIndex
  // values.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns a std::pair<BodyIndex, FrameIndex> containing the indexes
  // assigned to the new BodyTopology.
  std::pair<BodyIndex, FrameIndex> add_body() {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more bodies is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    BodyIndex body_index = BodyIndex(num_bodies());
    FrameIndex body_frame_index = add_frame(body_index);
    bodies_.emplace_back(body_index, body_frame_index);
    return std::make_pair(body_index, body_frame_index);
  }

  // Creates and adds a new FrameTopology, associated with the given
  // body_index, to this MultibodyTreeTopology.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The FrameIndex assigned to the new FrameTopology.
  FrameIndex add_frame(BodyIndex body_index) {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more frames is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    FrameIndex frame_index(num_frames());
    frames_.emplace_back(frame_index, body_index);
    return frame_index;
  }

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
  MobilizerIndex add_mobilizer(
      FrameIndex in_frame, FrameIndex out_frame,
      int num_positions, int num_velocities) {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more mobilizers is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    // Note: MultibodyTree double checks the mobilizer's frames belong to that
    // tree. Therefore the validity of in_frame and out_frame is already
    // guaranteed. We add the checks here for additional security.
    DRAKE_THROW_UNLESS(in_frame < num_frames());
    DRAKE_THROW_UNLESS(out_frame < num_frames());
    if (in_frame == out_frame) {
      throw std::runtime_error(
          "Attempting to add a mobilizer between a frame and itself");
    }
    if (IsThereAMobilizerBetweenFrames(in_frame, out_frame)) {
      throw std::runtime_error(fmt::format(
          "This multibody tree already has a mobilizer connecting "
          "inboard frame (index={}) and outboard frame (index={}). "
          "More than one mobilizer between two frames is not allowed.",
          in_frame, out_frame));
    }
    const BodyIndex inboard_body = frames_[in_frame].body;
    const BodyIndex outboard_body = frames_[out_frame].body;
    if (IsThereAMobilizerBetweenBodies(inboard_body, outboard_body)) {
      throw std::runtime_error(fmt::format(
          "This multibody tree already has a mobilizer connecting "
          "inboard body (index={}) and outboard body (index={}). "
          "More than one mobilizer between two bodies is not allowed.",
          inboard_body, outboard_body));
    }
    // Checks for graph loops. Each body can have only one inboard mobilizer.
    if (bodies_[outboard_body].inboard_mobilizer.is_valid()) {
      throw std::runtime_error(
          "When creating a model, an attempt was made to add two inboard "
          "joints to the same body; this is not allowed. One possible cause "
          "might be attempting to weld a robot to World somewhere other "
          "than its base link; see Drake issue #17429 for discussion and "
          "work-arounds, e.g., reversing some joint parent/child directions. "
          "Another possible cause might be attempting to form a kinematic "
          "loop using joints; to create a loop, consider using a "
          "LinearBushingRollPitchYaw instead of a joint.");
    }

    // The checks above guarantee that it is the first time we add an inboard
    // mobilizer to `outboard_body`. The DRAKE_DEMANDs below double check our
    // implementation.
    // BodyTopology::inboard_mobilizer and BodyTopology::parent_body are both
    // set within this method right after these checks.
    DRAKE_DEMAND(!bodies_[outboard_body].inboard_mobilizer.is_valid());
    DRAKE_DEMAND(!bodies_[outboard_body].parent_body.is_valid());
    MobilizerIndex mobilizer_index(num_mobilizers());

    // Make note of the inboard mobilizer for the outboard body.
    bodies_[outboard_body].inboard_mobilizer = mobilizer_index;
    // Similarly, record inboard_body as the parent of outboard_body.
    bodies_[outboard_body].parent_body = inboard_body;

    // Records "child" bodies for bookkeeping in the context of the tree
    // structure of MultibodyTree.
    bodies_[inboard_body].child_bodies.push_back(outboard_body);

    mobilizers_.emplace_back(mobilizer_index,
                             in_frame, out_frame,
                             inboard_body, outboard_body,
                             num_positions, num_velocities);
    return mobilizer_index;
  }

  // Creates and adds a new ForceElementTopology, associated with the given
  // force_index, to this MultibodyTreeTopology.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The ForceElementIndex assigned to the new ForceElementTopology.
  ForceElementIndex add_force_element() {
    if (is_valid()) {
      throw std::logic_error(
          "This MultibodyTreeTopology is finalized already. "
              "Therefore adding more force elements is not allowed. "
              "See documentation for Finalize() for details.");
    }
    ForceElementIndex force_index(num_force_elements());
    force_elements_.emplace_back(force_index);
    return force_index;
  }

  // Creates and adds a new JointActuatorTopology for a joint with `num_dofs`
  // degrees of freedom.
  // @param[in] num_dofs
  //   The number of joint dofs actuated by this actuator.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The JointActuatorIndex assigned to the new JointActuatorTopology.
  JointActuatorIndex add_joint_actuator(int num_dofs) {
    DRAKE_ASSERT(num_dofs > 0);
    if (is_valid()) {
      throw std::logic_error(
          "This MultibodyTreeTopology is finalized already. "
          "Therefore adding more joint actuators is not allowed. "
          "See documentation for Finalize() for details.");
    }
    const int actuator_index_start = num_actuated_dofs();
    const JointActuatorIndex actuator_index(num_joint_actuators());
    joint_actuators_.emplace_back(
        actuator_index, actuator_index_start, num_dofs);
    num_actuated_dofs_ += num_dofs;
    return actuator_index;
  }

  // This method must be called by MultibodyTree::Finalize() after all
  // topological elements in the tree (corresponding to joints, bodies, force
  // elements, constraints) were added and before any computations are
  // performed.
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
  void Finalize() {
    // If the topology is valid it means that it was already finalized.
    // Re-compilation is not allowed.
    if (is_valid()) {
      throw std::logic_error(
          "Attempting to call MultibodyTree::Finalize() on an already "
          "finalized MultibodyTree.");
    }

    // For each body, assign a body node in a depth first traversal order.
    std::stack<BodyIndex> stack;
    stack.push(BodyIndex(0));  // Starts at the root.
    forest_height_ = 1;  // At least one level with the world body at the root.
    body_nodes_.reserve(num_bodies());
    while (!stack.empty()) {
      const MobodIndex node(num_mobods());
      const BodyIndex current = stack.top();
      const BodyIndex parent = bodies_[current].parent_body;

      bodies_[current].mobod_index = node;

      // Computes level.
      int level = 0;  // level = 0 for the world body.
      if (current != 0) {  // Not the world body.
        level = bodies_[parent].level + 1;
        const MobilizerIndex mobilizer = bodies_[current].inboard_mobilizer;
        mobilizers_[mobilizer].mobod_index = node;
      }
      // Updates body levels.
      bodies_[current].level = level;
      // Keep track of the number of levels, the deepest (i.e. max) level.
      forest_height_ = std::max(forest_height_, level + 1);

      // Since we are doing a DFT, it is valid to ask for the parent node,
      // unless we are at the root.
      MobodIndex parent_node;
      if (node != 0) {  // If we are not at the root:
        parent_node = bodies_[parent].mobod_index;
        body_nodes_[parent_node].child_nodes.push_back(node);
      }

      // Creates BodyNodeTopology.
      body_nodes_.emplace_back(
          node, level /* node index and level */,
          parent_node /* This node's parent */,
          current     /* This node's body */,
          bodies_[current].parent_body       /* This node's parent body */,
          bodies_[current].inboard_mobilizer /* This node's mobilizer */);

      // We process bodies in the order they were added to the vector of child
      // bodies; this vector is filled in the order mobilizers are added to the
      // model. Therefore, when a given node branches out, we spawn branches in
      // the order mobilizers that connect this node to its children were added.
      // Since we are using a stack to store bodies that will be processed next,
      // we must place bodies in reverse order so that the first child is at the
      // top of the stack.
      stack.pop();  // Pops top element.
      for (auto it = bodies_[current].child_bodies.rbegin();
           it != bodies_[current].child_bodies.rend(); ++it) {
        stack.push(*it);
      }
    }

    // Checks that all bodies were reached. We could have this situation if a
    // user adds a body but forgets to add a mobilizer to it.
    // Bodies that were not reached were not assigned a valid level.
    // TODO(amcastro-tri): this will stop at the first body that is not
    // connected to the tree. Add logic to emit a message with ALL bodies that
    // are not properly connected to the tree.
    for (BodyIndex body(0); body < num_bodies(); ++body) {
      if (bodies_[body].level < 0) {
        throw std::runtime_error("Body with index " + std::to_string(body) +
            " was not assigned a mobilizer");
      }
    }

    // After we checked all bodies were reached above, the number of tree nodes
    // should equal the number of bodies in the tree.
    DRAKE_DEMAND(num_bodies() == num_mobods());

    // Compile information regarding the size of the system:
    // - Number of degrees of freedom (generalized positions and velocities).
    // - Start/end indexes for each node.
    //
    // Base-to-Tip loop in DFT order, skipping the world (node = 0).

    // Count number of generalized positions and velocities.
    num_positions_ = 0;
    num_velocities_ = 0;
    for (const auto& mobilizer : mobilizers_) {
      num_positions_ += mobilizer.num_positions;
      num_velocities_ += mobilizer.num_velocities;
    }
    num_states_ = num_positions_ + num_velocities_;

    // Place all the generalized positions first followed by the generalized
    // velocities.
    int position_index = 0;
    int velocity_index_in_state = num_positions_;
    for (MobodIndex node_index(1);
         node_index < num_mobods(); ++node_index) {
      BodyNodeTopology& node = body_nodes_[node_index];
      MobilizerTopology& mobilizer = mobilizers_[node.mobilizer];

      // Note that a Weld mobilizer claims it starts at the next available
      // slot but has zero positions and velocities. That means the next
      // mobilizer will start at the same place.
      mobilizer.positions_start = position_index;
      mobilizer.velocities_start_in_state = velocity_index_in_state;
      mobilizer.velocities_start_in_v =
          velocity_index_in_state - num_positions_;
      DRAKE_DEMAND(0 <= mobilizer.velocities_start_in_v);

      position_index += mobilizer.num_positions;
      velocity_index_in_state += mobilizer.num_velocities;

      node.mobilizer_positions_start = mobilizer.positions_start;
      node.num_mobilizer_positions = mobilizer.num_positions;
      node.mobilizer_velocities_start_in_state =
          mobilizer.velocities_start_in_state;
      node.num_mobilizer_velocities = mobilizer.num_velocities;

      // Start index in a vector containing only generalized velocities.
      node.mobilizer_velocities_start_in_v = mobilizer.velocities_start_in_v;
      DRAKE_DEMAND(0 <= node.mobilizer_velocities_start_in_v);
      // If the last mobilizer is a Weld, it's "start" location will be
      // where it _would_ put velocities if it had any.
      DRAKE_DEMAND(node.mobilizer_velocities_start_in_v <= num_velocities_);
    }
    DRAKE_DEMAND(position_index == num_positions_);
    DRAKE_DEMAND(velocity_index_in_state == num_states_);

    // Update position/velocity indexes for free bodies so that they are easily
    // accessible.
    for (BodyTopology& body : bodies_) {
      if (body.is_floating) {
        DRAKE_DEMAND(body.inboard_mobilizer.is_valid());
        const MobilizerTopology& mobilizer =
            get_mobilizer(body.inboard_mobilizer);
        body.floating_positions_start = mobilizer.positions_start;
        body.floating_velocities_start_in_v =
            mobilizer.velocities_start_in_v;
      }
    }

    ExtractForestInfo();

    // We are done with a successful Finalize() and we mark it as so.
    // Do not add any more code after this!
    is_valid_ = true;
  }

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
  void GetKinematicPathToWorld(
      MobodIndex from, std::vector<MobodIndex>* path_to_world) const {
    DRAKE_THROW_UNLESS(path_to_world != nullptr);

    const int path_size = get_body_node(from).level + 1;
    path_to_world->resize(path_size);
    (*path_to_world)[0] = world_mobod_index();  // Add the world.
    if (from == world_mobod_index()) return;

    // Navigate the tree inwards starting at "from" and ending at the root.
    for (MobodIndex node = from; node > world_mobod_index();
        node = get_body_node(node).parent_body_node) {
      (*path_to_world)[get_body_node(node).level] = node;
    }
    // Verify the last added node to the path is a child of the world.
    DRAKE_DEMAND(get_body_node((*path_to_world)[1]).level == 1);
  }

  // Returns `true` if the body with index `body_index` is anchored to the
  // world.
  // A body is said to be "anchored" if its kinematics path to the world only
  // contains weld mobilizers.
  // The complexity of this operation is O(depth), where "depth" refers to the
  // depth in the tree of the body node associated with `body_index`.
  bool IsBodyAnchored(BodyIndex body_index) const {
    DRAKE_DEMAND(is_valid());
    const BodyTopology& body = get_body(body_index);
    std::vector<MobodIndex> path_to_world;
    GetKinematicPathToWorld(body.mobod_index, &path_to_world);
    // Skip the world at path_to_world[0].
    for (size_t path_index = 1; path_index < path_to_world.size();
         ++path_index) {
      const BodyNodeTopology& node = get_body_node(path_to_world[path_index]);
      const MobilizerTopology& mobilizer = get_mobilizer(node.mobilizer);
      // If any of the mobilizers in the path is not a weld mobilizer, the body
      // is not anchored.
      if (!mobilizer.is_weld_mobilizer()) return false;
    }
    // If the loop above completes, then body_index is anchored to the world.
    return true;
  }

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
  //   (num_bodies()). This corresponds to a topology with no weld mobilizers.
  // - The world body is also included in a welded-bodies set, and this set is
  //   element zero in the returned vector.
  // - The minimum size of the list is one. This corresponds to a topology with
  //   all bodies welded to the world.
  std::vector<std::set<BodyIndex>> CreateListOfWeldedBodies() const   {
    std::vector<std::set<BodyIndex>> welded_bodies_list;
    // Reserve the maximum possible of welded bodies (that is, when each body
    // forms its own welded body) in advance in order to avoid reallocation in
    // welded_bodies_list which would cause the invalidation of references as
    // we recursively fill it in.
    welded_bodies_list.reserve(num_bodies());
    welded_bodies_list.push_back(std::set<BodyIndex>{world_index()});
    // We build the list of welded bodies recursively, starting with the world
    // body added to the very first welded body in the list.
    std::set<BodyIndex>& bodies_welded_to_world = welded_bodies_list.back();
    CreateListOfWeldedBodiesRecurse(
        world_index(), &bodies_welded_to_world, &welded_bodies_list);
    return welded_bodies_list;
  }

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
      std::vector<BodyIndex> body_indexes) const {
    DRAKE_DEMAND(is_valid());
    std::unordered_set<BodyIndex> outboard_bodies;
    auto collect_body = [&outboard_bodies](const BodyNodeTopology& node) {
      outboard_bodies.insert(node.body);
    };
    for (const BodyIndex& body_index : body_indexes) {
      DRAKE_DEMAND(body_index.is_valid() && body_index < num_bodies());
      // Skip bodies that are already traversed because the subtree with it
      // being the root has necessarily been traversed already.
      if (outboard_bodies.count(body_index) == 0) {
        const BodyNodeTopology& root =
            get_body_node(get_body(body_index).mobod_index);
        TraverseOutboardNodes(root, collect_body);
      }
    }
    std::vector<BodyIndex> results(outboard_bodies.begin(),
                                   outboard_bodies.end());
    std::sort(results.begin(), results.end());
    return results;
  }

 private:
  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the frames with indexes `frame` and `frame2`.
  bool IsThereAMobilizerBetweenFrames(
      FrameIndex frame1, FrameIndex frame2) const {
    for (const auto& mobilizer_topology : mobilizers_) {
      if (mobilizer_topology.connects_frames(frame1, frame2)) return true;
    }
    return false;
  }

  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the bodies with indexes `body2` and `body2`.
  bool IsThereAMobilizerBetweenBodies(
      BodyIndex body1, BodyIndex body2) const {
    for (const auto& mobilizer_topology : mobilizers_) {
      if (mobilizer_topology.connects_bodies(body1, body2)) return true;
    }
    return false;
  }

  // Recursive helper method for CreateListOfWeldedBodies().
  // This method scans the children of body with parent_index. If a child is
  // welded to body with parent_index, it gets added to the parent's body welded
  // body, parent_welded_body. Otherwise a new welded body is created for the
  // child body and gets added to the list of all welded bodies, welded_bodies.
  void CreateListOfWeldedBodiesRecurse(
      BodyIndex parent_index, std::set<BodyIndex> *parent_welded_body,
      std::vector<std::set<BodyIndex>> *welded_bodies) const {
    const BodyTopology& parent = get_body(parent_index);
    for (BodyIndex child_index : parent.child_bodies) {
      const BodyTopology& child = get_body(child_index);
      const MobilizerTopology& child_mobilizer =
          get_mobilizer(child.inboard_mobilizer);
      if (child_mobilizer.is_weld_mobilizer()) {
        // If the child body is welded to the parent body, we then add it to
        // the parent's body welded body, parent_welded_body. We continue the
        // recursion down the tree starting at child.
        parent_welded_body->insert(child_index);
        CreateListOfWeldedBodiesRecurse(
            child_index, parent_welded_body, welded_bodies);
      } else {
        // If the child body is not welded to the parent body, then we create a
        // new welded body to which child is added. We continue the recursion
        // down the tree starting at child.
        welded_bodies->push_back(std::set<BodyIndex>{child_index});
        std::set<BodyIndex>& child_group = welded_bodies->back();
        CreateListOfWeldedBodiesRecurse(child_index,
                                        &child_group,
                                        welded_bodies);
      }
    }
  }

  // This traverses the tree of nodes outboard of `base` and applies `operation`
  // on each of them, starting with `base`. The traversal is performed in depth
  // first order.
  // @pre Body nodes were already created and therefore they are indexed in
  // depth first order.
  void TraverseOutboardNodes(
      const BodyNodeTopology& base,
      std::function<void(const BodyNodeTopology&)> operation) const {
    DRAKE_DEMAND(num_mobods() != 0);
    operation(base);
    // We are done if the base has no more children.
    if (base.get_num_children() == 0) return;
    // Traverse outboard nodes. Since the tree is finalized, we know nodes are
    // in DFT order.
    const int base_level = base.level;
    for (MobodIndex node_index(base.index + 1);
         /* Reached the last node in the model. */
         node_index < num_bodies() &&
         /* Reached next tree in the multibody forest */
         get_body_node(node_index).level > base_level;
         ++node_index) {
      operation(get_body_node(node_index));
    }
  }

  // Computes the number of generalized velocities in the tree composed of the
  // nodes outboard of `base`, including the generalized velocities of `base`.
  // @pre Body nodes were already created.
  int CalcNumberOfOutboardVelocities(const BodyNodeTopology& base) const {
    DRAKE_DEMAND(num_mobods() != 0);
    int nv = 0;
    TraverseOutboardNodes(base, [&nv](const BodyNodeTopology& node) {
      nv += node.num_mobilizer_velocities;
    });
    return nv;
  }

  // Helper method to be used within Finalize() to obtain the topological
  // information that describes the multibody system as a "forest" of trees.
  void ExtractForestInfo() {
    const BodyNodeTopology& root = get_body_node(world_mobod_index());
    const int max_num_trees = root.child_nodes.size();
    num_tree_velocities_.reserve(max_num_trees);
    body_to_tree_index_.resize(num_bodies());
    velocity_to_tree_index_.resize(num_velocities());

    for (const MobodIndex& root_child_index : root.child_nodes) {
      const BodyNodeTopology& root_child = get_body_node(root_child_index);
      const int nt = CalcNumberOfOutboardVelocities(root_child);
      const TreeIndex tree_index(num_trees());
      num_tree_velocities_.push_back(nt);
      TraverseOutboardNodes(root_child, [&](const BodyNodeTopology& node) {
        // We recurse all bodies in this tree (with tree_index) to fill in the
        // maps from body index to tree index and from velocity index to tree
        // index.
        body_to_tree_index_[node.body] = tree_index;
        for (int i = 0; i < node.num_mobilizer_velocities; ++i) {
          const int v = node.mobilizer_velocities_start_in_v + i;
          velocity_to_tree_index_[v] = tree_index;
        }
      });
    }

    // N.B. For trees with no generalized velocities, this code sets
    // tree_velocities_start_in_v_[t] to point to the last dof (plus one) of the
    // last tree with non-zero velocities. The reason to do so is that we want
    // users of MultibodyTreeTopology to write code like so:
    //
    // const MultibodyTreeTopology& topology = ...
    // for (TreeIndex t(0); t < topology.num_trees(); ++t) {
    //   for (int m = 0; m < topology.num_tree_velocities(t); ++m) {
    //     const int v = topology.tree_velocities_start(t) + m;
    //     // ...
    //   }
    // }
    //
    // In the snippet above index v points to an entry in the vector of
    // generalized velocities for the full model that corresponds to the m-th
    // mobility for the t-th tree.
    tree_velocities_start_in_v_.resize(num_trees(), 0);
    for (int t = 1; t < num_trees(); ++t) {
      tree_velocities_start_in_v_[t] =
          tree_velocities_start_in_v_[t - 1] + num_tree_velocities_[t - 1];
    }
  }

  // is_valid is set to `true` after a successful Finalize().
  bool is_valid_{false};
  // Number of levels (or generations) in the forest topology. After Finalize()
  // there will be at least one level (level = 0) with the world body.
  int forest_height_{-1};

  // Topological elements:
  std::vector<FrameTopology> frames_;
  std::vector<BodyTopology> bodies_;
  std::vector<MobilizerTopology> mobilizers_;
  std::vector<ForceElementTopology> force_elements_;
  std::vector<JointActuatorTopology> joint_actuators_;
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
  // t = body_to_tree_index_[b] is the tree index to which the b-th body
  // belongs.
  std::vector<TreeIndex> body_to_tree_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
