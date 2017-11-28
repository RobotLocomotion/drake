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
///  - To aid the process of cloning or transmogrifying multibody tree
///    components without having to create maps between the "original" and
///    "cloned" objects. That process is tedious and error prone.
///  - Each Multibody tree element has a copy (acquired at
///    MultibodyTree::Finalize() stage) of its topology which serves as a
///    key into the Context for that element's state.
///  - The topology is also stored in the Context so that the Multibody tree's
///    topology can be validated against the stored topology in debug builds.

#include <algorithm>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// Data structure to store the topological information associated with a Body.
struct BodyTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyTopology);

  /// Default construction to invalid configuration.
  BodyTopology() {}

  /// Constructs a body topology struct with index `body_index` and a body frame
  /// with index `frame_index`.
  BodyTopology(BodyIndex body_index, FrameIndex frame_index) :
      index(body_index), body_frame(frame_index) {}

  /// Returns `true` if all members of `this` topology are exactly equal to the
  /// members of `other`.
  bool operator==(const BodyTopology& other) const {
    if (index != other.index) return false;
    if (inboard_mobilizer.is_valid() !=
        other.inboard_mobilizer.is_valid()) return false;
    if (inboard_mobilizer.is_valid() &&
        inboard_mobilizer != other.inboard_mobilizer) return false;
    if (parent_body.is_valid() != other.parent_body.is_valid()) return false;
    if (parent_body.is_valid() &&
        parent_body != other.parent_body) return false;
    if (child_bodies != other.child_bodies) return false;
    if (body_frame != other.body_frame) return false;
    if (level != other.level) return false;
    if (body_node != other.body_node) return false;
    return true;
  }

  /// Unique index in the MultibodyTree.
  BodyIndex index{0};

  /// Unique index to the one and only inboard mobilizer a body can have.
  /// By default this is initialized to "invalid" so that we can detect
  /// graph loops within add_mobilizer().
  /// This will remain "invalid" for the world body.
  MobilizerIndex inboard_mobilizer{};

  /// Within the tree structure of a MultibodyTree, the immediate inboard (or
  /// "parent") body connected by the Mobilizer indexed by `inboard_mobilizer`.
  /// By default this is initialized to "invalid" so that we can assert
  /// (from within add_mobilizer()) that each body can have only one parent
  /// body. Also, this will remain "invalid" for the world body.
  BodyIndex parent_body{};

  /// Within the tree structure of a MultibodyTree, the immediate outboard (or
  /// "child") bodies to this Body.
  std::vector<BodyIndex> child_bodies;

  /// Unique index to the frame associated with this body.
  FrameIndex body_frame{0};

  /// Depth level in the MultibodyTree, level = 0 for the world.
  /// Initialized to an invalid negative value so that we can detect at
  /// Finalize() when a user forgets to connect a body with a mobilizer.
  int level{-1};

  /// Index to the tree body node in the MultibodyTree.
  BodyNodeIndex body_node;
};

/// Data structure to store the topological information associated with a
/// Frame.
struct FrameTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameTopology);

  // Default construction to invalid configuration.
  FrameTopology() {}

  /// Constructs a frame topology for a frame with index `frame_index`
  /// associated with a body with index `body_index`.
  FrameTopology(FrameIndex frame_index, BodyIndex body_index) :
      index(frame_index), body(body_index) {}

  /// Returns `true` if all members of `this` topology are exactly equal to the
  /// members of `other`.
  bool operator==(const FrameTopology& other) const {
    if (index != other.index) return false;
    if (body != other.body) return false;
    return true;
  }

  /// Unique index in the MultibodyTree.
  FrameIndex index{0};

  /// Unique index of the body this physical frame attaches to.
  BodyIndex body{0};
};

/// Data structure to store the topological information associated with a
/// Mobilizer object. It stores:
/// - Indexes to the inboard/outboard frames of this mobilizer.
/// - Indexes to the inboard/outboard bodies of this mobilizer.
/// - Numbers of dofs admitted by this mobilizer.
/// - Indexing information to retrieve entries from the parent MultibodyTree
///   Context.
/// Additional information on topology classes is given in this file's
/// documentation at the top.
struct MobilizerTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MobilizerTopology);

  /// Default construction to invalid configuration.
  MobilizerTopology() {}

  /// Constructs a %MobilizerTopology by specifying the the index
  /// `mobilizer_index` for `this` new topology, the indexes to the inboard and
  /// outboard frames the Mobilizer will connect, given by `in_frame` and
  /// `out_frame` respectively, and similarly the inboard and outboard bodies
  /// being connected, given by `in_body` and `out_body`, respectively.
  /// The constructed topology will correspond to that of a Mobilizer with
  /// `num_positions_in` generalized positions and `num_velocities_in`
  /// generalized velocities.
  MobilizerTopology(
      MobilizerIndex mobilizer_index,
      FrameIndex in_frame, FrameIndex out_frame,
      BodyIndex in_body, BodyIndex out_body,
      int num_positions_in, int num_velocities_in) :
      index(mobilizer_index),
      inboard_frame(in_frame), outboard_frame(out_frame),
      inboard_body(in_body), outboard_body(out_body),
      num_positions(num_positions_in), num_velocities(num_velocities_in) {}

  /// Returns `true` if all members of `this` topology are exactly equal to the
  /// members of `other`.
  bool operator==(const MobilizerTopology& other) const {
    if (index != other.index) return false;

    if (inboard_frame != other.inboard_frame) return false;
    if (outboard_frame != other.outboard_frame) return false;
    if (inboard_body != other.inboard_body) return false;
    if (outboard_body != other.outboard_body) return false;

    if (body_node != other.body_node) return false;

    if (num_positions != other.num_positions) return false;
    if (positions_start != other.positions_start) return false;
    if (num_velocities != other.num_velocities) return false;
    if (velocities_start != other.velocities_start) return false;

    return true;
  }

  /// Returns `true` if this %MobilizerTopology connects frames identified by
  /// indexes `frame1` and `frame2`.
  bool connects_frames(FrameIndex frame1, FrameIndex frame2) const {
    return (inboard_frame == frame1 && outboard_frame == frame2) ||
           (inboard_frame == frame2 && outboard_frame == frame1);
  }

  /// Returns `true` if this %MobilizerTopology connects bodies identified by
  /// indexes `body1` and `body2`.
  bool connects_bodies(BodyIndex body1, BodyIndex body2) const {
    return (inboard_body == body1 && outboard_body == body2) ||
           (inboard_body == body2 && outboard_body == body1);
  }

  /// Unique index in the set of mobilizers.
  MobilizerIndex index;
  /// Index to the inboard frame.
  FrameIndex inboard_frame;
  /// Index to the outboard frame.
  FrameIndex outboard_frame;
  /// Index to the inboard body.
  BodyIndex inboard_body;
  /// Index to the outboard body.
  BodyIndex outboard_body;
  /// Index to the tree node in the MultibodyTree responsible for this
  /// mobilizer's computations. See the documentation for BodyNodeTopology for
  /// further details on how these computations are organized.
  BodyNodeIndex body_node;

  /// Mobilizer indexing info: Set at Finalize() time.
  /// Number of generalized coordinates granted by this mobilizer.
  int num_positions{0};
  /// First entry in the global array of states, `x = [q v z]`, for the parent
  /// MultibodyTree.
  int positions_start{0};
  /// Number of generalized velocities granted by this mobilizer.
  int num_velocities{0};
  /// First entry in the global array of states, `x = [q v z]`, for the parent
  /// MultibodyTree.
  int velocities_start{0};

  /// Start index in a vector containing only generalized velocities.
  /// It is also a valid index into a vector of generalized accelerations (which
  /// are the time derivatives of the generalized velocities) and into a vector
  /// of generalized forces.
  int velocities_start_in_v{0};
};

/// Data structure to store the topological information associated with a
/// ForceElement.
struct ForceElementTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ForceElementTopology);

  /// Default construction to an invalid configuration. This only exists to
  /// satisfy demands of working with various container classes.
  ForceElementTopology() {}

  /// Constructs a force element topology with index `force_element_index`.
  explicit ForceElementTopology(ForceElementIndex force_element_index) :
      index(force_element_index) {}

  /// Returns `true` if all members of `this` topology are exactly equal to the
  /// members of `other`.
  bool operator==(const ForceElementTopology& other) const {
    if (index != other.index) return false;
    return true;
  }

  /// Unique index in the MultibodyTree.
  ForceElementIndex index{0};
};

/// Data structure to store the topological information associated with a tree
/// node. A tree node essentially consists of a body and its inboard mobilizer.
/// A body node is in charge of the computations associated to that body and
/// mobilizer, especially within a base-to-tip or tip-to-base recursion.
/// As the topological entity associated with a tree node (and specifically a
/// MultibodyTree node), this struct contains information regarding parent and
/// child nodes, parent and child bodies, etc.
struct BodyNodeTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyNodeTopology);

  /// Default construction to invalid configuration.
  BodyNodeTopology() {}

  /// Constructor specifying the topological information for a tree node.
  /// A tree node is instantiated for each body in the multibody system and it
  /// contains, in addition to that particular body, the inboard mobilizer
  /// connecting the body to the rest of the tree inwards (i.e. towards the
  /// world or root of the tree) from the mobilizer.
  ///
  /// @param index_in The unique index for `this` body node.
  /// @param level_in The level (depth or generation) in the tree structure.
  /// @param parent_node_in The parent node, in a tree structure sense, of
  ///                       `this` node.
  /// @param body_in The index to the body associated with this node.
  /// @param parent_body_in The parent body, in a tree structure sense, to
  ///                       `body_in`. In other words, `parent_body_in` is the
  ///                       body associated with node `parent_node_in`.
  /// @param mobilizer_in The index to the mobilizer associated with this node.
  BodyNodeTopology(
      BodyNodeIndex index_in, int level_in,
      BodyNodeIndex parent_node_in,
      BodyIndex body_in, BodyIndex parent_body_in, MobilizerIndex mobilizer_in)
      : index(index_in), level(level_in),
      parent_body_node(parent_node_in),
      body(body_in), parent_body(parent_body_in), mobilizer(mobilizer_in) {}

  /// Returns `true` if all members of `this` topology are exactly equal to the
  /// members of `other`.
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
    if (mobilizer_velocities_start != other.mobilizer_velocities_start)
      return false;

    if (num_flexible_positions != other.num_flexible_positions)
      return false;
    if (flexible_positions_start != other.flexible_positions_start)
      return false;
    if (num_flexible_velocities != other.num_flexible_velocities)
      return false;
    if (flexible_velocities_start != other.flexible_velocities_start)
      return false;

    return true;
  }

  /// Unique index of this node in the MultibodyTree.
  BodyNodeIndex index{};

  /// Depth level in the MultibodyTree, level = 0 for the world.
  int level{-1};

  /// The unique index to the parent BodyNode of this node.
  BodyNodeIndex parent_body_node;

  BodyIndex body;         // This node's body B.
  BodyIndex parent_body;  // This node's parent body P.

  MobilizerIndex mobilizer;  // The mobilizer connecting bodies P and B.

  /// The list of child body nodes to this node.
  std::vector<BodyNodeIndex> child_nodes;

  /// Returns the number of children to this node.
  int get_num_children() const { return static_cast<int>(child_nodes.size());}


  /// Start and number of dofs for this node's mobilizer.
  int num_mobilizer_positions{0};
  int mobilizer_positions_start{0};
  int num_mobilizer_velocities{0};
  int mobilizer_velocities_start{0};

  // Start index in a vector containing only generalized velocities.
  // It is also a valid index into a vector of generalized accelerations (which
  // are the time derivatives of the generalized velocities).
  int mobilizer_velocities_start_in_v{0};

  /// Start and number of dofs for this node's body (flexible dofs).
  int num_flexible_positions{0};
  int flexible_positions_start{0};
  int num_flexible_velocities{0};
  int flexible_velocities_start{0};
};

/// Data structure to store the topological information associated with an
/// entire MultibodyTree.
class MultibodyTreeTopology {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  /// Default constructor creates an empty, invalid topology. The minimum valid
  /// topology for a minimum valid MultibodyTree contains at least the
  /// BodyTopology for the world. The topology for the _world_ body does not get
  /// added until MultibodyTree construction, which creates a _world_ body
  /// and adds it to the tree.
  MultibodyTreeTopology() {}

  /// Returns `true` if all members of `this` topology are exactly equal to the
  /// members of `other`.
  bool operator==(const MultibodyTreeTopology& other) const {
    if (is_valid_ != other.is_valid_) return false;
    if (tree_height_ != other.tree_height_) return false;

    if (num_positions_ != other.num_positions_) return false;
    if (num_velocities_ != other.num_velocities_) return false;
    if (num_states_ != other.num_states_) return false;

    if (bodies_ != other.bodies_) return false;
    if (frames_ != other.frames_) return false;
    if (mobilizers_ != other.mobilizers_) return false;
    if (body_nodes_ != other.body_nodes_) return false;

    return true;
  }

  /// Returns the number of bodies in the multibody tree. This includes the
  /// "world" body and therefore the minimum number of bodies after
  /// MultibodyTree::Finalize() will always be one, not zero.
  int get_num_bodies() const { return static_cast<int>(bodies_.size()); }

  /// Returns the number of physical frames in the multibody tree.
  int get_num_frames() const {
    return static_cast<int>(frames_.size());
  }

  /// Returns the number of mobilizers in the multibody tree. Since the "world"
  /// body does not have a mobilizer, the number of mobilizers will always equal
  /// the number of bodies minus one.
  int get_num_mobilizers() const {
    return static_cast<int>(mobilizers_.size());
  }

  /// Returns the number of tree nodes. This must equal the number of bodies.
  int get_num_body_nodes() const {
    return static_cast<int>(body_nodes_.size());
  }

  /// Returns the number of force elements in the multibody tree.
  int get_num_force_elements() const {
    return static_cast<int>(force_elements_.size());
  }

  /// Returns the number of tree levels in the topology.
  int get_tree_height() const {
    return tree_height_;
  }

  /// Returns a constant reference to the corresponding FrameTopology given the
  /// FrameIndex.
  const FrameTopology& get_frame(FrameIndex index) const {
    DRAKE_ASSERT(index < get_num_frames());
    return frames_[index];
  }

  /// Returns a constant reference to the corresponding BodyTopology given a
  /// BodyIndex.
  const BodyTopology& get_body(BodyIndex index) const {
    DRAKE_ASSERT(index < get_num_bodies());
    return bodies_[index];
  }

  /// Returns a constant reference to the corresponding BodyTopology given a
  /// BodyIndex.
  const MobilizerTopology& get_mobilizer(MobilizerIndex index) const {
    DRAKE_ASSERT(index < get_num_mobilizers());
    return mobilizers_[index];
  }

  /// Returns a constant reference to the corresponding BodyNodeTopology given
  /// a BodyNodeIndex.
  const BodyNodeTopology& get_body_node(BodyNodeIndex index) const {
    DRAKE_ASSERT(index < get_num_body_nodes());
    return body_nodes_[index];
  }

  /// Creates and adds a new BodyTopology to this MultibodyTreeTopology.
  /// The BodyTopology will be assigned a new, unique BodyIndex and FrameIndex
  /// values.
  ///
  /// @throws std::logic_error if Finalize() was already called on `this`
  /// topology.
  ///
  /// @returns a std::pair<BodyIndex, FrameIndex> containing the indexes
  /// assigned to the new BodyTopology.
  std::pair<BodyIndex, FrameIndex> add_body() {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more bodies is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    BodyIndex body_index = BodyIndex(get_num_bodies());
    FrameIndex body_frame_index = add_frame(body_index);
    bodies_.emplace_back(body_index, body_frame_index);
    return std::make_pair(body_index, body_frame_index);
  }

  /// Creates and adds a new FrameTopology, associated with the given
  /// body_index, to this MultibodyTreeTopology.
  ///
  /// @throws std::logic_error if Finalize() was already called on `this`
  /// topology.
  ///
  /// @returns The FrameIndex assigned to the new FrameTopology.
  FrameIndex add_frame(BodyIndex body_index) {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more frames is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    FrameIndex frame_index(get_num_frames());
    frames_.emplace_back(frame_index, body_index);
    return frame_index;
  }

  /// Creates and adds a new MobilizerTopology connecting the inboard and
  /// outboard multibody frames identified by indexes `in_frame` and
  /// `out_frame`, respectively. The created topology will correspond to that of
  /// a Mobilizer with `num_positions` and `num_velocities`.
  ///
  /// @throws std::runtime_error if either `in_frame` or `out_frame` do not
  /// index frame topologies in `this` %MultibodyTreeTopology.
  /// @throws a std::runtime_error if `in_frame == out_frame`.
  /// @throws a std::runtime_error if `in_frame` and `out_frame` already are
  /// connected by another mobilizer. More than one mobilizer between two frames
  /// is not allowed.
  /// @throws std::logic_error if Finalize() was already called on `this`
  /// topology.
  ///
  /// @returns The MobilizerIndex assigned to the new MobilizerTopology.
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
    DRAKE_THROW_UNLESS(in_frame < get_num_frames());
    DRAKE_THROW_UNLESS(out_frame < get_num_frames());
    if (in_frame == out_frame) {
      throw std::runtime_error(
          "Attempting to add a mobilizer between a frame and itself");
    }
    if (IsThereAMobilizerBetweenFrames(in_frame, out_frame)) {
      throw std::runtime_error(
          "This multibody tree already has a mobilizer connecting these two "
          "frames. More than one mobilizer between two frames is not allowed");
    }
    const BodyIndex inboard_body = frames_[in_frame].body;
    const BodyIndex outboard_body = frames_[out_frame].body;
    if (IsThereAMobilizerBetweenBodies(inboard_body, outboard_body)) {
      throw std::runtime_error(
          "This multibody tree already has a mobilizer connecting these two "
          "bodies. More than one mobilizer between two bodies is not allowed");
    }
    // Checks for graph loops. Each body can have only one inboard mobilizer.
    if (bodies_[outboard_body].inboard_mobilizer.is_valid()) {
      throw std::runtime_error(
          "This mobilizer is creating a closed loop since the outboard body "
          "already has an inboard mobilizer connected to it. "
          "If a physical loop is really needed, consider using a constraint "
          "instead.");
    }

    // The checks above guarantee that it is the first time we add an inboard
    // mobilizer to `outboard_body`. The DRAKE_DEMANDs below double check our
    // implementation.
    // BodyTopology::inboard_mobilizer and BodyTopology::parent_body are both
    // set within this method right after these checks.
    DRAKE_DEMAND(!bodies_[outboard_body].inboard_mobilizer.is_valid());
    DRAKE_DEMAND(!bodies_[outboard_body].parent_body.is_valid());
    MobilizerIndex mobilizer_index(get_num_mobilizers());

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

  /// Creates and adds a new ForceElementTopology, associated with the given
  /// force_index, to this MultibodyTreeTopology.
  ///
  /// @throws std::logic_error if Finalize() was already called on `this`
  /// topology.
  ///
  /// @returns The ForceElementIndex assigned to the new ForceElementTopology.
  ForceElementIndex add_force_element() {
    if (is_valid()) {
      throw std::logic_error(
          "This MultibodyTreeTopology is finalized already. "
              "Therefore adding more force elements is not allowed. "
              "See documentation for Finalize() for details.");
    }
    ForceElementIndex force_index(get_num_force_elements());
    force_elements_.emplace_back(force_index);
    return force_index;
  }

  /// This method must be called by MultibodyTree::Finalize() after all
  /// topological elements in the tree (corresponding to joints, bodies, force
  /// elements, constraints) were added and before any computations are
  /// performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to perform computations at a
  /// later stage. This preprocessing includes:
  /// - sorting in BFT order for fast recursions through the tree,
  /// - computation of state sizes and of pool sizes within cache entries,
  /// - computation of index maps to retrieve either state or cache entries for
  ///   each multibody element.
  ///
  /// If the finalize stage is successful, the `this` topology is validated,
  /// meaning it is up-to-date after this call.
  /// No more multibody tree elements can be added after a call to Finalize().
  ///
  /// @throws std::logic_error If users attempt to call this method on an
  ///         already finalized topology.
  /// @see is_valid()
  void Finalize() {
    // If the topology is valid it means that it was already finalized.
    // Re-compilation is not allowed.
    if (is_valid()) {
      throw std::logic_error(
          "Attempting to call MultibodyTree::Finalize() on an already """
          "finalized MultibodyTree.");
    }

    // Compute body levels in the tree. Root is the zero level.
    // Breadth First Traversal (a.k.a. Level Order Traversal).
    std::queue<BodyIndex> queue;
    queue.push(BodyIndex(0));  // Starts at the root.
    tree_height_ = 1;  // At least one level with the world body at the root.
    // While at it, create body nodes and index them in this BFT order for
    // fast tree traversals of MultibodyTree recursive algorithms.
    body_nodes_.reserve(get_num_bodies());
    while (!queue.empty()) {
      const BodyNodeIndex node(get_num_body_nodes());
      const BodyIndex current = queue.front();
      const BodyIndex parent = bodies_[current].parent_body;

      bodies_[current].body_node = node;

      // Computes level.
      int level = 0;  // level = 0 for the world body.
      if (current != 0) {  // Not the world body.
        level = bodies_[parent].level + 1;
        const MobilizerIndex mobilizer = bodies_[current].inboard_mobilizer;
        mobilizers_[mobilizer].body_node = node;
      }
      // Updates body levels.
      bodies_[current].level = level;
      // Keep track of the number of levels, the deepest (i.e. max) level.
      tree_height_ = std::max(tree_height_, level + 1);

      // Since we are doing a BFT, it is valid to ask for the parent node,
      // unless we are at the root.
      BodyNodeIndex parent_node;
      if (node != 0) {  // If we are not at the root:
        parent_node = bodies_[parent].body_node;
        body_nodes_[parent_node].child_nodes.push_back(node);
      }

      // Creates BodyNodeTopology.
      body_nodes_.emplace_back(
          node, level /* node index and level */,
          parent_node /* This node's parent */,
          current     /* This node's body */,
          bodies_[current].parent_body       /* This node's parent body */,
          bodies_[current].inboard_mobilizer /* This node's mobilizer */);

      // Pushes children to the back of the queue and pops current.
      for (BodyIndex child : bodies_[current].child_bodies) {
        queue.push(child);  // Pushes at the back.
      }
      queue.pop();  // Pops front element.
    }

    // Checks that all bodies were reached. We could have this situation if a
    // user adds a body but forgets to add a mobilizer to it.
    // Bodies that were not reached were not assigned a valid level.
    // TODO(amcastro-tri): this will stop at the first body that is not
    // connected to the tree. Add logic to emit a message with ALL bodies that
    // are not properly connected to the tree.
    for (BodyIndex body(0); body < get_num_bodies(); ++body) {
      if (bodies_[body].level < 0) {
        throw std::runtime_error("Body with index " + std::to_string(body) +
            " was not assigned a mobilizer");
      }
    }

    // After we checked all bodies were reached above, the number of tree nodes
    // should equal the number of bodies in the tree.
    DRAKE_DEMAND(get_num_bodies() == get_num_body_nodes());

    // Compile information regarding the size of the system:
    // - Number of degrees of freedom (generalized positions and velocities).
    // - Start/end indexes for each node.
    //
    // TODO(amcastro-tri): count body dofs (i.e. for flexible dofs).
    //
    // Base-to-Tip loop in BFT order, skipping the world (node = 0).

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
    int velocity_index = num_positions_;
    for (BodyNodeIndex node_index(1);
         node_index < get_num_body_nodes(); ++node_index) {
      BodyNodeTopology& node = body_nodes_[node_index];
      MobilizerTopology& mobilizer = mobilizers_[node.mobilizer];

      mobilizer.positions_start = position_index;
      mobilizer.velocities_start = velocity_index;
      mobilizer.velocities_start_in_v = velocity_index - num_positions_;
      DRAKE_DEMAND(0 <= mobilizer.velocities_start_in_v);

      position_index += mobilizer.num_positions;
      velocity_index += mobilizer.num_velocities;

      node.mobilizer_positions_start = mobilizer.positions_start;
      node.num_mobilizer_positions = mobilizer.num_positions;
      node.mobilizer_velocities_start = mobilizer.velocities_start;
      node.num_mobilizer_velocities = mobilizer.num_velocities;

      // Start index in a vector containing only generalized velocities.
      node.mobilizer_velocities_start_in_v = mobilizer.velocities_start_in_v;
      DRAKE_DEMAND(0 <= node.mobilizer_velocities_start_in_v);
      DRAKE_DEMAND(node.mobilizer_velocities_start_in_v < num_velocities_);
    }
    DRAKE_DEMAND(position_index == num_positions_);
    DRAKE_DEMAND(velocity_index == num_states_);

    // We are done with a successful Finalize() and we mark it as so.
    // Do not add any more code after this!
    is_valid_ = true;
  }

  /// Returns `true` if Finalize() was already called on `this` topology.
  /// @see Finalize()
  bool is_valid() const { return is_valid_; }

  /// Returns the total number of generalized positions in the model.
  int get_num_positions() const { return num_positions_; }

  /// Returns the total number of generalized velocities in the model.
  int get_num_velocities() const { return num_velocities_; }

  /// Returns the total size of the state vector in the model.
  int get_num_states() const { return num_states_; }

  /// Given a node in `this` topology, specified by its BodyNodeIndex `from`,
  /// this method computes the kinematic path formed by all the nodes in the
  /// tree that connect `from` with the root (corresponding to the world).
  ///
  /// @param[in] from
  ///   A node in the tree topology to which the path to the root (world) is to
  ///   be computed.
  /// @param[out] path_to_world
  ///   A std::vector of body node indexes that on output will contain the path
  ///   to the root of the tree. Forward iteration (from element 0 to element
  ///   size()-1) of `path_to_world` will traverse all nodes in the tree
  ///   starting at the root along the path to `from`. That is, forward
  ///   iteration starts with the root of the tree at `path_to_world[0]` and
  ///   ends with `from` at `path_to_world.back()`.
  ///   On input, `path_to_world` must be a valid pointer. On output this vector
  ///   will be resized, only if needed, to store as many elements as the level
  ///   (BodyNodeTopology::level) of body node `from` plus one (so that we can
  ///   include the root node in the path).
  void GetKinematicPathToWorld(
      BodyNodeIndex from, std::vector<BodyNodeIndex>* path_to_world) const {
    DRAKE_THROW_UNLESS(path_to_world != nullptr);

    const int path_size = get_body_node(from).level + 1;
    path_to_world->resize(path_size);

    // Navigate the tree inwards starting at "from" and ending at the root.
    for (BodyNodeIndex node = from; node > BodyNodeIndex(0);
        node = get_body_node(node).parent_body_node) {
      (*path_to_world)[get_body_node(node).level] = node;
    }
    // Verify the last added node to the path is a child of the world.
    DRAKE_DEMAND(get_body_node((*path_to_world)[1]).level == 1);
    (*path_to_world)[0] = BodyNodeIndex(0);  // Add the world.
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

  // is_valid is set to `true` after a successful Finalize().
  bool is_valid_{false};
  // Number of levels (or generations) in the tree topology. After Finalize()
  // there will be at least one level (level = 0) with the world body.
  int tree_height_{-1};

  // Topological elements:
  std::vector<FrameTopology> frames_;
  std::vector<BodyTopology> bodies_;
  std::vector<MobilizerTopology> mobilizers_;
  std::vector<ForceElementTopology> force_elements_;
  std::vector<BodyNodeTopology> body_nodes_;

  // Total number of generalized positions and velocities in the MultibodyTree
  // model.
  int num_positions_{0};
  int num_velocities_{0};
  int num_states_{0};
};

}  // namespace multibody
}  // namespace drake
