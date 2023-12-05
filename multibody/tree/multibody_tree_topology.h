#pragma once

/// @file
/// This file defines the topological structures which represent the logical
/// connectivities between multibody tree elements. For instance, the
/// LinkTopology for a Body will contain the topological information specifying
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
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Data structure to store the topological information associated with a Link.
struct LinkTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinkTopology);

  // Default construction to invalid configuration.
  LinkTopology() {}

  // Constructs a link topology struct with index `link_index` and a link frame
  // with index `frame_index`.
  LinkTopology(LinkIndex link_index, FrameIndex frame_index) :
      index(link_index), link_frame(frame_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const LinkTopology& other) const {
    if (index != other.index) return false;
    if (inboard_mobilizer.is_valid() != other.inboard_mobilizer.is_valid())
      return false;
    if (inboard_mobilizer.is_valid() &&
        inboard_mobilizer != other.inboard_mobilizer)
      return false;
    if (parent_link.is_valid() != other.parent_link.is_valid()) return false;
    if (parent_link.is_valid() &&
        parent_link != other.parent_link) return false;
    if (child_links != other.child_links) return false;
    if (link_frame != other.link_frame) return false;
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

  // Index in the MultibodyPlant and LinkJointGraph. World is 0.
  LinkIndex index{0};

  // Index to the one and only inboard mobilizer a Link can have.
  // This will remain "invalid" for World.
  MobodIndex inboard_mobilizer{};

  // Within the SpanningForest, the Link immediately inboard of this Link;
  // That is, the Link at the other side of the inboard_mobilizer at a level
  // one lower (closer to World) in the Forest. We're calling this "parent"
  // here but don't confuse it with the parent Link of a Joint.
  // This will remain "invalid" for World.
  LinkIndex parent_link{};

  // Within the SpanningForest, the immediate outboard (or
  // "child") Links to this Link. Links appear in child_links in the order
  // mobilizers were added to the model, with
  // MultibodyTreeTopology::add_mobilizer().
  std::vector<LinkIndex> child_links;

  // Unique index to the frame associated with this Link.
  FrameIndex link_frame{0};

  // Depth level in the SpanningForest, level = 0 for World.
  int level{-1};

  // Index to the mobilized body (BodyNode) modeling this Link in the
  // SpanningForest.
  MobodIndex mobod_index;

  // `true` if this topology corresponds to a floating Link.
  bool is_floating{false};

  // `true` if this topology corresponds to a floating Link with rotations
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
  // associated with a Link with index `link_index`.
  FrameTopology(FrameIndex frame_index, LinkIndex link_index) :
      index(frame_index), link(link_index) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const FrameTopology& other) const {
    if (index != other.index) return false;
    if (link != other.link) return false;
    return true;
  }

  // Index in the MultibodyPlant.
  FrameIndex index{0};

  // Index of the Link this physical frame attaches to.
  LinkIndex link{0};
};

// Data structure to store the topological information associated with a
// Mobilizer object. It stores:
//
// - Indexes to the inboard/outboard frames of this mobilizer.
// - Indexes to the inboard/outboard Links of this mobilizer.
// - Numbers of dofs admitted by this mobilizer.
// - Indexing information to retrieve entries from the parent MultibodyTree
//   Context.
//
// Additional information on topology classes is given in this file's
// documentation at the top.
struct MobilizerTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MobilizerTopology);

  // Constructs a MobilizerTopology from the corresponding Mobod, plus the
  // connected Links and the Frame geometric information from the modeled Joint.
  MobilizerTopology(
      MobodIndex mobilizer_index,
      FrameIndex in_frame, FrameIndex out_frame,
      LinkIndex in_link, LinkIndex out_link,
      const SpanningForest::Mobod& mobod) :
      index(mobilizer_index),
      inboard_frame(in_frame), outboard_frame(out_frame),
      inboard_link(in_link), outboard_link(out_link),
      num_positions(mobod.nq()),
      positions_start(mobod.q_start()),
      num_velocities(mobod.nv()), velocities_start_in_v(mobod.v_start()) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MobilizerTopology& other) const {
    if (index != other.index) return false;

    if (inboard_frame != other.inboard_frame) return false;
    if (outboard_frame != other.outboard_frame) return false;
    if (inboard_link != other.inboard_link) return false;
    if (outboard_link != other.outboard_link) return false;

    if (num_positions != other.num_positions) return false;
    if (positions_start != other.positions_start) return false;
    if (num_velocities != other.num_velocities) return false;
    if (velocities_start_in_v != other.velocities_start_in_v) return false;

    return true;
  }

  // Returns `true` if this MobilizerTopology connects frames identified by
  // indexes `frame1` and `frame2`.
  bool connects_frames(FrameIndex frame1, FrameIndex frame2) const {
    return (inboard_frame == frame1 && outboard_frame == frame2) ||
           (inboard_frame == frame2 && outboard_frame == frame1);
  }

  // Returns `true` if this MobilizerTopology connects Links identified by
  // indexes `link1` and `link2`.
  bool connects_links(LinkIndex link1, LinkIndex link2) const {
    return (inboard_link == link1 && outboard_link == link2) ||
           (inboard_link == link2 && outboard_link == link1);
  }

  // Returns `true` if this mobilizer topology corresponds to that of a weld
  // mobilizer.
  bool is_weld_mobilizer() const {
    return num_velocities == 0;
  }

  // Unique index in the set of mobilizers. There will be a corresponding
  // BodyNodeTopology with the same index.
  MobodIndex index;
  // Index to the inboard frame.
  FrameIndex inboard_frame;
  // Index to the outboard frame.
  FrameIndex outboard_frame;
  // Index to the inboard body.
  LinkIndex inboard_link;
  // Index to the outboard body.
  LinkIndex outboard_link;

  // Mobilizer indexing info: Set at Finalize() time.
  // Number of generalized coordinates granted by this mobilizer.
  int num_positions{0};
  // First entry in the global array of states, `x = [q v z]`, for the parent
  // MultibodyTree.
  int positions_start{0};
  // Number of generalized velocities granted by this mobilizer.
  int num_velocities{0};

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
  //                       `this` node.
  // @param link_in The index to the body associated with this node.
  // @param parent_link_in The parent body, in a tree structure sense, to
  //                       `link_in`. In other words, `parent_link_in` is the
  //                       body associated with node `parent_node_in`.
  BodyNodeTopology(
      MobodIndex index_in, int level_in,
      MobodIndex parent_node_in,
      LinkIndex link_in, LinkIndex parent_link_in)
      : index(index_in), level(level_in),
        parent_body_node(parent_node_in),
        link(link_in), parent_link(parent_link_in) {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const BodyNodeTopology& other) const {
    if (index != other.index) return false;
    if (level != other.level) return false;

    if (parent_body_node.is_valid() !=
        other.parent_body_node.is_valid()) return false;
    if (parent_body_node.is_valid() &&
        parent_body_node != other.parent_body_node) return false;

    if (link != other.link) return false;

    if (parent_link.is_valid() != other.parent_link.is_valid()) return false;
    if (parent_link.is_valid() &&
        parent_link != other.parent_link) return false;

    if (child_nodes != other.child_nodes) return false;

    if (num_mobilizer_positions != other.num_mobilizer_positions)
      return false;
    if (mobilizer_positions_start != other.mobilizer_positions_start)
      return false;
    if (num_mobilizer_velocities != other.num_mobilizer_velocities)
      return false;
    if (mobilizer_velocities_start_in_v !=
        other.mobilizer_velocities_start_in_v)
      return false;

    return true;
  }

  // Index of this node in the SpanningForest (0 for World). There
  // is an associated Mobilizer with the same index.
  MobodIndex index{};

  // Depth level in the SpanningForest, level = 0 for World.
  int level{-1};

  // The index to the inboard ("parent") BodyNode of this node.
  MobodIndex parent_body_node;

  LinkIndex link;         // This node's Link B.
  LinkIndex parent_link;  // This node's parent Link P.

  // The list of outboard ("child") body nodes to this node.
  std::vector<MobodIndex> child_nodes;

  // Returns the number of children to this node.
  int get_num_children() const { return ssize(child_nodes);}

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
  // LinkTopology for World.
  MultibodyTreeTopology() {}

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MultibodyTreeTopology& other) const {
    if (is_valid_ != other.is_valid_) return false;
    if (forest_height_ != other.forest_height_) return false;

    if (num_positions_ != other.num_positions_) return false;
    if (num_velocities_ != other.num_velocities_) return false;
    if (num_states_ != other.num_states_) return false;
    if (num_actuated_dofs_ != other.num_actuated_dofs_) return false;

    if (frames_ != other.frames_) return false;
    if (links_ != other.links_) return false;
    if (mobilizers_ != other.mobilizers_) return false;
    if (force_elements_ != other.force_elements_) return false;
    if (joint_actuators_ != other.joint_actuators_) return false;
    if (body_nodes_ != other.body_nodes_) return false;

    return true;
  }

  // Returns the number of Links in the MultibodyPlant. This includes the
  // World Link and therefore the minimum number of links after
  // MultibodyTree::Finalize() will always be one, not zero.
  int num_links() const { return ssize(links_); }

  // Returns the number of physical frames in the multibody tree.
  int num_frames() const {
    return ssize(frames_);
  }

  // Returns the number of Mobilizers. This is always the same as the number
  // of BodyNodes and the number of mobilized bodies in the SpanningForest.
  int num_mobilizers() const { return ssize(mobilizers_); }

  // Returns the number of BodyNodes. These are generated 1:1 from the
  // MobilizedBodies in the SpanningForest. If we combined welded Links,
  // there will be more Links than BodyNodes.
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

  // Returns a constant reference to the corresponding LinkTopology given a
  // LinkIndex.
  const LinkTopology& get_link(LinkIndex index) const {
    DRAKE_ASSERT(index < num_links());
    return links_[index];
  }

  // Mutable version of get_link().
  LinkTopology& get_mutable_link(LinkIndex index) {
    DRAKE_ASSERT(index < num_links());
    return links_[index];
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
  // body, LinkIndex(0), is invalid. Check with TreeIndex::is_valid().
  // @pre Index b is valid and b < num_links().
  TreeIndex body_to_tree_index(LinkIndex b) const {
    DRAKE_ASSERT(b < num_links());
    return link_to_tree_index_[b];
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

  // Creates and adds a new LinkTopology to this MultibodyTreeTopology.
  // The LinkTopology will be assigned a new, unique LinkIndex and FrameIndex
  // values.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns a std::pair<LinkIndex, FrameIndex> containing the indexes
  // assigned to the new LinkTopology.
  std::pair<LinkIndex, FrameIndex> add_link() {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more Links is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    LinkIndex link_index = LinkIndex(num_links());
    FrameIndex link_frame_index = add_frame(link_index);
    links_.emplace_back(link_index, link_frame_index);
    return std::make_pair(link_index, link_frame_index);
  }

  // Creates and adds a new FrameTopology, associated with the given
  // link_index, to this MultibodyTreeTopology.
  //
  // @throws std::exception if Finalize() was already called on `this`
  // topology.
  //
  // @returns The FrameIndex assigned to the new FrameTopology.
  FrameIndex add_frame(LinkIndex link_index) {
    if (is_valid()) {
      throw std::logic_error("This MultibodyTreeTopology is finalized already. "
                             "Therefore adding more frames is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    FrameIndex frame_index(num_frames());
    frames_.emplace_back(frame_index, link_index);
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
  // @returns The MobodIndex assigned to the new MobilizerTopology.
  MobodIndex add_mobilizer(const SpanningForest::Mobod& mobod,
                           FrameIndex in_frame, FrameIndex out_frame) {
    if (is_valid()) {
      throw std::logic_error(
          "This MultibodyTreeTopology is finalized already. "
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
    const LinkIndex inboard_link = frames_[in_frame].link;
    const LinkIndex outboard_link = frames_[out_frame].link;
    if (IsThereAMobilizerBetweenLinks(inboard_link, outboard_link)) {
      throw std::runtime_error(fmt::format(
          "This multibody tree already has a mobilizer connecting "
          "inboard body (index={}) and outboard body (index={}). "
          "More than one mobilizer between two bodies is not allowed.",
          inboard_link, outboard_link));
    }
    // Checks for graph loops. Each body can have only one inboard mobilizer.
    if (links_[outboard_link].inboard_mobilizer.is_valid()) {
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
    // mobilizer to `outboard_link`. The DRAKE_DEMANDs below double check our
    // implementation.
    // LinkTopology::inboard_mobilizer and LinkTopology::parent_link are both
    // set within this method right after these checks.
    DRAKE_DEMAND(!links_[outboard_link].inboard_mobilizer.is_valid());
    DRAKE_DEMAND(!links_[outboard_link].parent_link.is_valid());
    DRAKE_DEMAND(mobod.index() == num_mobilizers());
    const MobodIndex mobilizer_index(num_mobilizers());

    // Make note of the inboard mobilizer for the outboard body.
    links_[outboard_link].inboard_mobilizer = mobilizer_index;
    // Similarly, record inboard_link as the parent of outboard_link.
    links_[outboard_link].parent_link = inboard_link;

    // Records "child" links for bookkeeping in the context of the tree
    // structure of MultibodyTree.
    links_[inboard_link].child_links.push_back(outboard_link);

    mobilizers_.emplace_back(mobilizer_index, in_frame, out_frame, inboard_link,
                             outboard_link, mobod);
    return mobilizer_index;
  }

  void add_world_mobilizer(const SpanningForest::Mobod& world_mobod,
                           FrameIndex world_body_frame) {
    DRAKE_DEMAND(world_mobod.is_world());
    const LinkIndex world_link_index = frames_[world_body_frame].link;
    DRAKE_DEMAND(world_link_index == LinkIndex(0));
    DRAKE_DEMAND(mobilizers_.empty());
    // There is no inboard frame for this mobilizer. We'll use the World
    // body frame and link for both frames & links so we don't have to deal
    // with invalid indices. No computation depends on those.
    mobilizers_.emplace_back(MobodIndex(0), world_body_frame,
                             world_body_frame, world_link_index,
                             world_link_index, world_mobod);
    links_[world_link_index].inboard_mobilizer = MobodIndex(0);
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
  // topological elements in the plant (links, joints, constraints) were added
  // and a suitable SpanningForest built.
  //
  // We extract the necessary topological information from the Forest, i.e.
  // how links and joints are interconnected, and use that information to build
  // BodyNodeTopology objects directly in depth-first order and extract Tree
  // structure.
  //
  // TODO(sherm1) For historical reasons we're extracting information from the
  //  Forest and distributing it but we should work directly from the Forest
  //  without duplication.
  //
  // If the finalize stage is successful, the `this` topology is validated,
  // meaning it is up-to-date after this call.
  // No more multibody tree elements can be added after a call to Finalize().
  //
  // @throws std::exception If users attempt to call this method on an
  //         already finalized topology.
  // @see is_valid()
  void Finalize(const LinkJointGraph& graph) {
    // If the topology is valid it means that it was already finalized.
    // Re-compilation is not allowed.
    if (is_valid()) {
      throw std::logic_error(
          "Attempting to call MultibodyTree::Finalize() on an already "
          "finalized MultibodyTree.");
    }
    DRAKE_DEMAND(graph.forest_is_valid());

    const SpanningForest& forest = graph.forest();

    // Create a BodyNodeTopology corresponding to each Mobod in the forest,
    // indexed identically. Also update LinkTopology, though in the case
    // where we combine welded-together Links only the "active" Link
    // of each Composite gets updated here.
    body_nodes_.reserve(ssize(forest.mobods()));
    for (const auto& mobod : forest.mobods()) {
      const MobodIndex node_index(mobod.index());
      const LinkIndex link_index = mobod.link();
      LinkTopology& current_link = links_[link_index];
      current_link.mobod_index = node_index;
      current_link.level = mobod.level();
      const MobodIndex mobilizer_index = current_link.inboard_mobilizer;
      DRAKE_DEMAND(mobilizer_index == node_index);
      const MobilizerTopology& mobilizer = mobilizers_[mobilizer_index];
      DRAKE_DEMAND(mobilizer.index == node_index);

      MobodIndex parent_node;
      const LinkIndex parent_link_index =
          current_link.parent_link;  // invalid if World
      if (node_index != 0) {         // Skip if World.
        parent_node = links_[parent_link_index].mobod_index;
        body_nodes_[parent_node].child_nodes.push_back(node_index);
      }

      // Creates a BodyNodeTopology.
      DRAKE_DEMAND(node_index == ssize(body_nodes_));
      body_nodes_.emplace_back(node_index, mobod.level(), parent_node,
                               link_index, parent_link_index);

      // Copy coordinate assignments from the MobilizerTopology.
      BodyNodeTopology& node = body_nodes_.back();
      node.mobilizer_positions_start = mobilizer.positions_start;
      node.num_mobilizer_positions = mobilizer.num_positions;
      node.num_mobilizer_velocities = mobilizer.num_velocities;
      node.mobilizer_velocities_start_in_v = mobilizer.velocities_start_in_v;
    }

    num_positions_ = forest.num_positions();
    num_velocities_ = forest.num_velocities();
    num_states_ = num_positions_ + num_velocities_;

    // Update position/velocity indexes for free bodies so that they are easily
    // accessible.
    for (LinkTopology& link : links_) {
      if (link.is_floating) {
        DRAKE_DEMAND(link.inboard_mobilizer.is_valid());
        const MobilizerTopology& mobilizer =
            get_mobilizer(link.inboard_mobilizer);
        link.floating_positions_start = mobilizer.positions_start;
        link.floating_velocities_start_in_v = mobilizer.velocities_start_in_v;
      }
    }

    ExtractForestInfo(graph);

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
  // connecting the Links with indexes `link1` and `link2`.
  bool IsThereAMobilizerBetweenLinks(
      LinkIndex link1, LinkIndex link2) const {
    for (const auto& mobilizer_topology : mobilizers_) {
      if (mobilizer_topology.connects_links(link1, link2)) return true;
    }
    return false;
  }

  // Helper method to be used within Finalize() to obtain the topological
  // information that describes the multibody system as a "forest" of trees.
  // TODO(sherm1) Currently this copies from the graph and forest into the
  //  previous data structures, to establish that we are computing the same
  //  quantities. Once this works, should switch to using the forest data
  //  directly and cut out the redundant data structures.
  void ExtractForestInfo(const LinkJointGraph& graph) {
    const SpanningForest& forest = graph.forest();
    forest_height_ = forest.height();

    const BodyNodeTopology& root = get_body_node(MobodIndex(0));
    DRAKE_DEMAND(ssize(root.child_nodes) == ssize(forest.trees()));

    tree_velocities_start_in_v_.resize(ssize(forest.trees()), -1);
    num_tree_velocities_.resize(ssize(forest.trees()), -1);
    for (const auto& tree : forest.trees()) {
      const TreeIndex index = tree.index();
      tree_velocities_start_in_v_.at(index) = tree.v_start();
      num_tree_velocities_[index] = tree.nv();
     }

    velocity_to_tree_index_.reserve(forest.num_velocities());
    for (int v = 0; v < forest.num_velocities(); ++v)
      velocity_to_tree_index_.emplace_back(forest.v_to_tree(v));

    // Map each Link (including shadows added to break loops) to the tree
    // to which its modeling Mobod belongs.
    link_to_tree_index_.resize(ssize(graph.links()));
    for (const auto& link : graph.links()) {
      const MobodIndex mobod_index = graph.link_to_mobod(link.index());
      DRAKE_DEMAND(mobod_index.is_valid());
      const SpanningForest::Mobod& mobod = forest.mobods(mobod_index);

      // The tree index will be invalid for World.
      link_to_tree_index_[link.index()] = mobod.tree();
    }
  }

  // is_valid is set to `true` after a successful Finalize().
  bool is_valid_{false};
  // Number of levels in the full Forest topology. After Finalize()
  // there will be at least one level (level = 0) with the world body.
  int forest_height_{-1};

  // Topological elements:
  std::vector<FrameTopology> frames_;
  std::vector<LinkTopology> links_;
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
  // t = link_to_tree_index_[b] is the tree index to which the b-th Link
  // belongs.
  std::vector<TreeIndex> link_to_tree_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
