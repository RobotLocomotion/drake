#pragma once

/// @file
/// This file defines the topological structures which represent the logical
/// connectivities between multibody tree elements. For instance, the
/// BodyTopology for a Body will contain the topological information specifying
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
    if (mobilized_body.is_valid() != other.mobilized_body.is_valid())
      return false;
    if (mobilized_body.is_valid() && mobilized_body != other.mobilized_body)
      return false;
    if (inboard_body.is_valid() != other.inboard_body.is_valid()) return false;
    if (inboard_body.is_valid() && inboard_body != other.inboard_body)
      return false;
    if (child_bodies != other.child_bodies) return false;
    if (body_frame != other.body_frame) return false;
    if (level != other.level) return false;
    if (is_floating != other.is_floating) return false;
    if (has_quaternion_dofs != other.has_quaternion_dofs) return false;
    if (floating_positions_start != other.floating_positions_start)
      return false;
    if (floating_velocities_start != other.floating_velocities_start)
      return false;
    return true;
  }

  // Unique index in the MultibodyPlant (and MultibodyTree).
  BodyIndex index{0};

  // The MobilizedBody assigned to this Body. The associated mobilizer connects
  // this Body as the outboard body to the inboard body indicated below.
  // By default this is initialized to "invalid" so that we can detect
  // graph loops within add_mobilizer().
  // This will remain "invalid" for the world body.
  MobilizedBodyIndex mobilized_body{};

  // Within the tree structure of a MultibodyTree, the immediate inboard body
  // connected by the MobilizedBody indexed by `mobilized_body`.
  // By default this is initialized to "invalid" so that we can assert
  // (from within add_mobilizer()) that each body can have only one parent
  // body. Also, this will remain "invalid" for the world body.
  BodyIndex inboard_body{};

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

  // `true` if this topology corresponds to a floating body in space.
  bool is_floating{false};

  // `true` if this topology corresponds to a body with rotations parametrized
  // by a quaternion.
  bool has_quaternion_dofs{false};

  int floating_positions_start{-1};
  int floating_velocities_start{-1};
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
// MobilizedBody object. It stores:
//
// - Indexes to the inboard/outboard frames of this mobilizer.
// - Indexes to the inboard/outboard bodies of this mobilizer.
// - Numbers of dofs provided by this mobilizer.
// - Indexing information to retrieve entries from the parent MultibodyTree
//   Context.
//
// Additional information on topology classes is given in this file's
// documentation at the top.
struct MobilizedBodyTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MobilizedBodyTopology);

  // Default construction to invalid configuration.
  MobilizedBodyTopology() {}

  // Constructs a %MobilizedBodyTopology for a particular MobilizedBody. A
  // mobilized body contains all the computational information necessary for
  // performing operations involving mobilizers and their associated bodies.
  //
  // @param construction_index
  //     The unique index of this MobilizedBody, in construction order. We will
  //     reorder later for efficient computation.
  // @param in_frame   The F frame fixed to the inboard (parent) body P.
  // @param out_frame  The M frame fixed to the outboard (child) body B.
  // @param in_body    The inboard (parent) body P.
  // @param out_body   The outboard (child) body B.
  // @param num_position_in
  //     The number nq of generalized position coordinates q used by this
  //     mobilizer.
  // @param num_velocities_in
  //     The number nv of generalized velocity coordinates v used by this
  //     mobilizer. This is necessarily equal to the number of degrees of
  //     freedom provided by the mobilizer.
  MobilizedBodyTopology(MobilizedBodyIndex construction_index_in,
                        FrameIndex in_frame, FrameIndex out_frame,
                        BodyIndex in_body, BodyIndex out_body,
                        int num_positions_in, int num_velocities_in)
      : construction_index(construction_index_in),
        body(out_body), frame(out_frame),
        inboard_body(in_body), inboard_frame(in_frame),
        num_positions(num_positions_in), num_velocities(num_velocities_in) {}

  // Returns `true` if this %MobilizedBodyTopology connects frames identified by
  // indexes `frame1` and `frame2`.
  bool connects_frames(FrameIndex frame1, FrameIndex frame2) const {
    return (inboard_frame == frame1 && frame == frame2) ||
           (inboard_frame == frame2 && frame == frame1);
  }

  // Returns `true` if this %MobilizedBodyTopology connects bodies identified by
  // indexes `body1` and `body2`.
  bool connects_bodies(BodyIndex body1, BodyIndex body2) const {
    return (inboard_body == body1 && body == body2) ||
           (inboard_body == body2 && body == body1);
  }

  // Returns `true` if this is the World "mobilized" body, that is, `body` is
  // World (in which case our construction_index (and dft_index) is zero).
  bool is_world() const { return construction_index == 0; }

  // Returns `true` if this mobilizer topology corresponds to that of a weld
  // mobilizer. This will also be true if `this` MobilizedBody is World.
  // (You can think of World as being welded to the universe.)
  bool is_weld_mobilizer() const {
    return num_velocities == 0;
  }

  // Returns the number of outboard bodies for which this body is their
  // inboard body.
  int get_num_children() const {
    return static_cast<int>(outboard_mobilized_bodies.size());
  }

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MobilizedBodyTopology& other) const {
    if (construction_index != other.construction_index) return false;
    if (body != other.body) return false;
    if (frame != other.frame) return false;

    if (construction_index != MobilizedBodyIndex(0)) {
      if (inboard_body != other.inboard_body) return false;
      if (inboard_frame != other.inboard_frame) return false;
      if (inboard_mobilized_body != other.inboard_mobilized_body) return false;
    }

    if (num_positions != other.num_positions) return false;
    if (num_velocities != other.num_velocities) return false;

    if (positions_start != other.positions_start) return false;
    if (velocities_start != other.velocities_start) return false;
    if (velocities_start_in_v != other.velocities_start_in_v) return false;

    if (dft_index != other.dft_index) return false;
    if (level != other.level) return false;
    if (outboard_mobilized_bodies != other.outboard_mobilized_bodies)
      return false;

    return true;
  }

  // TODO(sherm1) Could avoid copying the std::vector member by spelling
  //  out swap member by member.
  void swap(MobilizedBodyTopology& other) {
      const MobilizedBodyTopology temp(*this);
      *this = other;
      other = temp;
  }

  /*----- Set at construction -----*/

  MobilizedBodyIndex construction_index;  // In as-constructed order.
  BodyIndex body;    // Index of the body B represented by this mobilized body.
  FrameIndex frame;  // Index of the mobilizer frame M fixed to B.

  // These two are valid unless this is the World MobilizedBody (index==0).
  BodyIndex inboard_body;    // Index of the inboard (parent) body P.
  FrameIndex inboard_frame;  // Index of the mobilizer frame F fixed to P.

  // Number of generalized coordinates q employed by this mobilizer.
  int num_positions{0};
  // Number of generalized velocities (and dofs) v provided by this mobilizer.
  int num_velocities{0};

  /*----- Set during Finalize() -----*/

  MobilizedBodyIndex dft_index;  // Index after depth-first reordering.

  // Depth level of this mobilized body in the MultibodyTree. World serves as
  // the root and has level==0. The mobilized body level is defined to be the
  // level of the `body` it mobilizes.
  int level{-1};

  // The unique inboard ("parent") mobilized body of `this` in the
  // MultibodyTree. The level of the inboard body is always one less than our
  // level. Valid for every mobilized body except World.
  MobilizedBodyIndex inboard_mobilized_body;

  // The outboard mobilized bodies for which this serves as their
  // inboard mobilized body.
  std::vector<MobilizedBodyIndex> outboard_mobilized_bodies;

  // First entry in the global array of states, `x = [q v z]`, for the parent
  // MultibodyTree.
  int positions_start{0};

  // First entry in the global array of states, `x = [q v z]`, for the parent
  // MultibodyTree.
  int velocities_start{0};

  // Start index in a vector containing only generalized velocities.
  // It is also a valid index into a vector of generalized accelerations (which
  // are the time derivatives of the generalized velocities) and into a vector
  // of generalized forces tau.
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
  // entire model.
  int actuator_index_start{-1};
  // The number of dofs actuated by this actuator.
  int num_dofs{-1};
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
    if (tree_height_ != other.tree_height_) return false;

    if (num_positions_ != other.num_positions_) return false;
    if (num_velocities_ != other.num_velocities_) return false;
    if (num_states_ != other.num_states_) return false;

    if (bodies_ != other.bodies_) return false;
    if (frames_ != other.frames_) return false;
    if (mobilized_bodies_ != other.mobilized_bodies_) return false;
    if (force_elements_ != other.force_elements_) return false;
    if (joint_actuators_ != other.joint_actuators_) return false;

    return true;
  }

  // Returns the number of bodies in the multibody tree. This includes the
  // "world" body and therefore the minimum number of bodies after
  // MultibodyTree::Finalize() will always be one, not zero.
  int num_bodies() const { return static_cast<int>(bodies_.size()); }

  // Returns the number of physical frames in the multibody tree.
  int num_frames() const {
    return static_cast<int>(frames_.size());
  }

  // Returns the number of mobilized bodies in the multibody tree. World is
  // considered a "mobilized" body, so there is always at least one.
  int num_mobilized_bodies() const {
    return static_cast<int>(mobilized_bodies_.size());
  }

  // Returns the number of force elements in the topology.
  int num_force_elements() const {
    return static_cast<int>(force_elements_.size());
  }

  // Returns the number of joint actuators in the topology.
  int num_joint_actuators() const {
    return static_cast<int>(joint_actuators_.size());
  }

  // Returns the number of tree levels in the topology.
  int tree_height() const {
    return tree_height_;
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

  // Returns a const reference to the corresponding MobilizedBodyTopology
  // given a MobilizedBodyIndex.
  const MobilizedBodyTopology& get_mobilized_body(
      MobilizedBodyIndex index) const {
    DRAKE_ASSERT(index < num_mobilized_bodies());
    const MobilizedBodyTopology& mobod = mobilized_bodies_[index];
    DRAKE_ASSERT(mobod.dft_index.is_valid() && mobod.dft_index == index ||
                 !mobod.dft_index.is_valid() &&
                     mobod.construction_index == index);
    return mobod;
  }

  MobilizedBodyTopology& get_mutable_mobilized_body(
            MobilizedBodyIndex index) {
    return const_cast<MobilizedBodyTopology&>(get_mobilized_body(index));
  }

  // Returns a const reference to the MobilizedBodyTopology for the inboard
  // mobilized body of the given mobilized body.
  // @pre index != 0 (World doesn't have a parent)
  const MobilizedBodyTopology& get_inboard_mobilized_body(
      MobilizedBodyIndex index) const {
    DRAKE_ASSERT(index > 0);  // World doesn't have an inboard body.
    const MobilizedBodyTopology& mobod = get_mobilized_body(index);
    DRAKE_ASSERT(mobod.inboard_mobilized_body.is_valid());
    const MobilizedBodyTopology& inboard_mobod =
        get_mobilized_body(mobod.inboard_mobilized_body);
    // If post-finalize then the levels must be right.
    DRAKE_ASSERT(!mobod.dft_index.is_valid() ||
            inboard_mobod.level == mobod.level-1);
  }

  // Returns a const reference to the corresponding JointActuatorTopology
  // given a JointActuatorIndex.
  const JointActuatorTopology& get_joint_actuator(
      JointActuatorIndex index) const {
    DRAKE_ASSERT(index < num_joint_actuators());
    return joint_actuators_[index];
  }

  // Returns the number of trees in the "forest" topology of the entire system.
  // We refer to as "tree" a subgraph in the topology having a tree structure
  // and whose base mobilized body connects to the world. The world does not
  // belong to any tree. In other words, the number of trees in the topology
  // corresponds to the number of children of the world Body (also called
  // "base bodies").
  int num_trees() const {
    return static_cast<int>(num_tree_velocities_.size());
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
  int tree_velocities_start(TreeIndex t) const {
    DRAKE_ASSERT(t < num_trees());
    return tree_velocities_start_[t];
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

  // Creates and adds a new MobilizedBodyTopology connecting the inboard and
  // outboard multibody frames identified by indexes `in_frame` and
  // `out_frame`, respectively. The created topology will correspond to that of
  // a MobilizedBody with `num_positions` and `num_velocities`.
  //
  // Note that construction doesn't enforce a tree ordering since it only
  // requires that the inboard & outboard bodies exist, but not that the
  // inboard body is already part of the tree. We'll be reordering later.
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
  // Does not return the "construction index" to avoid confusion, since we will
  // reorder these in depth-first order later.
  void add_mobilizer(
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
    if (bodies_[outboard_body].mobilized_body.is_valid()) {
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
    // BodyTopology::mobilized_body and BodyTopology::inboard_body are both
    // set within this method right after these checks.
    DRAKE_DEMAND(!bodies_[outboard_body].mobilized_body.is_valid());
    DRAKE_DEMAND(!bodies_[outboard_body].inboard_body.is_valid());

    // This index gives us the construction order, but won't be the final
    // ordering we use for computation.
    const MobilizedBodyIndex construction_index(num_mobilized_bodies());

    // Make note of the inboard mobilizer for the outboard body.
    bodies_[outboard_body].mobilized_body = construction_index;
    // Similarly, record inboard_body as the parent of outboard_body.
    bodies_[outboard_body].inboard_body = inboard_body;

    // Records "child" bodies for bookkeeping in the context of the tree
    // structure of MultibodyTree.
    bodies_[inboard_body].child_bodies.push_back(outboard_body);

    mobilized_bodies_.emplace_back(construction_index,
                                   in_frame, out_frame,
                                   inboard_body, outboard_body,
                                   num_positions, num_velocities);
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
  // - sorting the MobilizedBodies in depth-first order for fast recursions
  //     through the tree,
  // - assigning generalized coordinates and speeds as needed by each
  //     MobilizedBody in the same depth-first order,
  // - computation of state sizes and of pool sizes within cache entries,
  // - computation of index maps to retrieve either state or cache entries for
  //     each multibody element.
  //
  // If the finalize stage is successful, `this` topology is validated,
  // meaning it is up-to-date after this call. No more multibody plant or tree
  // elements can be added after a call to Finalize().
  //
  // @throws std::exception If users attempt to call this method on an
  //         already finalized topology.
  // @see is_valid()
  void Finalize() {
    // If the topology is valid it means that it was already finalized.
    // Re-finalization is not allowed.
    if (is_valid()) {
      throw std::logic_error(
          "Attempting to call MultibodyTree::Finalize() on an already "
          "finalized MultibodyTree.");
    }

    // Initially, MobilizedBodies are ordered as they were defined. We'll
    // calculate the desired depth-first order and
    // record the depth-first ordering in the `index` field of each
    // MobilizedBody. Then we'll re-order our vector of MobilizedBodyTopology
    // objects to match. MultibodyTree can then reorder the actual
    // MobilizedBodies the same way.

    // For each MobilizedBody, assign an index in a depth first traversal order.
    std::stack<MobilizedBodyIndex> stack;
    stack.push(MobilizedBodyIndex(0));  // Starts at the root.
    tree_height_ = 1;  // At least one level with the world body at the root.
    int next = 0;  // The DFT index for the next mobod.
    while (!stack.empty()) {
      const MobilizedBodyIndex current = stack.top();
      MobilizedBodyTopology& current_mobod = mobilized_bodies_[current];
      current_mobod.index = MobilizedBodyIndex(next);
      const MobilizedBodyIndex inboard =  // Invalid for World.
              current_mobod.inboard_mobilized_body;

      const int level = current == 0 ? 0 : mobilized_bodies_[inboard].level + 1;
      current_mobod.level = level;

      // Track # levels, i.e. length of the longest branch including World.
      tree_height_ = std::max(tree_height_, level + 1);

      // We process bodies in the order they were added to the vector of child
      // bodies; this vector is filled in the order mobilizers are added to the
      // model. Therefore, when a given node branches out, we spawn branches in
      // the order mobilizers that connect this node to its children were added.
      // Since we are using a stack to store bodies that will be processed next,
      // we must place bodies in reverse order so that the first child is at the
      // top of the stack.
      stack.pop();  // We're done with this one.
      for (auto it = current_mobod.outboard_mobilized_bodies.rbegin();
           it != current_mobod.outboard_mobilized_bodies.rend(); ++it) {
        stack.push(*it);
      }
    }

    // Now reorder the mobilized body topology entries to match their indexes
    // as set above. This is an O(n) pass since we put one in the right place
    // each iteration. Skip World since we know it is in the right place.
    for (MobilizedBodyIndex i(1); i < num_mobilized_bodies(); ++i) {
        MobilizedBodyTopology& mobod = mobilized_bodies_[i];
        while (mobod.index != i) {
            MobilizedBodyTopology& other = mobilized_bodies_[mobod.index];
            std::swap(mobod, other);
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

    // Compile information regarding the size of the system:
    // - Number of degrees of freedom (generalized positions and velocities).
    // - Start/end indexes for each node.
    //
    // Base-to-Tip loop in DFT order, skipping the world (mobod 0).

    // Count number of generalized positions and velocities.
    num_positions_ = 0;
    num_velocities_ = 0;
    for (const auto& mobilizer : mobilized_bodies_) {
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
      MobilizedBodyTopology& mobilizer = mobilized_bodies_[node.mobilizer];

      if (mobilizer.num_velocities == 0) {  // A weld mobilizer.
        // For weld mobilizers start indexes are not important since the number
        // of dofs is zero. However, we do allow accessing Eigen segments with
        // zero size and for that case Eigen enforces start >= zero.
        // Therefore, these start indexes are set to zero to allow zero sized
        // indexes without having to introduce any special logic for weld
        // mobilizers.
        mobilizer.positions_start = 0;
        mobilizer.velocities_start = 0;
        mobilizer.velocities_start_in_v = 0;
      } else {
        mobilizer.positions_start = position_index;
        mobilizer.velocities_start = velocity_index;
        mobilizer.velocities_start_in_v = velocity_index - num_positions_;
        DRAKE_DEMAND(0 <= mobilizer.velocities_start_in_v);
      }

      position_index += mobilizer.num_positions;
      velocity_index += mobilizer.num_velocities;

      node.mobilizer_positions_start = mobilizer.positions_start;
      node.num_mobilizer_positions = mobilizer.num_positions;
      node.mobilizer_velocities_start = mobilizer.velocities_start;
      node.num_mobilizer_velocities = mobilizer.num_velocities;

      // Start index in a vector containing only generalized velocities.
      node.mobilizer_velocities_start_in_v = mobilizer.velocities_start_in_v;
      // Demand indexes to be positive only for mobilizers with a non-zero
      // number of dofs.
      DRAKE_DEMAND(0 <= node.mobilizer_velocities_start_in_v ||
          node.num_mobilizer_velocities == 0);
      // This test would not pass for a model with all weld joints. Therefore
      // the check for num_velocities_ == 0.
      DRAKE_DEMAND(node.mobilizer_velocities_start_in_v < num_velocities_ ||
          num_velocities_ == 0);
    }
    DRAKE_DEMAND(position_index == num_positions_);
    DRAKE_DEMAND(velocity_index == num_states_);

    // Update position/velocity indexes for free bodies so that they are easily
    // accessible.
    for (BodyTopology& body : bodies_) {
      if (body.is_floating) {
        DRAKE_DEMAND(body.mobilized_body.is_valid());
        const MobilizedBodyTopology& mobilizer =
                get_mobilized_body(body.mobilized_body);
        body.floating_positions_start = mobilizer.positions_start;
        body.floating_velocities_start = mobilizer.velocities_start;
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

  // Given a MobilizedBody in `this` topology, specified by its
  // MobilizedBodyIndex `from`, computes the kinematic path formed by all the
  // mobilized bodies in the tree that connect `from` with the root
  // (corresponding to World).
  //
  // @param[in] from
  //   A mobilized body in the tree topology to which the path to the root
  //   (World) is to be computed.
  // @param[out] path_to_world
  //   A std::vector of mobilized body indices that on output will contain the
  //   path to the root of the tree, _including_ the root (World) mobilized
  //   body. Forward iteration (from element 0 to element size()-1) of
  //   `path_to_world` will traverse all mobilized bodies in the tree
  //   starting at World along the path to `from`. That is, forward
  //   iteration starts with the root of the tree at `path_to_world[0]` and
  //   ends with `from` at `path_to_world.back()`.
  //   On input, `path_to_world` must be a valid pointer. On output this vector
  //   (resized if needed) will contain `from`.level+1 entries so that we
  //   can include World in the path.
  void GetKinematicPathToWorld(
      MobilizedBodyIndex from,
      std::vector<MobilizedBodyIndex>* path_to_world) const {
    DRAKE_THROW_UNLESS(path_to_world != nullptr);

    const int path_size = get_mobilized_body(from).level + 1;
    path_to_world->resize(path_size);
    (*path_to_world)[0] = MobilizedBodyIndex(0);  // Add the world.
    if (from == MobilizedBodyIndex(0)) return;

    // Navigate the tree inwards starting at `from` and ending at World.
    for (const MobilizedBodyTopology* mobod = &get_mobilized_body(from);
         !mobod->is_world(); mobod = &get_inboard_mobilized_body(from)) {
      (*path_to_world)[mobod->level] = mobod->index;
    }
    // Verify the last added node to the path is a child of the world.
    DRAKE_DEMAND(get_mobilized_body((*path_to_world)[1]).level == 1);
  }

  // Returns `true` if the body with index `body_index` is anchored to the
  // world. A body is said to be "anchored" if its kinematics path to the world
  // only contains weld mobilizers. The complexity of this operation is
  // O(level), where "level" refers to the depth in the tree of the body node
  // associated with `body_index`.
  bool IsBodyAnchored(BodyIndex body_index) const {
    DRAKE_DEMAND(is_valid());
    const BodyTopology& body = get_body(body_index);
    std::vector<MobilizedBodyIndex> path_to_world;
    GetKinematicPathToWorld(body.mobilized_body, &path_to_world);
    // Skip the world at path_to_world[0].
    for (size_t path_index = 1; path_index < path_to_world.size();
         ++path_index) {
      const MobilizedBodyTopology& mobod =
          get_mobilized_body(path_to_world[path_index]);
      // If any of the mobilizers in the path is not a weld mobilizer, the body
      // is not anchored.
      if (!mobod.is_weld_mobilizer()) return false;
    }
    // If the loop above completes, then body_index is anchored to the world.
    return true;
  }

  // This method partitions the tree topology into sub-graphs such that two
  // bodies are in the same sub-graph if there is a path between them which
  // includes only weld mobilizers.
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
      const MobilizedBodyTopology& base) const {
    return CalcNumberOfOutboardVelocities(base) - base.num_velocities;
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
    auto collect_body =
        [&outboard_bodies](const MobilizedBodyTopology& node) {
      outboard_bodies.insert(node.body);
    };
    for (const BodyIndex& body_index : body_indexes) {
      DRAKE_DEMAND(body_index.is_valid() && body_index < num_bodies());
      // Skip bodies that are already traversed because the subtree with it
      // being the root has necessarily been traversed already.
      if (outboard_bodies.count(body_index) == 0) {
        const MobilizedBodyTopology& root =
            get_mobilized_body(get_body(body_index).mobilized_body);
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
    for (const auto& mobilizer_topology : mobilized_bodies_) {
      if (mobilizer_topology.connects_frames(frame1, frame2)) return true;
    }
    return false;
  }

  // Returns `true` if there is _any_ mobilizer in the multibody tree
  // connecting the bodies with indexes `body2` and `body2`.
  bool IsThereAMobilizerBetweenBodies(
      BodyIndex body1, BodyIndex body2) const {
    for (const auto& mobilizer_topology : mobilized_bodies_) {
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
      BodyIndex parent_index, std::set<BodyIndex>* parent_welded_body,
      std::vector<std::set<BodyIndex>>* welded_bodies) const {
    const BodyTopology& parent = get_body(parent_index);
    for (BodyIndex child_index : parent.child_bodies) {
      const BodyTopology& child = get_body(child_index);
      const MobilizedBodyTopology& child_mobilizer =
              get_mobilized_body(child.mobilized_body);
      if (child_mobilizer.is_weld_mobilizer()) {
        // If the child body is welded to the parent body, we then add it to
        // the parent's welded body set, parent_welded_body. We continue the
        // recursion down the tree starting at child.
        parent_welded_body->insert(child_index);
        CreateListOfWeldedBodiesRecurse(child_index, parent_welded_body,
                                        welded_bodies);
      } else {
        // If the child body is not welded to the parent body, then we create a
        // new welded body set in which child is the only member. We continue
        // the recursion down the tree starting at child.
        welded_bodies->push_back(std::set<BodyIndex>{child_index});
        std::set<BodyIndex>& child_group = welded_bodies->back();
        CreateListOfWeldedBodiesRecurse(child_index, &child_group,
                                        welded_bodies);
      }
    }
  }

  // This traverses the tree of mobilized bodies outboard of `base` and applies
  // `operation` on each of them, starting with `base`. The traversal is
  // performed in depth first order.
  // @pre Mobilized bodies were already created and are indexed in depth first
  // order.
  void TraverseOutboardNodes(
      const MobilizedBodyTopology& base,
      std::function<void(const MobilizedBodyTopology&)> operation) const {
    DRAKE_DEMAND(get_num_mobilized_bodies() != 0);
    operation(base);
    // We are done if the base has no more children.
    if (base.get_num_children() == 0) return;
    // Traverse outboard nodes. Since the tree is finalized, we know nodes are
    // in DFT order.
    const int base_level = base.level;
    for (BodyNodeIndex node_index(base.index + 1);
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
    DRAKE_DEMAND(get_num_body_nodes() != 0);
    int nv = 0;
    TraverseOutboardNodes(base, [&nv](const BodyNodeTopology& node) {
      nv += node.num_mobilizer_velocities;
    });
    return nv;
  }

  // Helper method to be used within Finalize() to obtain the topological
  // information that describes the multibody system as a "forest" of trees.
  void ExtractForestInfo() {
    const BodyNodeTopology& root = get_body_node(BodyNodeIndex(0));
    const int max_num_trees = root.child_nodes.size();
    num_tree_velocities_.reserve(max_num_trees);
    body_to_tree_index_.resize(num_bodies());
    velocity_to_tree_index_.resize(num_velocities());

    for (const BodyNodeIndex& root_child_index : root.child_nodes) {
      const BodyNodeTopology& root_child = get_body_node(root_child_index);
      const int nt = CalcNumberOfOutboardVelocities(root_child);
      if (nt > 0) {
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
    }

    // N.B. For trees with no generalized velocities, this code sets
    // tree_velocities_start_[t] to point to the last dof (plus one) of the last
    // tree with non-zero velocities. The reason to do so is that we want users
    // of MultibodyTreeTopology to write code like so:
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
    tree_velocities_start_.resize(num_trees(), 0);
    for (int t = 1; t < num_trees(); ++t) {
      tree_velocities_start_[t] =
          tree_velocities_start_[t - 1] + num_tree_velocities_[t - 1];
    }
  }

  // is_valid is set to `true` after a successful Finalize().
  bool is_valid_{false};
  // Number of levels (or generations) in the tree topology. After Finalize()
  // there will be at least one level (level = 0) with the world body.
  int tree_height_{-1};

  // Topological elements:
  std::vector<FrameTopology> frames_;
  std::vector<BodyTopology> bodies_;
  std::vector<MobilizedBodyTopology> mobilized_bodies_;
  std::vector<ForceElementTopology> force_elements_;
  std::vector<JointActuatorTopology> joint_actuators_;

  // Total number of generalized positions and velocities in the MultibodyTree
  // model.
  int num_positions_{0};
  int num_velocities_{0};
  int num_states_{0};
  int num_actuated_dofs_{0};

  // Number of generalized velocities for the t-th tree.
  std::vector<int> num_tree_velocities_;
  // Given the generalized velocities vector v for the entire model, the vector
  // vt = {v(m) s.t. m ∈ [mₛ, mₑ)}, with mₛ = tree_velocities_start_[t] and iₑ =
  // tree_velocities_start_[t] + num_tree_velocities_[t], are the generalized
  // velocities for the t-th tree.
  std::vector<int> tree_velocities_start_;
  // t = velocity_to_tree_index_[m] is the tree index to which the m-th velocity
  // belongs.
  std::vector<TreeIndex> velocity_to_tree_index_;
  // t = body_to_tree_index_[b] is the tree index to which the b-th body
  // belongs.
  std::vector<TreeIndex> body_to_tree_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
