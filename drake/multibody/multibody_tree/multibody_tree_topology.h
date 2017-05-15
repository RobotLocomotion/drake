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

  /// Unique index in the MultibodyTree.
  BodyIndex index{0};

  /// Unique index to the one and only inboard mobilizer a body can have.
  /// By default this is left initialized to "invalid" so that we can detect
  /// graph loops whithin add_mobilizer().
  MobilizerIndex inboard_mobilizer{};

  /// Unique index to the frame associated with this body.
  FrameIndex body_frame{0};

  /// Depth level in the MultibodyTree, level = 0 for the world.
  /// Initialized to an invalid negative value.
  int level{-1};
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
  MobilizerTopology(
      MobilizerIndex mobilizer_index,
      FrameIndex in_frame, FrameIndex out_frame,
      BodyIndex in_body, BodyIndex out_body) :
      index(mobilizer_index),
      inboard_frame(in_frame), outboard_frame(out_frame),
      inboard_body(in_body), outboard_body(out_body) {}

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

  /// Mobilizer indexing info: Set at Finalize() time.
  // Number of generalized coordinates granted by this mobilizer.
  int num_positions{0};
  // First entry in the global array of generalized coordinates for the parent
  // MultibodyTree.
  int positions_start{0};
  // Number of generalized velocities granted by this mobilizer.
  int num_velocities{0};
  // First entry in the global array of generalized velocities for the parent
  // MultibodyTree.
  int velocities_start{0};
};

/// Data structure to store the topological information associated with an
/// entire MultibodyTree.
struct MultibodyTreeTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  /// Default constructor creates an empty, invalid topology. The minimum valid
  /// topology for a minimum valid MultibodyTree containts at least the
  /// BodyTopology for the world. The topology for the _world_ body does not get
  /// added until MultibodyTree construction, which creates a _world_ body
  /// and adds it to the tree.
  MultibodyTreeTopology() {}

  /// Returns the number of bodies in the multibody tree. This includes the
  /// "world" body and therefore the minimum number of bodies after
  /// MultibodyTree::Finalize() will always be one, not zero.
  int get_num_bodies() const { return static_cast<int>(bodies.size()); }

  /// Returns the number of physical frames in the multibody tree.
  int get_num_frames() const {
    return static_cast<int>(frames.size());
  }

  /// Returns the number of mobilizers in the multibody tree. Since the "world"
  /// body does not have a mobilizer, the number of mobilizers will always equal
  /// the number of bodies minus one.
  int get_num_mobilizers() const {
    return static_cast<int>(mobilizers.size());
  }

  /// Creates and adds a new BodyTopology to this MultibodyTreeTopology.
  /// The BodyTopology will be assigned a new, unique BodyIndex and FrameIndex
  /// values.
  /// @returns a std::pair<BodyIndex, FrameIndex> containing the indexes
  /// assigned to the new BodyTopology.
  std::pair<BodyIndex, FrameIndex> add_body() {
    BodyIndex body_index = BodyIndex(get_num_bodies());
    FrameIndex body_frame_index = add_frame(body_index);
    bodies.emplace_back(body_index, body_frame_index);
    return std::make_pair(body_index, body_frame_index);
  }

  /// Creates and adds a new FrameTopology, associated with the given
  /// body_index, to this MultibodyTreeTopology.
  /// @returns The FrameIndex assigned to the new FrameTopology.
  FrameIndex add_frame(BodyIndex body_index) {
    FrameIndex frame_index(get_num_frames());
    frames.emplace_back(frame_index, body_index);
    return frame_index;
  }

  /// Returns a constant reference to the corresponding FrameTopology given the
  /// FrameIndex.
  const FrameTopology& get_frame(FrameIndex index) const {
    DRAKE_ASSERT(index < get_num_frames());
    return frames[index];
  }

  /// Creates and adds a new MobilizerTopology connecting the inboard and
  /// outboard multibody frames identified by indexes `in_frame` and
  /// `out_frame`, respectively.
  /// @returns The MobilizerIndex assigned to the new MobilizerTopology.
  /// @throws std::runtime_error if either `in_frame` or `out_frame` do not
  /// index frame topologies in `this` %MultibodyTreeTopology.
  /// @throws a std::runtime_error if `in_frame == out_frame`.
  /// @throws a std::runtime_error if `in_frame` and `out_frame` already are
  /// connected by another mobilizer. More than one mobilizer between two frames
  /// is not allowed.
  MobilizerIndex add_mobilizer(
      FrameIndex in_frame, FrameIndex out_frame) {
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
    const BodyIndex inboard_body = frames[in_frame].body;
    const BodyIndex outboard_body = frames[out_frame].body;
    if (IsThereAMobilizerBetweenBodies(inboard_body, outboard_body)) {
      throw std::runtime_error(
          "This multibody tree already has a mobilizer connecting these two "
          "bodies. More than one mobilizer between two bodies is not allowed");
    }
    // Checks for graph loops. Each body can have and only one inboard
    // mobilizer.
    if (bodies[outboard_body].inboard_mobilizer.is_valid()) {
      throw std::runtime_error(
          "This mobilizer is creating a closed loop since the outboard body "
          "already has an inboard mobilizer connected to it. "
          "If a physical loop is really needed, consider using a constraint "
          "instead.");
    }
    MobilizerIndex mobilizer_index(get_num_mobilizers());

    // Make note of the inboard mobilizer for the outboard body.
    bodies[outboard_body].inboard_mobilizer = mobilizer_index;

    mobilizers.emplace_back(mobilizer_index,
                            in_frame, out_frame,
                            inboard_body, outboard_body);
    return mobilizer_index;
  }

  bool is_valid{false};

  /// Topology gets validated by MultibodyTree::Finalize().
  void set_valid() { is_valid = true; }

  /// Returns `true` if there is _any_ mobilizer in the multibody tree
  /// connecting the frames with indexes `frame` and `frame2`.
  bool IsThereAMobilizerBetweenFrames(FrameIndex frame1, FrameIndex frame2) {
    for (const auto& mobilizer_topology : mobilizers) {
      if (mobilizer_topology.connects_frames(frame1, frame2)) return true;
    }
    return false;
  }

  /// Returns `true` if there is _any_ mobilizer in the multibody tree
  /// connecting the bodies with indexes `body2` and `body2`.
  bool IsThereAMobilizerBetweenBodies(BodyIndex body1, BodyIndex body2) {
    for (const auto& mobilizer_topology : mobilizers) {
      if (mobilizer_topology.connects_bodies(body1, body2)) return true;
    }
    return false;
  }

  std::vector<BodyTopology> bodies;
  std::vector<FrameTopology> frames;
  std::vector<MobilizerTopology> mobilizers;
};

}  // namespace multibody
}  // namespace drake
