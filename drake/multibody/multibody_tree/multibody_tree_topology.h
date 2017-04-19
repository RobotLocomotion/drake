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
///    compoments without having to create maps between the "original" and
///    "cloned" objects. That process is tedious and error prone.
///  - Each Multibody tree element has a copy (aquired at
///    MultibodyTree::Finalize() stage) of its topology which serves as a
///    key into the Context for that element's state.
///  - The topology is also stored in the Context so that the Multibody tree's
///    topology can be validated against the stored topology in debug builds.

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

#include <algorithm>
#include <utility>
#include <vector>

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

  // Unique index in the MultibodyTree.
  BodyIndex index{0};

  // Unique index to the frame associated with this body.
  FrameIndex body_frame{0};

  // Depth level in the MultibodyTree, level = 0 for the world.
  // Initialized to an invalid negative value.
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

  // Unique index in the MultibodyTree.
  FrameIndex index{0};

  // Unique index of the body this physical frame attaches to.
  BodyIndex body{0};
};

/// Data structure to store the topological information associated with an
/// entire MultibodyTree.
struct MultibodyTreeTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  /// Default constructor creates an empty, invalid topology. The minimum valid
  /// topology for a minimum valid MultibodyTree containts at least the
  /// BodyTopology for the world. The topology for the _world_ body does not get
  /// added but until MultibodyTree construction, which creates a _world_ body
  /// and adds it to the tree.
  MultibodyTreeTopology() {}

  /// Returns the number of bodies in the multibody tree.
  int get_num_bodies() const { return static_cast<int>(bodies.size()); }

  /// Returns the number of physical frames in the multibody tree.
  int get_num_frames() const {
    return static_cast<int>(frames.size());
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

  bool is_valid{false};

  /// Topology gets validated by MultibodyTree::Finalize().
  void set_valid() { is_valid = true; }

  std::vector<BodyTopology> bodies;
  std::vector<FrameTopology> frames;
};

}  // namespace multibody
}  // namespace drake
