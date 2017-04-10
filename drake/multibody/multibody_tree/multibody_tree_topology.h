#pragma once

/// @file
/// This file defines the topological structures to represent the logical
/// connectivities between multibody tree components. For instance, the
/// BodyTopology for a Body will contain the topological information specifying
/// its inboard body (or parent body) in the parent tree, its outboard bodies
/// (or children) and the level or depth in the MultibodyTree.
/// All of this information is independent of the particular scalar type T the
/// MultibodyTree and its components are instantiated with.
/// All of the data structures defined in this file are meant to be the most
/// minimalist representation that can store this information.
/// All of these data structures are meant to be copiable to aid the process of
/// cloning or transmogrifying multibody tree compoments.

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

  // Default construction with invalid initialization.
  BodyTopology() {}

  // Constructs a body topology struct with unique index `body_index` and a
  // body frame with unique index `frame_index`.
  BodyTopology(BodyIndex body_index, FrameIndex frame_index) :
      index(body_index), body_frame(frame_index) {}

  // Unique id in the MultibodyTree.
  BodyIndex index{0};

  // Body frame.
  FrameIndex body_frame{0};

  // Depth level in the MultibodyTree, level = 0 for the world.
  // Initialized to an invalid negative value.
  int level{-1};
};

/// Data structure to store the topological information associated with a
/// Frame.
struct FrameTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameTopology);

  // Default construction with invalid initialization.
  FrameTopology() {}

  /// Constructs a frame topology for a frame with unique index `frame_index`
  /// associated with a body with index `body_index`.
  FrameTopology(FrameIndex frame_index, BodyIndex body_index) :
      index(frame_index), body(body_index) {}

  // Unique identifier in the MultibodyTree.
  FrameIndex index{0};

  // Unique identifier of the body this physical frame attaches to.
  BodyIndex body{0};
};

/// Data structure to store the topological information associated with an
/// entire MultibodyTree.
struct MultibodyTreeTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  // Default constructor creates an empty, invalid, topology.
  MultibodyTreeTopology() {}

  // Returns the number of bodies in the multibody tree.
  int get_num_bodies() const {return static_cast<int>(bodies.size()); }

  // Returns the number of physical frames in the multibody tree.
  int get_num_frames() const {
    return static_cast<int>(frames.size());
  }

  // Creates and adds a new BodyTopology to this MultibodyTreeTopology.
  // A unique index is assigned to the newly created BodyTopology and a new
  // FrameTopology is created and associated to the body topology.
  std::pair<BodyIndex, FrameIndex> add_body() {
    BodyIndex body_index = BodyIndex(get_num_bodies());
    FrameIndex body_frame_index = add_frame(body_index);
    bodies.emplace_back(body_index, body_frame_index);
    return std::make_pair(body_index, body_frame_index);
  }

  // Creates and adds a FrameTopology to this MultibodyTreeTopology.
  // All physical frames are associated with a body here identified by their
  // unique index, body_index.
  FrameIndex add_frame(BodyIndex body_index) {
    FrameIndex frame_index(get_num_frames());
    frames.emplace_back(frame_index, body_index);
    return frame_index;
  }

  bool is_valid{false};

  /// Topology gets re-validated by MultibodyTree::Compile().
  void validate() { is_valid = true; }

  bool is_valid_body_id(BodyIndex index) {
    return index < get_num_bodies();
  }

  bool is_valid_frame_id(FrameIndex index) {
    return index < get_num_frames();
  }

  // Clears arrays.
  void clear() {
    bodies.clear();
    frames.clear();
  }

  std::vector<BodyTopology> bodies;
  std::vector<FrameTopology> frames;
};

}  // namespace multibody
}  // namespace drake
