#pragma once

/// @file
/// This file defines "topological" structures to represent the logical
/// connectivities between multibody tree components. For instance, the
/// BodyTopology for a Body will contain the topological information specifying
/// its inboard body (or parent body) in the parent tree, its outboard bodies
/// (or children) and the level or depth in the MultibodyTree.
/// All of this information is independent of the particular scalar type T the
/// MultibodyTree, and its components, are instantiated with.
/// All of the data structures defined in this file are meant to be the most
/// minimalist representation that can store this information.
/// All of these data structures are meant to be copiable to aid the process of
/// cloning or transmogrifying multibody tree compoments.

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

#include <algorithm>
#include <vector>

namespace drake {
namespace multibody {

/// Data structure to store the topological information associated with a Body.
struct BodyTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyTopology);

  // Default constructor is disabled to force the initialization with the only
  // valid constructor with the signature BodyTopology(BodyIndex, FrameIndex).
  BodyTopology() = delete;

  // Constructs a body topology struct with unique index `unique_index`.
  BodyTopology(BodyIndex body_index, FrameIndex frame_index) :
      index(body_index), body_frame(frame_index) {}

  // Unique id in the MultibodyTree.
  BodyIndex index;

  // Depth level in the MultibodyTree, level = 0 for the world.
  // Initialized to an invalid negative value.
  int level{-1};

  // Body frame.
  FrameIndex body_frame;
};

/// Data structure to store the topological information associated with a
/// PhysicalFrame.
struct PhysicalFrameTopology {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PhysicalFrameTopology);

  // Default constructor is disabled to force the initialization with the only
  // valid constructor with the signature
  // PhysicalFrameTopology(FrameIndex, BodyIndex).
  PhysicalFrameTopology() = delete;

  PhysicalFrameTopology(FrameIndex unique_index, BodyIndex body_index) :
      index(unique_index), body(body_index) {}

  // Unique identifier in the MultibodyTree.  TypeSafeIndex objects must be
  // initialized to non-negative values.
  FrameIndex index;

  // Unique identifier of the body this physical frame attaches to.
  BodyIndex body;
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
  int get_num_physical_frames() const {
    return static_cast<int>(physical_frames.size());
  }

  // Creates and adds a new BodyTopology to this MultibodyTreeTopology.
  // A unique index is assigned to the newly created BodyTopology and a new
  // PhysicalFrameTopology is created and associated to the body topology.
  BodyIndex add_body() {
    invalidate();
    BodyIndex body_index = BodyIndex(get_num_bodies());
    FrameIndex body_frame = add_physical_frame(body_index);
    BodyTopology body(body_index, body_frame);
    bodies.push_back(body);
    return body.index;
  }

  // Creates and adds a PhysicalFrameTopology to this MultibodyTreeTopology.
  // All physical frames are associated with a body here identified by their
  // unique index, body_index.
  FrameIndex add_physical_frame(BodyIndex body_index) {
    invalidate();  // Records topology has changed.
    FrameIndex frame_index(get_num_physical_frames());
    PhysicalFrameTopology frame(frame_index, body_index);
    physical_frames.push_back(frame);
    return frame_index;
  }

  bool is_valid{false};

  // Topology is invalidated when more multibody tree elements are added.
  void invalidate() { is_valid = false; }

  /// Topology gets re-validated by MultibodyTree::Compile().
  void validate() { is_valid = true; }

  bool is_valid_body_id(BodyIndex index) {
    return index < get_num_bodies();
  }

  bool is_valid_frame_id(FrameIndex index) {
    return index < get_num_physical_frames();
  }

  std::vector<BodyTopology> bodies;
  std::vector<PhysicalFrameTopology> physical_frames;
};

}  // namespace multibody
}  // namespace drake
