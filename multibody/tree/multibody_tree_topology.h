#pragma once

/// @file
/// This file is unnecessarily duplicative of multibody/topology and is
/// being removed.

#include <optional>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

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
// JointActuator.
struct JointActuatorTopology {
  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const JointActuatorTopology& other) const = default;

  // Unique index in the MultibodyTree.
  JointActuatorIndex index{0};
  // For an actuator in a MultibodyTree model, this dof start index corresponds
  // to the first entry in the global array u containing all actuation values
  // for the entire model. Actuator dof start indexes are assigned in the order
  // actuators are added to the model, that is, in the order of
  // JointActuatorIndex.
  int actuator_dof_start{-1};
  // The number of dofs actuated by this actuator.
  int num_dofs{-1};
};

// Data structure to store the topological information associated with an
// entire SpanningForest of a MultibodyPlant.
class MultibodyTreeTopology {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyTreeTopology);

  // Default constructor creates an empty, invalid topology.
  MultibodyTreeTopology() {}

  ~MultibodyTreeTopology();

  // Returns `true` if all members of `this` topology are exactly equal to the
  // members of `other`.
  bool operator==(const MultibodyTreeTopology& other) const;

  // Returns the number of physical frames in the multibody tree.
  int num_frames() const { return ssize(frame_topology_); }

  // Returns the number of joint actuators in the topology.
  int num_joint_actuators() const { return ssize(joint_actuator_topology_); }

  // Returns the number of levels in the forest topology.
  int forest_height() const { return forest_height_; }

  // Returns a constant reference to the corresponding FrameTopology given the
  // FrameIndex.
  const FrameTopology& get_frame_topology(FrameIndex index) const {
    DRAKE_ASSERT(index < num_frames());
    return frame_topology_[index];
  }

  // Returns a constant reference to the corresponding JointActuatorTopology
  // given a JointActuatorIndex.
  const JointActuatorTopology& get_joint_actuator_topology(
      JointActuatorIndex index) const;

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
    DRAKE_ASSERT(b < ssize(rigid_body_to_tree_index_));
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

  // Creates and adds a new FrameTopology, associated with the given
  // BodyIndex, to this MultibodyTreeTopology.
  //
  // @throws std::exception if FinalizeTopology() was already called on `this`
  // topology.
  // @pre the FrameIndex is the one for the next available slot
  void add_frame_topology(FrameIndex, BodyIndex);

  // Creates and adds a new JointActuatorTopology for a joint with `num_dofs`
  // degrees of freedom.
  //
  // @throws std::exception if FinalizeTopology() was already called on `this`
  // topology.
  // @pre the given index is the one for the next available slot
  void add_joint_actuator_topology(JointActuatorIndex actuator_index,
                                   int num_dofs);

  // Removes `actuator_index` from the list of joint actuators. The
  // `actuator_dof_start` will be modified if necessary for other actuators.
  // @throws std::exception if called post-Finalize.
  // @throws std::exception if the actuator with the index `actuator_index` has
  // already been removed.
  void RemoveJointActuatorTopology(JointActuatorIndex actuator_index);

  // This method must be called by MultibodyTree::Finalize() after all
  // topological elements in the plant (rigid bodies, joints, constraints) were
  // added and a suitable SpanningForest built.
  //
  // We extract the necessary topological information from the Forest, i.e. how
  // rigid bodies and joints are interconnected, and use that information to
  // build MobilizerTopology objects directly in depth-first order and extract
  // Tree structure.
  //
  // If the finalize stage is successful, the `this` topology is validated,
  // meaning it is up-to-date after this call. No more multibody elements can be
  // added after a call to Finalize().
  //
  // @throws std::exception If users attempt to call this method on an
  //         already finalized topology.
  // @see is_valid()
  void FinalizeTopology(const LinkJointGraph& graph);

  // Returns `true` if FinalizeTopology() was already called on `this` topology.
  // @see FinalizeTopology()
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
  // Helper method to be used within FinalizeTopology() to obtain the
  // topological information that describes the multibody system as a "forest"
  // of trees.
  void ExtractForestInfo(const LinkJointGraph& graph);

  // is_valid is set to `true` after a successful FinalizeTopology().
  bool is_valid_{false};
  // Number of levels in the full Forest topology. After FinalizeTopology()
  // there will be at least one level (level = 0) with the world body.
  int forest_height_{-1};

  // Topological elements:
  std::vector<FrameTopology> frame_topology_;
  std::vector<std::optional<JointActuatorTopology>> joint_actuator_topology_;

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
