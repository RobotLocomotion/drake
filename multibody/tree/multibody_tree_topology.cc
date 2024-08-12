#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

bool MultibodyTreeTopology::operator==(
    const MultibodyTreeTopology& other) const {
  if (is_valid_ != other.is_valid_) return false;
  if (forest_height_ != other.forest_height_) return false;

  if (frames_ != other.frames_) return false;
  if (rigid_bodies_ != other.rigid_bodies_) return false;
  if (mobilizers_ != other.mobilizers_) return false;
  if (joint_actuators_ != other.joint_actuators_) return false;
  if (body_nodes_ != other.body_nodes_) return false;

  if (num_positions_ != other.num_positions_) return false;
  if (num_velocities_ != other.num_velocities_) return false;
  if (num_states_ != other.num_states_) return false;
  if (num_actuated_dofs_ != other.num_actuated_dofs_) return false;

  if (num_tree_velocities_ != other.num_tree_velocities_) return false;
  if (tree_velocities_start_in_v_ != other.tree_velocities_start_in_v_)
    return false;
  // Each velocity should have a valid tree index, so it is ok to compare these
  // directly.
  if (velocity_to_tree_index_ != other.velocity_to_tree_index_) return false;
  // The world body (BodyIndex(0)) does not have a valid tree index so we skip
  // it when comparing for equality.
  DRAKE_DEMAND(!other.rigid_body_to_tree_index_[0].is_valid());
  return std::equal(rigid_body_to_tree_index_.begin() + 1,
                    rigid_body_to_tree_index_.end(),
                    other.rigid_body_to_tree_index_.begin() + 1,
                    other.rigid_body_to_tree_index_.end());
}

bool RigidBodyTopology::operator==(const RigidBodyTopology& other) const {
  if (index != other.index) return false;
  if (inboard_mobilizer.is_valid() != other.inboard_mobilizer.is_valid())
    return false;
  if (inboard_mobilizer.is_valid() &&
      inboard_mobilizer != other.inboard_mobilizer)
    return false;
  if (parent_body.is_valid() != other.parent_body.is_valid()) return false;
  if (parent_body.is_valid() && parent_body != other.parent_body) return false;
  if (child_bodies != other.child_bodies) return false;
  if (body_frame != other.body_frame) return false;
  if (level != other.level) return false;
  if (mobod_index != other.mobod_index) return false;
  if (is_floating != other.is_floating) return false;
  if (has_quaternion_dofs != other.has_quaternion_dofs) return false;
  if (floating_positions_start != other.floating_positions_start) return false;
  if (floating_velocities_start_in_v != other.floating_velocities_start_in_v)
    return false;
  return true;
}

bool BodyNodeTopology::operator==(const BodyNodeTopology& other) const {
  if (index != other.index) return false;
  if (level != other.level) return false;

  if (parent_body_node.is_valid() != other.parent_body_node.is_valid())
    return false;
  if (parent_body_node.is_valid() && parent_body_node != other.parent_body_node)
    return false;

  if (rigid_body != other.rigid_body) return false;

  if (parent_rigid_body.is_valid() != other.parent_rigid_body.is_valid())
    return false;
  if (parent_rigid_body.is_valid() &&
      parent_rigid_body != other.parent_rigid_body)
    return false;

  if (child_nodes != other.child_nodes) return false;

  if (num_mobilizer_positions != other.num_mobilizer_positions) return false;
  if (mobilizer_positions_start != other.mobilizer_positions_start)
    return false;
  if (num_mobilizer_velocities != other.num_mobilizer_velocities) return false;
  if (mobilizer_velocities_start_in_v != other.mobilizer_velocities_start_in_v)
    return false;

  return true;
}

MultibodyTreeTopology::~MultibodyTreeTopology() = default;

const JointActuatorTopology& MultibodyTreeTopology::get_joint_actuator(
    JointActuatorIndex index) const {
  DRAKE_ASSERT(index < ssize(joint_actuators_));
  DRAKE_DEMAND(joint_actuators_[index].has_value());
  return *joint_actuators_[index];
}

std::pair<BodyIndex, FrameIndex> MultibodyTreeTopology::add_rigid_body() {
  if (is_valid()) {
    throw std::logic_error(
        "This MultibodyTreeTopology is finalized already. "
        "Therefore adding more rigid bodies is not allowed. "
        "See documentation for Finalize() for details.");
  }
  BodyIndex body_index = BodyIndex(num_rigid_bodies());
  FrameIndex body_frame_index = add_frame(body_index);
  rigid_bodies_.emplace_back(body_index, body_frame_index);
  return std::make_pair(body_index, body_frame_index);
}

FrameIndex MultibodyTreeTopology::add_frame(BodyIndex body_index) {
  if (is_valid()) {
    throw std::logic_error(
        "This MultibodyTreeTopology is finalized already. "
        "Therefore adding more frames is not allowed. "
        "See documentation for Finalize() for details.");
  }
  FrameIndex frame_index(num_frames());
  frames_.emplace_back(frame_index, body_index);
  return frame_index;
}

MobodIndex MultibodyTreeTopology::add_mobilizer(
    const SpanningForest::Mobod& mobod, FrameIndex in_frame,
    FrameIndex out_frame) {
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
  const BodyIndex inboard_body = frames_[in_frame].rigid_body;
  const BodyIndex outboard_body = frames_[out_frame].rigid_body;
  if (IsThereAMobilizerBetweenRigidBodies(inboard_body, outboard_body)) {
    throw std::runtime_error(fmt::format(
        "This multibody tree already has a mobilizer connecting "
        "inboard rigid body (index={}) and outboard rigid body (index={}). "
        "More than one mobilizer between two bodies is not allowed.",
        inboard_body, outboard_body));
  }
  // Checks for graph loops. Each body can have only one inboard mobilizer.
  if (rigid_bodies_[outboard_body].inboard_mobilizer.is_valid()) {
    throw std::runtime_error(
        "When creating a model, an attempt was made to add two inboard "
        "joints to the same rigid body; this is not allowed. One possible "
        "cause might be attempting to weld a robot to World somewhere other "
        "than its base rigid body; see Drake issue #17429 for discussion and "
        "work-arounds, e.g., reversing some joint parent/child directions. "
        "Another possible cause might be attempting to form a kinematic "
        "loop using joints; to create a loop, consider using a "
        "LinearBushingRollPitchYaw instead of a joint.");
  }

  // The checks above guarantee that it is the first time we add an inboard
  // mobilizer to `outboard_body`. The DRAKE_DEMANDs below double check our
  // implementation. RigidBodyTopology::inboard_mobilizer and
  // RigidBodyTopology::parent_body are both set within this method right
  // after these checks.
  DRAKE_DEMAND(!rigid_bodies_[outboard_body].inboard_mobilizer.is_valid());
  DRAKE_DEMAND(!rigid_bodies_[outboard_body].parent_body.is_valid());
  DRAKE_DEMAND(mobod.index() == num_mobilizers());
  const MobodIndex mobilizer_index(num_mobilizers());

  // Make note of the inboard mobilizer for the outboard body.
  rigid_bodies_[outboard_body].inboard_mobilizer = mobilizer_index;
  // Similarly, record inboard_body as the parent of outboard_body.
  rigid_bodies_[outboard_body].parent_body = inboard_body;

  // Records "child" rigid bodies for bookkeeping in the context of the tree
  // structure of the multibody forest.
  rigid_bodies_[inboard_body].child_bodies.push_back(outboard_body);

  mobilizers_.emplace_back(mobilizer_index, in_frame, out_frame, inboard_body,
                           outboard_body, mobod);
  return mobilizer_index;
}

void MultibodyTreeTopology::add_world_mobilizer(
    const SpanningForest::Mobod& world_mobod, FrameIndex world_body_frame) {
  DRAKE_DEMAND(world_mobod.is_world());
  const BodyIndex world_body_index = frames_[world_body_frame].rigid_body;
  DRAKE_DEMAND(world_body_index == BodyIndex(0));
  DRAKE_DEMAND(mobilizers_.empty());
  // There is no inboard frame for this mobilizer. We'll use the World body
  // frame and rigid body for both frames & bodies so we don't have to deal
  // with invalid indices. No computation depends on those.
  mobilizers_.emplace_back(MobodIndex(0), world_body_frame, world_body_frame,
                           world_body_index, world_body_index, world_mobod);
  rigid_bodies_[world_body_index].inboard_mobilizer = MobodIndex(0);
}

JointActuatorIndex MultibodyTreeTopology::add_joint_actuator(int num_dofs) {
  DRAKE_ASSERT(num_dofs > 0);
  if (is_valid()) {
    throw std::logic_error(
        "This MultibodyTreeTopology is finalized already. "
        "Therefore adding more joint actuators is not allowed. "
        "See documentation for Finalize() for details.");
  }
  const int actuator_index_start = num_actuated_dofs();
  const JointActuatorIndex actuator_index(ssize(joint_actuators_));
  joint_actuators_.push_back(
      JointActuatorTopology{.index = actuator_index,
                            .actuator_index_start = actuator_index_start,
                            .num_dofs = num_dofs});
  num_actuated_dofs_ += num_dofs;
  return actuator_index;
}

void MultibodyTreeTopology::RemoveJointActuator(
    JointActuatorIndex actuator_index) {
  DRAKE_DEMAND(actuator_index < ssize(joint_actuators_));
  DRAKE_THROW_UNLESS(!is_valid());
  DRAKE_THROW_UNLESS(joint_actuators_[actuator_index].has_value());

  // Reduce the total number of actuated dofs.
  const int num_dofs = (*joint_actuators_[actuator_index]).num_dofs;
  DRAKE_DEMAND(num_actuated_dofs_ >= num_dofs);
  num_actuated_dofs_ -= num_dofs;

  // Mark the actuator as "removed".
  joint_actuators_[actuator_index] = std::nullopt;

  // Update the actuator_index_start for all joint actuators that come after
  // the one we just removed.
  for (JointActuatorIndex i(actuator_index); i < ssize(joint_actuators_); ++i) {
    if (joint_actuators_[i].has_value()) {
      (*joint_actuators_[i]).actuator_index_start -= num_dofs;
    }
  }
}

// TODO(sherm1) For historical reasons we're extracting information from the
//  Forest and distributing it but we should work directly from the Forest
//  without duplication.
void MultibodyTreeTopology::Finalize(const LinkJointGraph& graph) {
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
  // indexed identically. Note that Mobods are already in depth-first order so
  // we can use the same numbering for BodyNodes. Also update RigidBodyTopology,
  // though in the case where we combine welded-together rigid bodies only the
  // "active" body of each Composite gets updated here.
  body_nodes_.reserve(ssize(forest.mobods()));
  for (const auto& mobod : forest.mobods()) {
    const MobodIndex node_index(mobod.index());
    const BodyIndex rigid_body_index =
        graph.links(mobod.link_ordinal()).index();
    RigidBodyTopology& current_body = rigid_bodies_[rigid_body_index];
    current_body.mobod_index = node_index;
    current_body.level = mobod.level();
    const MobodIndex mobilizer_index = current_body.inboard_mobilizer;
    DRAKE_DEMAND(mobilizer_index == node_index);
    const MobilizerTopology& mobilizer = mobilizers_[mobilizer_index];
    DRAKE_DEMAND(mobilizer.index == node_index);

    MobodIndex parent_node;
    const BodyIndex parent_body_index =
        current_body.parent_body;  // invalid if World
    if (node_index != 0) {         // Skip if World.
      // N.B. This works because we're working depth-first so will already
      // have processed this node's parent.
      parent_node = rigid_bodies_[parent_body_index].mobod_index;
      body_nodes_[parent_node].child_nodes.push_back(node_index);
    }

    // Creates a BodyNodeTopology.
    DRAKE_DEMAND(node_index == ssize(body_nodes_));
    body_nodes_.emplace_back(node_index, mobod.level(), parent_node,
                             rigid_body_index, parent_body_index);

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

  // Update position/velocity indexes for free rigid bodies so that they are
  // easily accessible.
  for (RigidBodyTopology& rigid_body : rigid_bodies_) {
    if (rigid_body.is_floating) {
      DRAKE_DEMAND(rigid_body.inboard_mobilizer.is_valid());
      const MobilizerTopology& mobilizer =
          get_mobilizer(rigid_body.inboard_mobilizer);
      rigid_body.floating_positions_start = mobilizer.positions_start;
      rigid_body.floating_velocities_start_in_v =
          mobilizer.velocities_start_in_v;
    }
  }

  ExtractForestInfo(graph);

  // We are done with a successful Finalize() and we mark it as so.
  // Do not add any more code after this!
  is_valid_ = true;
}

bool MultibodyTreeTopology::IsThereAMobilizerBetweenFrames(
    FrameIndex frame1, FrameIndex frame2) const {
  for (const auto& mobilizer_topology : mobilizers_) {
    if (mobilizer_topology.connects_frames(frame1, frame2)) return true;
  }
  return false;
}

// Returns `true` if there is _any_ mobilizer in the multibody tree
// connecting the Links with indexes `body1` and `body2`.
bool MultibodyTreeTopology::IsThereAMobilizerBetweenRigidBodies(
    BodyIndex body1, BodyIndex body2) const {
  for (const auto& mobilizer_topology : mobilizers_) {
    if (mobilizer_topology.connects_rigid_bodies(body1, body2)) return true;
  }
  return false;
}

// TODO(sherm1) Currently this copies from the graph and forest into the
//  previous data structures, to establish that we are computing the same
//  quantities. Once this works, should switch to using the forest data
//  directly and cut out the redundant data structures.
void MultibodyTreeTopology::ExtractForestInfo(const LinkJointGraph& graph) {
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

  // Map each Link in the graph (including shadows added to break loops) to
  // the tree
  // to which its modeling Mobod belongs.
  rigid_body_to_tree_index_.resize(ssize(graph.links()));
  for (const auto& link : graph.links()) {
    const MobodIndex mobod_index = graph.link_to_mobod(link.index());
    DRAKE_DEMAND(mobod_index.is_valid());
    const SpanningForest::Mobod& mobod = forest.mobods(mobod_index);

    // The tree index will be invalid for World.
    rigid_body_to_tree_index_[link.index()] = mobod.tree();
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
