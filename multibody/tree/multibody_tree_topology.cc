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

  if (mobilizer.is_valid() != other.mobilizer.is_valid()) return false;
  if (mobilizer.is_valid() && mobilizer != other.mobilizer) return false;

  if (child_nodes != other.child_nodes) return false;

  if (num_mobilizer_positions != other.num_mobilizer_positions) return false;
  if (mobilizer_positions_start != other.mobilizer_positions_start)
    return false;
  if (num_mobilizer_velocities != other.num_mobilizer_velocities) return false;
  if (mobilizer_velocities_start_in_state !=
      other.mobilizer_velocities_start_in_state)
    return false;
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

MobilizerIndex MultibodyTreeTopology::add_mobilizer(FrameIndex in_frame,
                                                    FrameIndex out_frame,
                                                    int num_positions,
                                                    int num_velocities) {
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
  MobilizerIndex mobilizer_index(num_mobilizers());

  // Make note of the inboard mobilizer for the outboard body.
  rigid_bodies_[outboard_body].inboard_mobilizer = mobilizer_index;
  // Similarly, record inboard_body as the parent of outboard_body.
  rigid_bodies_[outboard_body].parent_body = inboard_body;

  // Records "child" rigid bodies for bookkeeping in the context of the tree
  // structure of the multibody forest.
  rigid_bodies_[inboard_body].child_bodies.push_back(outboard_body);

  mobilizers_.emplace_back(mobilizer_index, in_frame, out_frame, inboard_body,
                           outboard_body, num_positions, num_velocities);
  return mobilizer_index;
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

void MultibodyTreeTopology::Finalize() {
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
  body_nodes_.reserve(num_rigid_bodies());
  while (!stack.empty()) {
    const MobodIndex node(num_mobods());
    const BodyIndex current = stack.top();
    const BodyIndex parent = rigid_bodies_[current].parent_body;

    rigid_bodies_[current].mobod_index = node;

    // Computes level.
    int level = 0;       // level = 0 for the world body.
    if (current != 0) {  // Not the world body.
      level = rigid_bodies_[parent].level + 1;
      const MobilizerIndex mobilizer = rigid_bodies_[current].inboard_mobilizer;
      mobilizers_[mobilizer].mobod_index = node;
    }
    // Updates body levels.
    rigid_bodies_[current].level = level;
    // Keep track of the number of levels, the deepest (i.e. max) level.
    forest_height_ = std::max(forest_height_, level + 1);

    // Since we are doing a DFT, it is valid to ask for the parent node,
    // unless we are at the root.
    MobodIndex parent_node;
    if (node != 0) {  // If we are not at the root:
      parent_node = rigid_bodies_[parent].mobod_index;
      body_nodes_[parent_node].child_nodes.push_back(node);
    }

    // Creates BodyNodeTopology.
    body_nodes_.emplace_back(
        node, level /* node index and level */,
        parent_node /* This node's parent */, current /* This node's body */,
        rigid_bodies_[current].parent_body /* This node's parent body */,
        rigid_bodies_[current].inboard_mobilizer /* This node's mobilizer */);

    // We process bodies in the order they were added to the vector of child
    // bodies; this vector is filled in the order mobilizers are added to the
    // model. Therefore, when a given node branches out, we spawn branches in
    // the order mobilizers that connect this node to its children were added.
    // Since we are using a stack to store bodies that will be processed next,
    // we must place bodies in reverse order so that the first child is at the
    // top of the stack.
    stack.pop();  // Pops top element.
    for (auto it = rigid_bodies_[current].child_bodies.rbegin();
         it != rigid_bodies_[current].child_bodies.rend(); ++it) {
      stack.push(*it);
    }
  }

  // Checks that all bodies were reached. We could have this situation if a
  // user adds a body but forgets to add a mobilizer to it.
  // Bodies that were not reached were not assigned a valid level.
  // TODO(amcastro-tri): this will stop at the first body that is not
  // connected to the tree. Add logic to emit a message with ALL bodies that
  // are not properly connected to the tree.
  for (BodyIndex body(0); body < num_rigid_bodies(); ++body) {
    if (rigid_bodies_[body].level < 0) {
      throw std::runtime_error("Body with index " + std::to_string(body) +
                               " was not assigned a mobilizer");
    }
  }

  // After we checked all bodies were reached above, the number of tree nodes
  // should equal the number of bodies in the tree.
  DRAKE_DEMAND(num_rigid_bodies() == num_mobods());

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
  for (MobodIndex node_index(1); node_index < num_mobods(); ++node_index) {
    BodyNodeTopology& node = body_nodes_[node_index];
    MobilizerTopology& mobilizer = mobilizers_[node.mobilizer];

    // Note that a Weld mobilizer claims it starts at the next available
    // slot but has zero positions and velocities. That means the next
    // mobilizer will start at the same place.
    mobilizer.positions_start = position_index;
    mobilizer.velocities_start_in_state = velocity_index_in_state;
    mobilizer.velocities_start_in_v = velocity_index_in_state - num_positions_;
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
  for (RigidBodyTopology& body : rigid_bodies_) {
    if (body.is_floating) {
      DRAKE_DEMAND(body.inboard_mobilizer.is_valid());
      const MobilizerTopology& mobilizer =
          get_mobilizer(body.inboard_mobilizer);
      body.floating_positions_start = mobilizer.positions_start;
      body.floating_velocities_start_in_v = mobilizer.velocities_start_in_v;
    }
  }

  ExtractForestInfo();

  // We are done with a successful Finalize() and we mark it as so.
  // Do not add any more code after this!
  is_valid_ = true;
}

void MultibodyTreeTopology::GetKinematicPathToWorld(
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

bool MultibodyTreeTopology::IsBodyAnchored(BodyIndex body_index) const {
  DRAKE_DEMAND(is_valid());
  const RigidBodyTopology& body = get_rigid_body(body_index);
  std::vector<MobodIndex> path_to_world;
  GetKinematicPathToWorld(body.mobod_index, &path_to_world);
  // Skip the world at path_to_world[0].
  for (size_t path_index = 1; path_index < path_to_world.size(); ++path_index) {
    const BodyNodeTopology& node = get_body_node(path_to_world[path_index]);
    const MobilizerTopology& mobilizer = get_mobilizer(node.mobilizer);
    // If any of the mobilizers in the path is not a weld mobilizer, the body
    // is not anchored.
    if (!mobilizer.is_weld_mobilizer()) return false;
  }
  // If the loop above completes, then body_index is anchored to the world.
  return true;
}

std::vector<std::set<BodyIndex>>
MultibodyTreeTopology::CreateListOfWeldedBodies() const {
  std::vector<std::set<BodyIndex>> welded_bodies_list;
  // Reserve the maximum possible of welded bodies (that is, when each body
  // forms its own welded body) in advance in order to avoid reallocation in
  // welded_bodies_list which would cause the invalidation of references as
  // we recursively fill it in.
  welded_bodies_list.reserve(num_rigid_bodies());
  welded_bodies_list.push_back(std::set<BodyIndex>{world_index()});
  // We build the list of welded bodies recursively, starting with the world
  // body added to the very first welded body in the list.
  std::set<BodyIndex>& bodies_welded_to_world = welded_bodies_list.back();
  CreateListOfWeldedBodiesRecurse(world_index(), &bodies_welded_to_world,
                                  &welded_bodies_list);
  return welded_bodies_list;
}

std::vector<BodyIndex> MultibodyTreeTopology::GetTransitiveOutboardBodies(
    std::vector<BodyIndex> body_indexes) const {
  DRAKE_DEMAND(is_valid());
  std::unordered_set<BodyIndex> outboard_bodies;
  auto collect_body = [&outboard_bodies](const BodyNodeTopology& node) {
    outboard_bodies.insert(node.rigid_body);
  };
  for (const BodyIndex& body_index : body_indexes) {
    DRAKE_DEMAND(body_index.is_valid() && body_index < num_rigid_bodies());
    // Skip bodies that are already traversed because the subtree with it
    // being the root has necessarily been traversed already.
    if (!outboard_bodies.contains(body_index)) {
      const BodyNodeTopology& root =
          get_body_node(get_rigid_body(body_index).mobod_index);
      TraverseOutboardNodes(root, collect_body);
    }
  }
  std::vector<BodyIndex> results(outboard_bodies.begin(),
                                 outboard_bodies.end());
  std::sort(results.begin(), results.end());
  return results;
}

bool MultibodyTreeTopology::IsThereAMobilizerBetweenFrames(
    FrameIndex frame1, FrameIndex frame2) const {
  for (const auto& mobilizer_topology : mobilizers_) {
    if (mobilizer_topology.connects_frames(frame1, frame2)) return true;
  }
  return false;
}

bool MultibodyTreeTopology::IsThereAMobilizerBetweenRigidBodies(
    BodyIndex body1, BodyIndex body2) const {
  for (const auto& mobilizer_topology : mobilizers_) {
    if (mobilizer_topology.connects_rigid_bodies(body1, body2)) return true;
  }
  return false;
}

void MultibodyTreeTopology::CreateListOfWeldedBodiesRecurse(
    BodyIndex parent_index, std::set<BodyIndex>* parent_welded_body,
    std::vector<std::set<BodyIndex>>* welded_bodies) const {
  const RigidBodyTopology& parent = get_rigid_body(parent_index);
  for (BodyIndex child_index : parent.child_bodies) {
    const RigidBodyTopology& child = get_rigid_body(child_index);
    const MobilizerTopology& child_mobilizer =
        get_mobilizer(child.inboard_mobilizer);
    if (child_mobilizer.is_weld_mobilizer()) {
      // If the child body is welded to the parent body, we then add it to
      // the parent's body welded body, parent_welded_body. We continue the
      // recursion down the tree starting at child.
      parent_welded_body->insert(child_index);
      CreateListOfWeldedBodiesRecurse(child_index, parent_welded_body,
                                      welded_bodies);
    } else {
      // If the child body is not welded to the parent body, then we create a
      // new welded body to which child is added. We continue the recursion
      // down the tree starting at child.
      welded_bodies->push_back(std::set<BodyIndex>{child_index});
      std::set<BodyIndex>& child_group = welded_bodies->back();
      CreateListOfWeldedBodiesRecurse(child_index, &child_group, welded_bodies);
    }
  }
}

void MultibodyTreeTopology::TraverseOutboardNodes(
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
       node_index < num_rigid_bodies() &&
       /* Reached next tree in the multibody forest */
       get_body_node(node_index).level > base_level;
       ++node_index) {
    operation(get_body_node(node_index));
  }
}

int MultibodyTreeTopology::CalcNumberOfOutboardVelocities(
    const BodyNodeTopology& base) const {
  DRAKE_DEMAND(num_mobods() != 0);
  int nv = 0;
  TraverseOutboardNodes(base, [&nv](const BodyNodeTopology& node) {
    nv += node.num_mobilizer_velocities;
  });
  return nv;
}

void MultibodyTreeTopology::ExtractForestInfo() {
  const BodyNodeTopology& root = get_body_node(world_mobod_index());
  const int max_num_trees = root.child_nodes.size();
  num_tree_velocities_.reserve(max_num_trees);
  rigid_body_to_tree_index_.resize(num_rigid_bodies());
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
      rigid_body_to_tree_index_[node.rigid_body] = tree_index;
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

}  // namespace internal
}  // namespace multibody
}  // namespace drake
