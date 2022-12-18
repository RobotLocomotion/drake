#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

#include <algorithm>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include "drake/multibody/tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {
namespace internal {

std::vector<BodyIndex> FindPath(const MultibodyPlant<double>& plant,
                                BodyIndex start, BodyIndex end) {
  DRAKE_DEMAND(start.is_valid());
  DRAKE_DEMAND(end.is_valid());
  const MultibodyTreeTopology& topology = GetInternalTree(plant).get_topology();

  // Do a breadth first search in the tree. The `worklist` stores the nodes
  // ready for exploration; In the `ancestor` map, the values are the immediate
  // ancestor nodes of the keys (ancestor in the sense of "towards the `start`
  // node"; NOT in the sense of parent/child). All the values in `ancestor` map
  // have been explored.
  std::queue<BodyIndex> worklist({start});
  std::unordered_map<BodyIndex, BodyIndex> ancestors{{start, {}}};
  auto visit_edge = [&](BodyIndex current, BodyIndex next) {
    DRAKE_DEMAND(next.is_valid());
    // Try to insert `next` into the map; no-op in case it already existed.
    const bool inserted = ancestors.emplace(next, current).second;
    if (inserted) {
      // next is just successfully inserted into ancestors.
      worklist.push(next);
    }
  };
  while (!worklist.empty()) {
    BodyIndex current = worklist.front();
    worklist.pop();
    if (current == end) {
      break;
    }
    const BodyTopology& current_node = topology.get_body(current);
    if (current != world_index()) {
      const BodyIndex parent = current_node.parent_body;
      visit_edge(current, parent);
    }
    for (BodyIndex child : current_node.child_bodies) {
      visit_edge(current, child);
    }
  }

  // Retrieve the path in reverse order.
  std::vector<BodyIndex> path;
  for (BodyIndex current = end; ; current = ancestors.at(current)) {
    path.push_back(current);
    if (current == start) {
      break;
    }
  }

  // Switch to forward order.
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<MobilizerIndex> FindMobilizersOnPath(
    const MultibodyPlant<double>& plant, BodyIndex start, BodyIndex end) {
  const std::vector<BodyIndex> path = FindPath(plant, start, end);
  std::vector<MobilizerIndex> mobilizers_on_path;
  mobilizers_on_path.reserve(path.size() - 1);
  const MultibodyTree<double>& tree = GetInternalTree(plant);
  for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
    const BodyTopology& body_topology = tree.get_topology().get_body(path[i]);
    if (path[i] != world_index() && body_topology.parent_body == path[i + 1]) {
      // path[i] is the child of path[i+1] in MultibodyTreeTopology, they are
      // connected by path[i]'s inboard mobilizer.
      mobilizers_on_path.push_back(body_topology.inboard_mobilizer);
    } else {
      // path[i] is the parent of path[i+1] in MultibodyTreeTopology, they are
      // connected by path[i+1]'s inboard mobilizer.
      mobilizers_on_path.push_back(
          tree.get_topology().get_body(path[i + 1]).inboard_mobilizer);
    }
  }
  return mobilizers_on_path;
}

BodyIndex FindBodyInTheMiddleOfChain(const MultibodyPlant<double>& plant,
                                     BodyIndex start, BodyIndex end) {
  const std::vector<BodyIndex> path = FindPath(plant, start, end);

  // path_not_weld goes from start to end, it excludes the path[i] if path[i]
  // and path[i-1] is connected through a welded joint.
  std::vector<BodyIndex> path_not_weld;
  path_not_weld.reserve(path.size());
  path_not_weld.push_back(start);
  const MultibodyTree<double>& tree = GetInternalTree(plant);
  const std::vector<MobilizerIndex> mobilizer_indices =
      FindMobilizersOnPath(plant, path[0], path.back());
  for (int i = 0; i < static_cast<int>(mobilizer_indices.size()); ++i) {
    const MobilizerIndex mobilizer_index = mobilizer_indices[i];
    const Mobilizer<double>& mobilizer = tree.get_mobilizer(mobilizer_index);
    if (mobilizer.num_positions() != 0) {
      path_not_weld.push_back(path[i + 1]);
    }
  }

  return path_not_weld[(path_not_weld.size() / 2)];
}

// TODO(hongkai.dai): determine the joint type through a Reifier.
bool IsRevolute(const Mobilizer<double>& mobilizer) {
  const bool is_revolute =
      (mobilizer.num_positions() == 1 && mobilizer.num_velocities() == 1 &&
       mobilizer.can_rotate() && !mobilizer.can_translate());
  if (is_revolute) {
    DRAKE_THROW_UNLESS(dynamic_cast<const RevoluteMobilizer<double>*>(
                           &mobilizer) != nullptr);
  }
  return is_revolute;
}

// TODO(hongkai.dai): determine the joint type through a Reifier.
bool IsWeld(const Mobilizer<double>& mobilizer) {
  const bool is_weld =
      (mobilizer.num_positions() == 0 && mobilizer.num_velocities() == 0 &&
       !mobilizer.can_rotate() && !mobilizer.can_translate());
  if (is_weld) {
    DRAKE_THROW_UNLESS(dynamic_cast<const WeldMobilizer<double>*>(
                           &mobilizer) != nullptr);
  }
  return is_weld;
}

// TODO(hongkai.dai): determine the joint type through a Reifier.
bool IsPrismatic(const Mobilizer<double>& mobilizer) {
  const bool is_prismatic =
      (mobilizer.num_positions() == 1 && mobilizer.num_velocities() == 1 &&
       !mobilizer.can_rotate() && mobilizer.can_translate());
  if (is_prismatic) {
    DRAKE_THROW_UNLESS(
        dynamic_cast<const PrismaticMobilizer<double>*>(&mobilizer) !=
        nullptr);
  }
  return is_prismatic;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
