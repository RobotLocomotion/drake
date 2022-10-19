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
  // ready for exploration; the keys of the `parent` map are the already-
  // explored notes, and the values are the ancestor nodes (ancestor in the
  // sense of "towards the `start` node"; NOT in the sense of parent/child).
  std::queue<BodyIndex> worklist({start});
  std::unordered_map<BodyIndex, BodyIndex> ancestors{{start, {}}};
  auto visit_edge = [&](BodyIndex current, BodyIndex next) {
    DRAKE_DEMAND(next.is_valid());
    if (ancestors.emplace(next, current).second) {
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
    if (body_topology.parent_body.is_valid() &&
        body_topology.parent_body == path[i + 1]) {
      mobilizers_on_path.push_back(body_topology.inboard_mobilizer);
    } else {
      mobilizers_on_path.push_back(
          tree.get_topology().get_body(path[i + 1]).inboard_mobilizer);
    }
  }
  return mobilizers_on_path;
}

BodyIndex FindBodyInTheMiddleOfChain(const MultibodyPlant<double>& plant,
                                     BodyIndex start, BodyIndex end) {
  const std::vector<BodyIndex> path = FindPath(plant, start, end);

  // path_only_revolute goes from start to end. If path[i] and path[i-1] is
  // connected through a prismatic joint, then path[i] is not included in
  // path_only_revolute.
  std::vector<BodyIndex> path_only_revolute;
  path_only_revolute.reserve(path.size());
  path_only_revolute.push_back(start);
  const MultibodyTree<double>& tree = GetInternalTree(plant);
  for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
    const BodyTopology& body_topology = tree.get_topology().get_body(path[i]);
    MobilizerIndex mobilizer_index;
    if (body_topology.parent_body.is_valid() &&
        body_topology.parent_body == path[i + 1]) {
      mobilizer_index = body_topology.inboard_mobilizer;
    } else {
      mobilizer_index =
          tree.get_topology().get_body(path[i + 1]).inboard_mobilizer;
    }
    const Mobilizer<double>& mobilizer = tree.get_mobilizer(mobilizer_index);
    if (dynamic_cast<const RevoluteMobilizer<double>*>(&mobilizer) != nullptr) {
      path_only_revolute.push_back(path[i + 1]);
    } else if (dynamic_cast<const WeldMobilizer<double>*>(&mobilizer) !=
               nullptr) {
      continue;
    } else {
      throw std::invalid_argument(
          "FindBodyInTheMiddleOfChain: only allow revolute or weld mobilizer "
          "along the path.");
    }
  }

  return path_only_revolute[(path_only_revolute.size() / 2)];
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
