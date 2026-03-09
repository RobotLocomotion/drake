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
  const MultibodyTree<double>& tree = GetInternalTree(plant);
  const SpanningForest& forest = tree.forest();

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
    const LinkJointGraph::Link& current_link = forest.link_by_index(current);
    const SpanningForest::Mobod& current_mobod =
        forest.mobods(current_link.mobod_index());
    if (current != world_index()) {
      const SpanningForest::Mobod& parent_mobod =
          forest.mobods(current_mobod.inboard());
      const BodyIndex parent =
          tree.forest().links(parent_mobod.link_ordinal()).index();
      visit_edge(current, parent);
    }
    for (const MobodIndex& child_mobod_index : current_mobod.outboards()) {
      const SpanningForest::Mobod& child_mobod =
          forest.mobods(child_mobod_index);
      const BodyIndex child =
          tree.forest().links(child_mobod.link_ordinal()).index();
      visit_edge(current, child);
    }
  }

  // Retrieve the path in reverse order.
  std::vector<BodyIndex> path;
  for (BodyIndex current = end;; current = ancestors.at(current)) {
    path.push_back(current);
    if (current == start) {
      break;
    }
  }

  // Switch to forward order.
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<MobodIndex> FindMobilizersOnPath(
    const MultibodyPlant<double>& plant, BodyIndex start, BodyIndex end) {
  const std::vector<BodyIndex> path = FindPath(plant, start, end);
  std::vector<MobodIndex> mobilizers_on_path;
  mobilizers_on_path.reserve(path.size() - 1);
  const MultibodyTree<double>& tree = GetInternalTree(plant);
  const SpanningForest& forest = tree.forest();
  for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
    // Figure out which mobilizer connects path[i] with path[i+1].
    const LinkJointGraph::Link& link_i = forest.link_by_index(path[i]);
    const LinkJointGraph::Link& link_ip1 = forest.link_by_index(path[i + 1]);
    const SpanningForest::Mobod& mobod_i = forest.mobods(link_i.mobod_index());
    if (!mobod_i.is_world() && mobod_i.inboard() == link_ip1.mobod_index()) {
      // path[i] is the child of path[i+1] in MultibodyTreeTopology, they are
      // connected by path[i]'s inboard mobilizer.
      mobilizers_on_path.push_back(link_i.mobod_index());
    } else {
      // path[i] is the parent of path[i+1] in MultibodyTreeTopology, they are
      // connected by path[i+1]'s inboard mobilizer.
      mobilizers_on_path.push_back(link_ip1.mobod_index());
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
  const std::vector<MobodIndex> mobilizer_indices =
      FindMobilizersOnPath(plant, path[0], path.back());
  for (int i = 0; i < static_cast<int>(mobilizer_indices.size()); ++i) {
    const MobodIndex mobilizer_index = mobilizer_indices[i];
    const Mobilizer<double>& mobilizer = tree.get_mobilizer(mobilizer_index);
    if (mobilizer.num_positions() != 0) {
      path_not_weld.push_back(path[i + 1]);
    }
  }

  return path_not_weld[(path_not_weld.size() / 2)];
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
