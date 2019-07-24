#pragma once

#include <deque>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/examples/planar_gripper/dev/contact_metric.h"
#include "drake/examples/planar_gripper/dev/contact_mode.h"

namespace drake {
namespace examples {
namespace planar_gripper {
namespace contact_search {

std::vector<const ContactMode*> DepthFirstSearchRecursion(
    const ContactMode& start, const ContactMode& goal,
    std::unordered_set<ContactMode::Id>* visit_log) {
  // mark node as having been visited
  visit_log->insert(start.get_id());

  if (start.get_id() == goal.get_id()) {
    // solution found
    return {&start};
  }

  for (const ContactMode* mode : start.get_connected_modes()) {
    if (visit_log->find(mode->get_id()) != visit_log->end()) {
      // node has been visited before
      continue;
    }

    // recursively search for a path
    auto subpath = DepthFirstSearchRecursion(*mode, goal, visit_log);

    if (subpath.size() > 0) {
      // a solution was found so backtrack it
      subpath.push_back(&start);
      return subpath;
    }
  }

  // no solution
  return {};
}

std::vector<const ContactMode*> DepthFirstSearch(const ContactMode& start,
                                                 const ContactMode& goal) {
  // keeping track of visited nodes
  std::unordered_set<ContactMode::Id> visit_log;

  auto path = DepthFirstSearchRecursion(start, goal, &visit_log);

  // returned list is orderd [start,...,goal]
  std::reverse(path.begin(), path.end());

  return path;
}

// Reconstruct a path from start to goal node using the connections in
// came_from. Note that this function returns an empty path if the connections
// are missing.
std::vector<const ContactMode*> ReconstructSolutionPath(
    const std::unordered_map<const ContactMode*, const ContactMode*>& came_from,
    const ContactMode* start, const ContactMode* goal) {
  std::vector<const ContactMode*> path;

  path.push_back(goal);
  uint mode_count = 0;
  while (path.back()->get_id() != start->get_id() &&
         mode_count < came_from.size()) {
    if (came_from.find(path.back()) == came_from.end()) {
      return {};
    }
    path.push_back(came_from.find(path.back())->second);
    mode_count++;
  }

  // returned list is orderd [start,...,goal]
  std::reverse(path.begin(), path.end());

  return path;
}

std::vector<const ContactMode*> BreadthFirstSearch(const ContactMode& start,
                                                   const ContactMode& goal) {
  std::deque<const ContactMode*> paths_queue;
  std::unordered_map<const ContactMode*, const ContactMode*> came_from;
  std::unordered_set<ContactMode::Id> visit_log;

  paths_queue.push_back(&start);

  while (paths_queue.size() > 0) {
    // no specific order for search order
    auto contact_mode = paths_queue.front();
    paths_queue.pop_front();
    if (contact_mode->get_id() == goal.get_id()) {
      // solution found
      return ReconstructSolutionPath(came_from, &start, &goal);
    }
    for (const auto connected_contact_mode :
         contact_mode->get_connected_modes()) {
      if (visit_log.find(connected_contact_mode->get_id()) == visit_log.end()) {
        // the node hasn't been found yet
        visit_log.insert(connected_contact_mode->get_id());
        paths_queue.push_back(connected_contact_mode);
        came_from[connected_contact_mode] = contact_mode;
      }
    }
  }

  // no solution
  return {};
}

template <typename T>
std::vector<const ContactMode*> AStarSearch(const ContactMode& start,
                                            const ContactMode& goal,
                                            const ContactMetric<T>& distance,
                                            const ContactMetric<T>& heuristic) {
  // the queue stores (mode pointer, f_score) pairs with lowest f_score first
  // out
  auto mode_compare = [](std::pair<const ContactMode*, T> lhs,
                         std::pair<const ContactMode*, T> rhs) {
    return lhs.second > rhs.second;
  };
  std::priority_queue<std::pair<const ContactMode*, T>,
                      std::vector<std::pair<const ContactMode*, T>>,
                      decltype(mode_compare)>
      open_set(mode_compare);
  std::unordered_map<const ContactMode*, const ContactMode*> came_from;
  std::unordered_map<ContactMode::Id, T> g_score;

  open_set.push(std::make_pair(&start, heuristic.Eval(start, goal)));
  g_score[start.get_id()] = T(0);

  while (open_set.size() > 0) {
    // get the node with the lowest f_score i.e. cost + heuristic
    const ContactMode* current = open_set.top().first;
    open_set.pop();

    if (current->get_id() == goal.get_id()) {
      // solution found
      return ReconstructSolutionPath(came_from, &start, &goal);
    }

    for (const auto neighbor : current->get_connected_modes()) {
      T tentative_g_score =
          g_score[current->get_id()] + distance.Eval(*current, *neighbor);
      if (g_score.find(neighbor->get_id()) == g_score.end() ||
          tentative_g_score < g_score[neighbor->get_id()]) {
        // found a new path or a better path to a known node
        came_from[neighbor] = current;
        g_score[neighbor->get_id()] = tentative_g_score;
        open_set.push(std::make_pair(
            neighbor, tentative_g_score + heuristic.Eval(*neighbor, goal)));
      }
    }
  }

  // no solution
  return {};
}

}  // namespace contact_search
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
