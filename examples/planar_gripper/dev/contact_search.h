#include <deque>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "drake/examples/planar_gripper/dev/contact_metric.h"
#include "drake/examples/planar_gripper/dev/contact_mode.h"

namespace drake {
namespace examples {
namespace planar_gripper {
namespace contact_search {

std::vector<ContactMode*> depth_first_search_recursion_(
    ContactMode* start, ContactMode* goal,
    std::unordered_set<ContactMode*>& visit_log) {
  // mark node as having been visited
  visit_log.insert(start);

  if (start == goal) {
    // solution found
    std::vector<ContactMode*> path;
    path.push_back(start);
    return path;
  }

  for (ContactMode* mode : start->get_connected_modes()) {
    if (visit_log.find(mode) != visit_log.end()) {
      // node has been visited before
      continue;
    }

    // recursively search for a path
    auto subpath = depth_first_search_recursion_(mode, goal, visit_log);

    if (subpath.size() > 0) {
      // a solution was found so backtrack it
      subpath.push_back(start);
      return subpath;
    }
  }

  // no solution
  std::vector<ContactMode*> empty_path;
  return empty_path;
}

std::vector<ContactMode*> depth_first_search(ContactMode* start,
                                             ContactMode* goal) {
  // keeping track of visited nodes
  std::unordered_set<ContactMode*> visit_log;

  auto path = depth_first_search_recursion_(start, goal, visit_log);

  // returned list is orderd [start,...,goal]
  std::reverse(path.begin(), path.end());

  return path;
}

std::vector<ContactMode*> breadth_first_search(ContactMode* start,
                                               ContactMode* goal) {
  std::deque<std::vector<ContactMode*>> paths_queue;
  std::unordered_set<ContactMode*> visit_log;
  visit_log.insert(start);
  paths_queue.push_back({start});

  while (paths_queue.size() > 0) {
    // no specific order for search order
    auto path = paths_queue.front();
    paths_queue.pop_front();
    if (path.back() == goal) {
      // solution found
      return path;
    }
    visit_log.insert(path.back());
    for (auto contact_mode : path.back()->get_connected_modes()) {
      if (visit_log.find(contact_mode) == visit_log.end()) {
        // the node hasn't been searched yet
        auto new_path = path;
        new_path.push_back(contact_mode);
        paths_queue.push_back(new_path);
      }
    }
  }

  // no solution
  std::vector<ContactMode*> empty_path;
  return empty_path;
}

std::vector<ContactMode*> reconstruct_a_star_solution_(
    std::unordered_map<ContactMode*, ContactMode*> came_from,
    ContactMode* start, ContactMode* goal) {
  std::vector<ContactMode*> path;

  path.push_back(goal);
  while (path.back() != start) {
    path.push_back(came_from[path.back()]);
  }

  // returned list is orderd [start,...,goal]
  std::reverse(path.begin(), path.end());

  return path;
}

template <typename T>
std::vector<ContactMode*> a_star_search(ContactMode* start, ContactMode* goal,
                                        ContactMetric<T>& distance,
                                        ContactMetric<T>& heuristic) {
  // the queue stores (node pointer, f_score) pairs with lowest f_score first
  // out
  auto mode_compare = [](std::pair<ContactMode*, double> lhs,
                         std::pair<ContactMode*, double> rhs) {
    return lhs.second > rhs.second;
  };
  std::priority_queue<std::pair<ContactMode*, double>,
                      std::vector<std::pair<ContactMode*, double>>,
                      decltype(mode_compare)>
      open_set(mode_compare);
  std::unordered_map<ContactMode*,ContactMode*> came_from;
  std::unordered_map<ContactMode*, double> g_score;
  double current_f_score;
  ContactMode* current;

  open_set.push({start, heuristic.eval(start, goal)});
  g_score[start] = 0.;

  while (open_set.size() > 0) {
    // get the node with the lowest f_score i.e. cost + heuristic
    std::tie(current, current_f_score) = open_set.top();
    open_set.pop();

    if (current == goal) {
      // solution found
      return reconstruct_a_star_solution_(came_from, start, goal);
    }

    for (auto neighbor : current->get_connected_modes()) {
      double tentative_g_score =
          g_score[current] + distance.eval(current, neighbor);
      if (g_score.find(neighbor) == g_score.end() ||
          tentative_g_score < g_score[neighbor]) {
        // found a new path or a better path to a known node
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        open_set.push(
            {neighbor, tentative_g_score + heuristic.eval(neighbor, goal)});
      }
    }
  }

  // no solution
  std::vector<ContactMode*> empty_path;
  return empty_path;
}

}  // namespace contact_search
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
