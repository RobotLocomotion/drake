#pragma once

#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drake {
namespace multibody {
namespace internal {

/* The directed graph type to operate on. This is an adjacency list, mapping a
 * node to its successors. */
template <typename T>
using SccGraph = std::unordered_map<T, std::unordered_set<T>>;

/* The result type is an ordered sequence of strongly connected components. */
template <typename T>
using SccResult = std::vector<std::unordered_set<T>>;

/*
@returns a bottom-up sequence of strongly connected components of a directed
`graph`, using Tarjan's Algorithm.

Tarjan's Algorithm (named for its discoverer, Robert Tarjan) is a graph theory
algorithm for finding the strongly connected components of a graph.

Based on:
http://en.wikipedia.org/wiki/Tarjan%27s_strongly_connected_components_algorithm
https://logarithmic.net/pfh-files/blog/01208083168/tarjan.py
*/
template <typename T>
SccResult<T> strongly_connected_components(const SccGraph<T>& graph) {
  int index_counter{0};
  std::vector<T> stack;
  std::unordered_map<T, int> lowlinks;
  std::unordered_map<T, int> index;
  SccResult<T> result;

  std::function<void(T)> strongconnect;
  strongconnect = [&](T node) {
    // set the depth index for this node to the smallest unused index.
    index[node] = index_counter;
    lowlinks[node] = index_counter;
    ++index_counter;
    stack.push_back(node);

    if (graph.count(node)) {
      const auto& successors = graph.at(node);
      for (const auto& successor : successors) {
        if (!lowlinks.count(successor)) {
          // Successor has not yet been visited; recurse on it.
          strongconnect(successor);
          lowlinks[node] = std::min(lowlinks[node], lowlinks[successor]);
        } else if (std::find(stack.begin(), stack.end(), successor)
                   != stack.end()) {
          // the successor is in the stack and hence in the current strongly
          // connected component (SCC).
          lowlinks[node] = std::min(lowlinks[node], index[successor]);
        }
      }
    }

    // If `node` is a root node, pop the stack and generate an SCC.
    if (lowlinks[node] == index[node]) {
      std::unordered_set<T> connected_component;

      while (true) {
        T successor = stack.back();
        stack.pop_back();
        connected_component.insert(successor);
        if (successor == node) { break; }
      }
      result.emplace_back(std::move(connected_component));
    }
  };

  for (const auto& item : graph) {
    const auto& node = item.first;
    if (!lowlinks.count(node)) {
      strongconnect(node);
    }
  }

  return result;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
