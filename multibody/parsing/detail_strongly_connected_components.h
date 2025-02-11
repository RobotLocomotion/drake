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
node to its successors. */
template <typename T>
using DirectedGraph = std::unordered_map<T, std::unordered_set<T>>;

/* The result type is an ordered sequence of strongly connected components. */
template <typename T>
using StronglyConnectedComponents = std::vector<std::unordered_set<T>>;

/*
Computes a top-down topologically ordered sequence of strongly connected
components of a directed `graph`, using Tarjan's Algorithm.

Tarjan's Algorithm (named for its discoverer, Robert Tarjan) is a graph theory
algorithm for finding the strongly connected components (SCCs) of a graph.

Based on:
https://en.wikipedia.org/wiki/Tarjan%27s_strongly_connected_components_algorithm
https://logarithmic.net/pfh-files/blog/01208083168/tarjan.py

Recall that strongly connected components are subgraphs where every node is
reachable from every other node in the component. Strongly connected components
form a partition of the graph.
https://en.wikipedia.org/wiki/Strongly_connected_component

@param graph  The graph to analyze.

@returns a top-down topologically ordered sequence of SCCs, with properties
further explained below.

The return value for an empty graph will be an empty sequence.

By the definition of partition, each of the SCCs is non-empty, and their number
will be at least one for a non-empty graph, and at most the number of nodes in
the graph. The sum of sizes of the SCCs will be equal to the number of nodes in
the graph.

Top-down ordering means that result[0] (the "highest" SCC) has no out-edges
(its members have successors outside itself), and result[result.size() - 1]
(the "lowest" SCC) has no in-edges (its members are not successors of nodes
outside itself). Other returned SCCs may have any combination of in-edges or
out-edges, provided topological sort order is maintained.

Note that the returned SCCs form a condensed graph that is guaranteed to be
acyclic -- all cycles have been "hidden" inside an SCC.

@tparam T  The type used for labeling graph nodes.
*/
template <typename T>
StronglyConnectedComponents<T> FindStronglyConnectedComponents(
    const DirectedGraph<T>& graph) {
  int index_counter{0};
  std::vector<T> stack;
  std::unordered_map<T, int> lowlinks;
  std::unordered_map<T, int> index;
  StronglyConnectedComponents<T> result;

  std::function<void(T)> strongconnect;
  strongconnect = [&](T node) {
    // set the depth index for this node to the smallest unused index.
    index[node] = index_counter;
    lowlinks[node] = index_counter;
    ++index_counter;
    stack.push_back(node);

    if (graph.contains(node)) {
      const auto& successors = graph.at(node);
      for (const auto& successor : successors) {
        if (!lowlinks.contains(successor)) {
          // Successor has not yet been visited; recurse on it.
          strongconnect(successor);
          lowlinks[node] = std::min(lowlinks[node], lowlinks[successor]);
        } else if (std::find(stack.begin(), stack.end(), successor) !=
                   stack.end()) {
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
        if (successor == node) {
          break;
        }
      }
      result.emplace_back(std::move(connected_component));
    }
  };

  for (const auto& item : graph) {
    const auto& node = item.first;
    if (!lowlinks.contains(node)) {
      strongconnect(node);
    }
  }

  return result;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
