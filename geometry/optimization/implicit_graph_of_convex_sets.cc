#include "drake/geometry/optimization/implicit_graph_of_convex_sets.h"

#include <queue>
#include <unordered_set>

namespace drake {
namespace geometry {
namespace optimization {

using Edge = GraphOfConvexSets::Edge;
using EdgeId = GraphOfConvexSets::EdgeId;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

ImplicitGraphOfConvexSets::ImplicitGraphOfConvexSets() = default;

ImplicitGraphOfConvexSets::~ImplicitGraphOfConvexSets() = default;

std::vector<Edge*> ImplicitGraphOfConvexSets::Successors(Vertex* v) {
  DRAKE_THROW_UNLESS(v != nullptr);
  if (!gcs_.IsValid(*v)) {
    throw std::runtime_error(fmt::format(
        "Vertex '{}' is not associated with this implicit graph.", v->name()));
  }
  if (!expanded_vertices_.contains(v->id())) {
    Expand(v);
    expanded_vertices_.insert(v->id());
  }
  return v->outgoing_edges();
}

void ImplicitGraphOfConvexSets::ExpandRecursively(Vertex* start,
                                                  int max_successor_calls) {
  DRAKE_THROW_UNLESS(start != nullptr);
  std::queue<Vertex*> queue;
  std::unordered_set<VertexId> visited;
  queue.push(start);
  int count = 0;
  while (!queue.empty()) {
    Vertex* u = queue.front();
    queue.pop();
    visited.insert(u->id());
    for (Edge* e : Successors(u)) {
      Vertex* v = &e->v();
      if (!visited.contains(v->id())) {
        queue.push(v);
      }
    }
    if (++count == max_successor_calls) {
      return;
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
