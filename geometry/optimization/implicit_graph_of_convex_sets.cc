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
    throw std::runtime_error(
        fmt::format("Vertex '{}' is not associated with this implicit graph. "
                    "Valid vertices can be obtained by calling source() or by "
                    "(recursive) calls to Successors().",
                    v->name()));
  }
  if (successor_cache_.find(v->id()) != successor_cache_.end()) {
    return successor_cache_[v->id()];
  }
  auto result = DoSuccessors(v);
  successor_cache_[v->id()] = result;
  return result;
}

const GraphOfConvexSets& ImplicitGraphOfConvexSets::BuildExplicitGcs(
    Vertex* start, int max_vertices) {
  DRAKE_THROW_UNLESS(start != nullptr);
  std::queue<Vertex*> queue;
  std::unordered_set<VertexId> visited;
  queue.push(start);
  while (!queue.empty() && gcs_.num_vertices() < max_vertices) {
    Vertex* u = queue.front();
    queue.pop();
    visited.insert(u->id());
    for (Edge* e : Successors(u)) {
      Vertex* v = &e->v();
      if (!visited.contains(v->id())) {
        queue.push(v);
      }
    }
  }
  if (gcs_.num_vertices() == max_vertices) {
    log()->warn(
        "ImplicitGraphOfConvexSets::BuildExplicitGCS() reached the max number "
        "of vertices. The graph is not fully expanded.");
  }
  return gcs_;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
