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

ImplicitGraphOfConvexSetsFromExplicit::ImplicitGraphOfConvexSetsFromExplicit(
    const GraphOfConvexSets& gcs)
    : explicit_gcs_(gcs) {}

ImplicitGraphOfConvexSetsFromExplicit::
    ~ImplicitGraphOfConvexSetsFromExplicit() = default;

Vertex* ImplicitGraphOfConvexSetsFromExplicit::ImplicitVertexFromExplicit(
    const GraphOfConvexSets::Vertex& v_explicit) {
  DRAKE_THROW_UNLESS(explicit_gcs_.IsValid(v_explicit));
  auto it = explicit_to_implicit_map_.find(v_explicit.id());
  if (it == explicit_to_implicit_map_.end()) {
    Vertex* v_implicit = mutable_gcs().AddVertexFromTemplate(v_explicit);
    explicit_to_implicit_map_[v_explicit.id()] = v_implicit;
    implicit_to_explicit_map_[v_implicit->id()] = &v_explicit;
    return v_implicit;
  }
  return it->second;
}

void ImplicitGraphOfConvexSetsFromExplicit::Expand(Vertex* u_implicit) {
  DRAKE_THROW_UNLESS(u_implicit != nullptr);
  DRAKE_THROW_UNLESS(mutable_gcs().IsValid(*u_implicit));
  // Expand is only called once per vertex, so the outgoing edges should be
  // empty.
  DRAKE_DEMAND(u_implicit->outgoing_edges().empty());

  // Find the corresponding explicit vertex. The Successors() method should
  // only call this with vertices that have been added to the implicit GCS.
  const Vertex* u_explicit = implicit_to_explicit_map_.at(u_implicit->id());
  DRAKE_DEMAND(u_explicit != nullptr);

  // For each outgoing edge from the explicit vertex, add a corresponding edge
  // the to the implicit GCS.
  for (const Edge* e_explicit : u_explicit->outgoing_edges()) {
    DRAKE_DEMAND(e_explicit != nullptr);

    // Add the successor vertex to the implicit GCS if it hasn't been added yet.
    Vertex* v_implicit{nullptr};
    auto it = explicit_to_implicit_map_.find(e_explicit->v().id());
    if (it == explicit_to_implicit_map_.end()) {
      v_implicit = mutable_gcs().AddVertexFromTemplate(e_explicit->v());
      explicit_to_implicit_map_[e_explicit->v().id()] = v_implicit;
      implicit_to_explicit_map_[v_implicit->id()] = &e_explicit->v();
    } else {
      // Then the successor vertex has already been added to the implicit GCS,
      // but the edge has not been added yet (since Expand is only called once
      // per vertex)
      v_implicit = it->second;
    }
    DRAKE_DEMAND(v_implicit != nullptr);
    mutable_gcs().AddEdgeFromTemplate(u_implicit, v_implicit, *e_explicit);
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
