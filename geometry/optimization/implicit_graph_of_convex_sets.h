#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/geometry/optimization/graph_of_convex_sets.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A base class to define the interface to an implicit graph of convex sets.

Implementations of this class must implement DoSuccesors() and provide some
method of accessing at least one vertex in the graph.

@experimental

@ingroup geometry_optimization
*/
class ImplicitGraphOfConvexSets {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitGraphOfConvexSets);

  virtual ~ImplicitGraphOfConvexSets();

  /** Returns the outgoing edges from `v`, which defines the "successors" of
  `v` in the common notation of implicit graph search.

  @throws std::exception if `v` is not already registered with the graph.
  */
  std::vector<GraphOfConvexSets::Edge*> Successors(
      GraphOfConvexSets::Vertex* v);

  /** For finite graphs, this makes repeated calls to Successors() until all
  vertices have been added to the graph, and returns a reference to the internal
  GCS which now contains the entire graph. To protect against infinite loops for
  infinite graphs, we set a maximum number of vertices; this can be set to
  infinity if you are confident your graph is finite.

  @throws std::exception if `start` is not already registered with the graph.
  */
  const GraphOfConvexSets& BuildExplicitGcs(GraphOfConvexSets::Vertex* start,
                                            int max_vertices = 1000);

 protected:
  /** Constructs the (empty) implicit GCS. */
  ImplicitGraphOfConvexSets();

  /* Returns a mutable pointer to the gcs which derived classes can use to call
  AddVertex(), AddEdge(), etc. */
  GraphOfConvexSets* gcs() { return &gcs_; }

  /** DoSuccessors implementations should call gcs()->AddVertex() and
  gcs()->AddEdge() as needed and should only return edges that are registered
  with gcs() and point to vertices that are registered with gcs().

  Due to a caching mechanism, implementations can assume that DoSuccessors(v)
  will only be called once for each `v`. This means that in simple cases, they
  can always calls AddEdge/AddVertex without checking whether those
  edges/vertices have already been added to the gcs(). */
  virtual std::vector<GraphOfConvexSets::Edge*> DoSuccessors(
      GraphOfConvexSets::Vertex* v) = 0;

 private:
  // The internal GCS object maintains ownership of all vertices and edges that
  // have been expanded over the lifetime of this object.
  GraphOfConvexSets gcs_;

  std::map<GraphOfConvexSets::VertexId, std::vector<GraphOfConvexSets::Edge*>>
      successor_cache_{};
};

// TODO(russt): Add an implementation the provides an interface for an implicit
// GCS given an explicit GCS. Note that this will require something to copy
// Vertex and Edges; probably something like GCS::CopyVertex(Vertex* v) that
// copies the sets, costs, and constraints, but not the internal Edge pointers,
// etc.

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
