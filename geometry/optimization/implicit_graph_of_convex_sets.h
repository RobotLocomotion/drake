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
  `v` in the common notation of implicit graph search. The internal gcs() object
  is expanded as needed to include the edges (and the vertices they point to)
  that are returned.

  @throws std::exception if `v` is not already registered with the graph.
  */
  std::vector<GraphOfConvexSets::Edge*> Successors(
      GraphOfConvexSets::Vertex* v);

  /** Makes repeated recursive calls to Successors() until no new vertices will
  be added to the graph, or `max_successor_calls` has been reached.

  @throws std::exception if `v` is not already registered with the graph.
  */
  void ExpandRecursively(GraphOfConvexSets::Vertex* v,
                         int max_successor_calls = 1000);

  /* Returns a const reference to the GCS that has been expanded so far. */
  const GraphOfConvexSets& gcs() const { return gcs_; }

 protected:
  /** Constructs the (empty) implicit GCS. */
  ImplicitGraphOfConvexSets();

  /* Returns a mutable reference to the gcs which derived classes can use to
  call AddVertex(), AddEdge(), etc. */
  GraphOfConvexSets& mutable_gcs() { return gcs_; }

  /** Expand() implementations should call mutable_gcs().AddVertex() and
  mutable_gcs().AddEdge() as needed and should only return edges that are
  registered with gcs_ and point to vertices that are registered with gcs_.

  Due to a caching mechanism, implementations can assume that Expand(v)
  will only be called once for each `v`. */
  virtual std::vector<GraphOfConvexSets::Edge*> Expand(
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
