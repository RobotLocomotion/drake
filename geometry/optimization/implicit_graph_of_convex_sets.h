#pragma once

#include <set>
#include <string>
#include <unordered_map>
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

  Note: The input arguments are mutable because expanding a vertex requires
  changes to the underlying vertex object. Similarly, the output is mutable
  because callers will need to get mutable vertex pointers from the returned
  edges to expand them further.

  @throws std::exception if `v` is not already registered with the graph.
  */
  std::vector<GraphOfConvexSets::Edge*> Successors(
      GraphOfConvexSets::Vertex* v);

  /** Makes repeated recursive calls to Successors() until no new vertices will
  be added to the graph, or `max_successor_calls` has been reached.

  Note: `v` is mutable because expanding a vertex requires changes to the
  underlying vertex object.

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

  /** Expands a vertex `v` by adding its outgoing edges (and the vertices that
  they point to) to the mutable_gcs(), calling mutable_gcs().AddVertex() and
  mutable_gcs().AddEdge() as needed.

  Due to a caching mechanism, implementations can assume that Expand(v)
  will only be called once for each `v`. */
  virtual void Expand(GraphOfConvexSets::Vertex* v) = 0;

 private:
  // The internal GCS object maintains ownership of all vertices and edges that
  // have been expanded over the lifetime of this object.
  GraphOfConvexSets gcs_;

  std::set<GraphOfConvexSets::VertexId> expanded_vertices_{};
};

/** Provides an implicit GCS interface given an explicit GCS. Vertices and
edges are cloned into the implicit GCS as they are expanded. */
class ImplicitGraphOfConvexSetsFromExplicit final
    : public ImplicitGraphOfConvexSets {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitGraphOfConvexSetsFromExplicit);

  /** Constructs an implicit GCS from an explicit GCS. `gcs` must remain
  valid for the lifetime of this object. */
  explicit ImplicitGraphOfConvexSetsFromExplicit(const GraphOfConvexSets& gcs);

  virtual ~ImplicitGraphOfConvexSetsFromExplicit();

  /** Looks up the implicit vertex corresponding to `v`. If `v` is not
  already in the implicit GCS, it is added.
  @throws std::exception if `v` is not registered with the explicit GCS passed
  in the constructor. */
  GraphOfConvexSets::Vertex* ImplicitVertexFromExplicit(
      const GraphOfConvexSets::Vertex& v_explicit);

 protected:
  void Expand(GraphOfConvexSets::Vertex*) override;

 private:
  const GraphOfConvexSets& explicit_gcs_{};
  std::unordered_map<GraphOfConvexSets::VertexId, GraphOfConvexSets::Vertex*>
      explicit_to_implicit_map_{};
  std::unordered_map<GraphOfConvexSets::VertexId,
                     const GraphOfConvexSets::Vertex*>
      implicit_to_explicit_map_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
