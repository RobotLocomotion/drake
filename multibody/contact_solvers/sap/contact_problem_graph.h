#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* This class represents the graph for a SapContactProblem where each node in
 the graph corresponds to a clique (for a definition refer to the class's
 documentation for SapContactProblem) and each edge in the graph corresponds to
 a "constraint cluster", see ConstraintCluster. A constraint cluster is the set
 of all constraints acting on the same clique, or on the same pair of cliques.
 This concept is described in detail with an example in [Castro et al., 2021].

 NOTE: This code in Drake is an extension of the ideas in [Castro et al., 2021].
 While [Castro et al., 2021] refers to "trees" in the multibody system, this
 idea is extended to the more general concept of "cliques" for any mechanical
 system. Similarly, while [Castro et al., 2021] refers to the set of contact
 constraints between one or two trees as "patches", this concept has been
 extended to "constraint cluster" to refer to a set of arbitrary constraints
 (not necessarily contact constraints.)

 Clusters are created in the order constraints are added to the problem, see
 AddConstraint(). The first time a constraint between one or two cliques is
 added, a cluster is created. Subsequent addition of more constraints between
 the same set of cliques (one or two) will be associated with the already
 existing cluster in the problem. */
class ContactProblemGraph {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactProblemGraph);

  /* A cluster of constraints is defined as the set of constraints between a set
  of one or two cliques. Since the order in this set of cliques does not matter,
  cliques are stored as a SortedPair. Along with the cliques pair and the set of
  constraints, this struct stores additional metadata related to problem size
  useful when building a mathematical model of the problem. */
  class ConstraintCluster {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstraintCluster);

    /* Constructor for a cluster of constraints between the cliques specified by
     `cliques`. Indexes in `cliques` must be non-negative, or an
     exception is thrown.*/
    explicit ConstraintCluster(SortedPair<int> cliques);

    /* Adds a constraint with index `constraint_index` and
     `num_constraint_equations` constraint equations into this cluster. Both
     `constraint_index` and `num_constraint_equations` must be non-negative or
     an exception is thrown. */
    ConstraintCluster& AddConstraint(int constraint_index,
                                     int num_constraint_equations);

    const SortedPair<int>& cliques() const { return cliques_; }
    int num_constraints() const { return ssize(constraint_index_); }
    int num_total_constraint_equations() const {
      return num_constraint_equations_;
    }
    const std::vector<int>& constraint_index() const {
      return constraint_index_;
    }
    const std::vector<int>& constraint_num_equations() const {
      return constraint_num_equations_;
    }

   private:
    /* The one or two cliques connected by the set of constraints in this
     cluster. We allow cliques.first() and cliques.second() to be the same
     clique to identify a constraint within the same clique. This corresponds to
     a graph edge that connects the clique's node with itself. */
    SortedPair<int> cliques_;

    /* Total number of constraint equations contributed by this cluster. */
    int num_constraint_equations_{0};

    /* Given the i-th constraint in this cluster, constraint_index[i] returns
     the constraint index assigned by ContactProblemGraph::AddConstraint(). */
    std::vector<int> constraint_index_;

    /* Given the i-th constraint in this cluster, constraint_num_equations[i]
     returns the number of constraint equations contributed by the constraint
     with index constraint_index[i]. */
    std::vector<int> constraint_num_equations_;
  };

  /* Constructs an empty graph. */
  ContactProblemGraph() = default;

  /* Constructor for a graph with `num_cliques` nodes.
   @throws if `num_cliques` is negative. */
  explicit ContactProblemGraph(int num_cliques);

  /* Resets this graph to store `num_cliques` and zero constraints.
   @throws if `num_cliques` is negative. */
  void ResetNumCliques(int num_cliques);

  /* Add constraint within a single clique with `num_constraint_equations`. An
   exception is thrown if `clique` is not in [0, num_cliques()) or if
   `num_constraint_equations` is negative. Returns the index to the newly added
   constraint. */
  int AddConstraint(int clique, int num_constraint_equations);

  /* Add constraint between two cliques with `num_constraint_equations`. An
   exception is thrown if either `first_clique` or `second_clique` is not in [0,
   num_cliques()) or if `num_constraint_equations` is negative. Returns the
   index to the newly added constraint. */
  int AddConstraint(int first_clique, int second_clique,
                    int num_constraint_equations);

  /* Number of cliques (nodes) in the graph. */
  int num_cliques() const { return num_cliques_; }

  /* Number of clusters (edges) in the graph. */
  int num_clusters() const { return ssize(clusters_); }

  /* Number of constraints added with calls to AddConstraint(). */
  int num_constraints() const { return num_constraints_; }

  /* Returns the total number of constraint equations for this graph. */
  int num_constraint_equations() const { return num_constraint_equations_; }

  /* The list of all clusters in the graph. Clusters in the returned
   `std::vector` are sorted in the order documented in this class's
   documentation. */
  const std::vector<ConstraintCluster>& clusters() const { return clusters_; }

  /* Get the cluster by index. See clusters() for details on how clusters are
   indexed by `cluster_index`.
   @throws if `cluster_index` is outside [0, num_clusters()). */
  const ConstraintCluster& get_cluster(int cluster_index) const {
    DRAKE_THROW_UNLESS(0 <= cluster_index && cluster_index < num_clusters());
    return clusters_[cluster_index];
  }

  /* Returns the permutation of participating cliques.
   We say a clique is "participating" when referenced by a cluster (an edge) in
   the graph. Since not all cliques might be participating, their "participating
   clique index" can differ from their original clique (node) index.
   Participating cliques are assigned an index in the order they get referenced
   by AddConstraint(). */
  const PartialPermutation& participating_cliques() const {
    return participating_cliques_;
  }

 private:
  /* Helper to add a constraint between a pair of cliques. */
  int AddConstraint(SortedPair<int> cliques, int num_constrained_dofs);

  int num_cliques_{0};
  int num_constraints_{0};
  int num_constraint_equations_{0};
  /* Vector of all clusters in the graph. Clusters are created in the order
   constraints are added to the problem. The first time a constraint between
   two cliques is added, a cluster is created. Subsequent addition of more
   constraints between the same two cliques will be associated with the
   already existing cluster in the problem.
   Given a SortedPair of cliques, an index into this array can be obtained
   with the map pair_to_cluster_index_. */
  std::vector<ConstraintCluster> clusters_;
  // Map cliques pair to cluster index.
  std::unordered_map<SortedPair<int>, int> pair_to_cluster_index_;
  PartialPermutation participating_cliques_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
