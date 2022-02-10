#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* This class represents the graph for a SapContactProblem where each node in
the graph corresponds to a clique and each edge in the graph corresponds to a
"constraint cluster", see ConstraintCluster. A cluster of constraints or
"constraint cluster" is defined as the set of constraints between one or two
cliques. This concept is described in detail with an example in [Castro et al.,
2021].

NOTE: This code in Drake is an extension to the ideas in [Castro et al., 2021].
While [Castro et al., 2021] refers to "trees" in the multibody system, this idea
is extended to the more general concept of "cliques" for any mechanical system.
Similarly, while [Castro et al., 2021] refers to as "patches" to the the set of
constact constraints between one or two trees, this concept has been extended to
"constraint cluster" to refer to a set of arbitrary constraints (not necessarily
contact constraints.)

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
  struct ConstraintCluster {
    /* The one or two cliques connected by the set of constraints in this
     cluster. We allow cliques.first() and cliques.second() to be the same
     clique to identify a constraint within the same clique. This corresponds to
     a graph edge that connects the clique's node with itself. */
    drake::SortedPair<int> cliques;

    /* Total number of constraint equations contributed by this cluster. */
    int num_constraint_equations{0};

    /* Given the i-th constraint in this cluster, constraint_index[i] returns
     the constraint index assigned by ContactProblemGraph::AddConstraint(). */
    std::vector<int> constraint_index;

    /* Given the i-th constraint in this cluster, constraint_num_equations[i]
     returns the number of constraint equations contributed by the constraint
     with index constraint_index[i]. */
    std::vector<int> constraint_num_equations;
  };

  /* Constructor for a graph with `num_cliques`. */
  explicit ContactProblemGraph(int num_cliques);

  /* Add constraint within a single clique with `num_constraint_equations`. An
   exception is thrown if `clique` is not in (0, num_cliques()) or if
   `num_constraint_equations` is negative. Returns the index to the newly added
   constraint. */
  int AddConstraint(int clique, int num_constraint_equations);

  /* Add constraint between two cliques with `num_constraint_equations`. An
   exception is thrown if either `first_clique` or `second_clique` is not in (0,
   num_cliques()) or if `num_constraint_equations` is negative. Returns the
   index to the newly added constraint. */
  int AddConstraint(int first_clique, int second_clique,
                    int num_constrained_dofs);

  /* Number of cliques (nodes) in the graph. */
  int num_cliques() const { return num_cliques_; }

  /* Number of clusters (edges) in the graph. */
  int num_clusters() const { return static_cast<int>(clusters_.size()); }

  /* Number of constraints added with calls to AddConstraint(). */
  int num_constraints() const { return num_constraints_; }

  /* Returns the total number of constraint equations for this graph. */
  int num_constraint_equations() const { return num_constraint_equations_; }

  /* The list of all clusters in the graph. Clusters in the returned
   `std::vector` are sorted in the order documented in this class's
   documentation. */
  const std::vector<ConstraintCluster>& clusters() const { return clusters_; }

  /* Get the cluster by index. See clusters() for details on how clusters are
   indexed by `cluster_index`. */
  const ConstraintCluster& get_cluster(int cluster_index) const {
    return clusters_[cluster_index];
  }

 private:
  int num_cliques_{0};
  int num_constraints_{0};
  int num_constraint_equations_{0};
  // Vector of all clusters in the graph. Clusters are created in the order
  // constraints are added to the problem. The first time a constraint between
  // two cliques is added, a cluster is created. Subsequent addition of more
  // constraints between the same two cliques will be associated with the
  // already existing cluster in the problem.
  // Given a SortedPair of cliques, an index into this array can be obtained
  // with the map pair_to_cluster_index_.
  std::vector<ConstraintCluster> clusters_;
  // Map cliques pair to cluster index.
  std::unordered_map<SortedPair<int>, int> pair_to_cluster_index_;

  /* Helper to add a constraint between a pair of cliques. */
  int AddConstraint(const SortedPair<int>& cliques_pair,
                    int num_constrained_dofs);
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
