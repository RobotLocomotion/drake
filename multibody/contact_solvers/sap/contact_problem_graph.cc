#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

ContactProblemGraph::ContactProblemGraph(int num_cliques)
    : num_cliques_(num_cliques) {}

int ContactProblemGraph::AddConstraint(const SortedPair<int>& cliques_pair,
                                       int num_constraint_equations) {
  const int cluster_index = clusters_.size();

  auto [iterator, new_cluster] = pair_to_cluster_index_.insert(
      std::make_pair(cliques_pair, cluster_index));
  if (new_cluster) clusters_.push_back(ConstraintCluster());

  ConstraintCluster& cluster = clusters_[iterator->second];
  cluster.cliques = cliques_pair;
  cluster.num_constraint_equations += num_constraint_equations;
  cluster.constraint_num_equations.push_back(num_constraint_equations);
  const int constraint_index = num_constraints_;
  cluster.constraint_index.push_back(constraint_index);
  ++num_constraints_;
  num_constraint_equations_ += num_constraint_equations;
  return constraint_index;
}

int ContactProblemGraph::AddConstraint(int clique,
                                       int num_constraint_equations) {
  DRAKE_THROW_UNLESS(0 <= clique && clique < num_cliques());
  DRAKE_THROW_UNLESS(num_constraint_equations >= 0);
  const auto cliques_pair = drake::MakeSortedPair(clique, clique);
  return AddConstraint(cliques_pair, num_constraint_equations);
}

int ContactProblemGraph::AddConstraint(int first_clique, int second_clique,
                                       int num_constraint_equations) {
  DRAKE_THROW_UNLESS(0 <= first_clique && first_clique < num_cliques());
  DRAKE_THROW_UNLESS(0 <= second_clique && second_clique < num_cliques());
  DRAKE_THROW_UNLESS(num_constraint_equations >= 0);
  return AddConstraint(MakeSortedPair(first_clique, second_clique),
                       num_constraint_equations);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
