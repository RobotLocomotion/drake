#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

ContactProblemGraph::ConstraintCluster::ConstraintCluster(
    SortedPair<int> cliques)
    : cliques_(std::move(cliques)) {
  DRAKE_THROW_UNLESS(cliques_.first() >= 0 && cliques_.second() >= 0);
}

ContactProblemGraph::ConstraintCluster&
ContactProblemGraph::ConstraintCluster::AddConstraint(
    int constraint_index, int num_constraint_equations) {
  DRAKE_THROW_UNLESS(constraint_index >= 0 && num_constraint_equations >= 0);
  num_constraint_equations_ += num_constraint_equations;
  constraint_index_.push_back(constraint_index);
  constraint_num_equations_.push_back(num_constraint_equations);
  return *this;
}

ContactProblemGraph::ContactProblemGraph(int num_cliques)
    : num_cliques_(num_cliques), participating_cliques_(num_cliques) {
  DRAKE_THROW_UNLESS(num_cliques >= 0);
}

void ContactProblemGraph::ResetNumCliques(int num_cliques) {
  DRAKE_THROW_UNLESS(num_cliques >= 0);
  num_cliques_ = num_cliques;
  num_constraints_ = 0;
  num_constraint_equations_ = 0;
  clusters_.clear();
  pair_to_cluster_index_.clear();
  participating_cliques_ = PartialPermutation(num_cliques);
}

int ContactProblemGraph::AddConstraint(SortedPair<int> cliques,
                                       int num_constraint_equations) {
  participating_cliques_.push(cliques.first());
  participating_cliques_.push(cliques.second());
  auto [iterator, new_cluster] =
      pair_to_cluster_index_.insert(std::make_pair(cliques, clusters_.size()));
  if (new_cluster) clusters_.emplace_back(std::move(cliques));
  const int cluster_index = iterator->second;
  const int constraint_index = num_constraints_++;
  ConstraintCluster& cluster = clusters_[cluster_index];
  cluster.AddConstraint(constraint_index, num_constraint_equations);
  num_constraint_equations_ += num_constraint_equations;
  return constraint_index;
}

int ContactProblemGraph::AddConstraint(int clique,
                                       int num_constraint_equations) {
  DRAKE_THROW_UNLESS(0 <= clique && clique < num_cliques());
  DRAKE_THROW_UNLESS(num_constraint_equations >= 0);
  return AddConstraint(MakeSortedPair(clique, clique),
                       num_constraint_equations);
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
