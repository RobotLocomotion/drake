#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <iostream>
#include <map>

#include "drake/common/drake_copyable.h"
#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapContactProblem<T>::SapContactProblem(const T& time_step,
                                        std::vector<MatrixX<T>>&& A,
                                        VectorX<T>&& v_star)
    : time_step_(time_step), A_(std::move(A)), v_star_(std::move(v_star)) {
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_DEMAND(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_DEMAND(v_star_.size() == nv_);
}

template <typename T>
SapContactProblem<T>::SapContactProblem(
    std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
    std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints)
    : A_(std::move(A)),
      v_star_(std::move(v_star)),
      constraints_(std::move(constraints)) {
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_DEMAND(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_DEMAND(v_star_.size() == nv_);
}

template <typename T>
void SapContactProblem<T>::AddConstraint(std::unique_ptr<SapConstraint<T>> c) {
  DRAKE_DEMAND(c->clique0() < num_cliques());
  DRAKE_DEMAND(c->clique1() < num_cliques());
  DRAKE_DEMAND(c->clique0_jacobian().cols() == num_velocities(c->clique0()));
  if (c->clique1() >= 0) {
    DRAKE_DEMAND(c->clique1_jacobian().cols() == num_velocities(c->clique1()));
  }
  num_constrained_dofs_ += c->num_constrained_dofs();
  constraints_.push_back(std::move(c));  
}

template <typename T>
int SapContactProblem<T>::num_cliques() const {
  return A_.size();
}

template <typename T>
int SapContactProblem<T>::num_constraints() const {
  return constraints_.size();
}

template <typename T>
int SapContactProblem<T>::num_velocities() const {
  return nv_;
}

template <typename T>
int SapContactProblem<T>::num_velocities(int clique) const {
  DRAKE_DEMAND(0 <= clique && clique < num_cliques());
  return A_[clique].rows();
}

template <typename T>
const SapConstraint<T>& SapContactProblem<T>::get_constraint(int k) const {
  DRAKE_DEMAND(0 <= k && k < num_constraints());
  return *constraints_[k];
}

template <typename T>
const std::vector<MatrixX<T>>& SapContactProblem<T>::dynamics_matrix() const {
  return A_;
}

template <typename T>
ContactProblemGraph SapContactProblem<T>::MakeGraph() const {
  // We sort the constraint groups by clique pair lexicographically. This allow
  // us to provide an invariant when checking the correctness of the code. As an
  // additional invariant, this will us allow later on the enumerate
  // participating cliques in the order they were provided.
  // TODO(amcastro-tri): while these invariants are nice to have an
  // unordered_map might be more efficient. Try this if profiling reveals a
  // bottleneck here.
  std::map<SortedPair<int>, std::vector<int>> constraint_groups;
  for (size_t k = 0; k < constraints_.size(); ++k) {
    const auto& c = constraints_[k];
    const int c0 = c->clique0();  // N.B. we know that clique0() > 0 always.
    // The create a "loop" to signify a constraint within the same clique.
    const int c1 = c->clique1() >= 0 ? c->clique1() : c->clique0();
    const auto cliques_pair = drake::MakeSortedPair(c0, c1);
    constraint_groups[cliques_pair].push_back(k);
  }

  const int num_edges = constraint_groups.size();
  ContactProblemGraph graph(num_cliques(), num_edges);
  for (auto& e : constraint_groups) {
    graph.AddConstraintGroup(ContactProblemGraph::ConstraintGroup(e.first, std::move(e.second)));
  }

  return graph;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapContactProblem)
