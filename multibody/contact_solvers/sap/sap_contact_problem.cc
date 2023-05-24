#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/plant/slicing_and_indexing.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapContactProblem<T>::SapContactProblem(const T& time_step)
    : time_step_(time_step) {
  DRAKE_THROW_UNLESS(time_step > 0.0);
}

template <typename T>
SapContactProblem<T>::SapContactProblem(const T& time_step,
                                        std::vector<MatrixX<T>> A,
                                        VectorX<T> v_star)
    : time_step_(time_step),
      A_(std::move(A)),
      v_star_(std::move(v_star)),
      graph_(num_cliques()) {
  DRAKE_THROW_UNLESS(time_step > 0.0);
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_THROW_UNLESS(Ac.size() >= 0);
    DRAKE_THROW_UNLESS(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_THROW_UNLESS(v_star_.size() == nv_);
}

template <typename T>
void SapContactProblem<T>::Reset(std::vector<MatrixX<T>> A, VectorX<T> v_star) {
  A_ = std::move(A);
  v_star_ = std::move(v_star);
  graph_.ResetNumCliques(num_cliques());
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_THROW_UNLESS(Ac.size() >= 0);
    DRAKE_THROW_UNLESS(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_THROW_UNLESS(v_star_.size() == nv_);
  constraints_.clear();
}

template <typename T>
std::unique_ptr<SapContactProblem<T>> SapContactProblem<T>::Clone() const {
  auto clone = std::make_unique<SapContactProblem<T>>(time_step_, A_, v_star_);
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = get_constraint(i);
    clone->AddConstraint(c.Clone());
  }
  return clone;
}

template <typename T>
ReducedMapping SapContactProblem<T>::Reduce(
    std::vector<int> locked_indices,
    std::vector<std::vector<int>> clique_locked_indices) {
  const int num_cliques = static_cast<int>(clique_locked_indices.size());
  const int num_constraints = this->constraints_.size();
  ReducedMapping mapping{
      .clique_permutation = PartialPermutation(num_cliques),
      .constraint_permutation = PartialPermutation(num_constraints)};

  // Project v_star and A matrices to reduced DOFs.
  VectorX<T> v_star_reduced =
      drake::multibody::internal::ExcludeRows(v_star_, locked_indices);

  std::vector<MatrixX<T>> A_reduced;
  A_reduced.reserve(A_.size());
  for (int i = 0; i < num_cliques; ++i) {
    // Clique participates if at least one of its dofs is not locked.
    if (static_cast<int>(clique_locked_indices[i].size()) < A_[i].rows()) {
      A_reduced.push_back(drake::multibody::internal::ExcludeRowsCols(
          A_[i], clique_locked_indices[i]));
      mapping.clique_permutation.push(i);
    }
  }

  A_reduced.resize(mapping.clique_permutation.permuted_domain_size());

  // Grab the constraints before Reset() clears the vector.
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints =
      std::move(this->constraints_);

  this->Reset(A_reduced, v_star_reduced);

  // Project constraints to reduced DOFs. If all DOFs of a clique have been
  // eliminated its jacobian becomes zero size and no longer contributes,
  // therefore we eliminate that clique from the constraint. In the case where
  // both cliques have 0 DOF, we completely eliminate the constraint from the
  // reduced contact problem.
  for (int i = 0; i < num_constraints; ++i) {
    std::unique_ptr<SapConstraint<T>> c = std::move(constraints[i]);

    MatrixBlock<T> jacobians[2];
    int cliques[2];
    int idx = 0;

    // If the first clique participates, include it and project its jacobian.
    if (mapping.clique_permutation.participates(c->first_clique())) {
      cliques[idx] =
          mapping.clique_permutation.permuted_index(c->first_clique());
      jacobians[idx] = drake::multibody::internal::ExcludeCols(
          c->first_clique_jacobian(), clique_locked_indices[c->first_clique()]);
      ++idx;
    }

    // If the original constraint involved a second clique and that clique
    // participates, include it and project its jacobian.
    if (c->num_cliques() > 1 &&
        mapping.clique_permutation.participates(c->second_clique())) {
      cliques[idx] =
          mapping.clique_permutation.permuted_index(c->second_clique());
      jacobians[idx] = drake::multibody::internal::ExcludeCols(
          c->second_clique_jacobian(),
          clique_locked_indices[c->second_clique()]);
      ++idx;
    }

    // Determine the clique contributions and reset the jacobians to their
    // reduced versions.
    if (idx > 0) {
      c->set_first_clique(cliques[0]);
      c->set_first_clique_jacobian(jacobians[0]);
      if (idx > 1) {
        c->set_second_clique(cliques[1]);
        c->set_second_clique_jacobian(jacobians[1]);
      } else {
        c->set_second_clique(-1);
      }
      this->AddConstraint(std::move(c));
      mapping.constraint_permutation.push(i);
    }
  }

  return mapping;
}

template <typename T>
int SapContactProblem<T>::AddConstraint(std::unique_ptr<SapConstraint<T>> c) {
  if (c->first_clique() >= num_cliques()) {
    throw std::runtime_error(
        "First clique index must be strictly lower than num_cliques()");
  }
  if (c->num_cliques() == 2 && c->second_clique() >= num_cliques()) {
    throw std::runtime_error(
        "Second clique index must be strictly lower than num_cliques()");
  }
  if (c->first_clique_jacobian().cols() != num_velocities(c->first_clique())) {
    throw std::runtime_error(
        "The number of columns in the constraint's "
        "Jacobian does not match the number of velocities in this problem for "
        "the first clique.");
  }
  if (c->num_cliques() == 2 && c->second_clique_jacobian().cols() !=
                                   num_velocities(c->second_clique())) {
    throw std::runtime_error(
        "The number of columns in the constraint's "
        "Jacobian does not match the number of velocities in this problem for "
        "the second clique.");
  }
  if (num_velocities(c->first_clique()) == 0 ||
      (c->num_cliques() == 2 && num_velocities(c->second_clique()) == 0)) {
    throw std::runtime_error(
        "Adding constraint to a clique with zero number of velocities is not "
        "allowed.");
  }

  // Update graph.
  const int ni = c->num_constraint_equations();
  const int constraint_index =
      c->num_cliques() == 1
          ? graph_.AddConstraint(c->first_clique(), ni)
          : graph_.AddConstraint(c->first_clique(), c->second_clique(), ni);

  constraints_.push_back(std::move(c));

  return constraint_index;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapContactProblem)
