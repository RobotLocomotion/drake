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

// TODO(sap_joint_locking): Implement this function.
template <typename T>
void SapContactProblem<T>::ReduceToSelectedDofs(
    SapContactProblem<T>* problem, std::vector<int> indices,
    std::vector<std::vector<int>> indices_per_tree) const {
  DRAKE_DEMAND(problem != nullptr);
  DRAKE_DEMAND(problem->time_step_ == this->time_step_);

  VectorX<T> v_star_reduced =
      drake::multibody::internal::SelectRows(v_star_, indices);

  std::vector<MatrixX<T>> A_reduced(A_.size());
  for (int i = 0; i < static_cast<int>(indices_per_tree.size()); ++i) {
    A_reduced[i] =
        drake::multibody::internal::SelectRowsCols(A_[i], indices_per_tree[i]);
  }

  problem->Reset(A_reduced, v_star_reduced);

  // Joint locking does not change any deformable dofs, therefore any cliques
  // with non-dense Jacobians do not need to be reduced.
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = get_constraint(i);
    std::unique_ptr<SapConstraint<T>> c_clone = c.Clone();

    MatrixBlock<T> jacobians[2];
    int cliques[2];
    int idx = 0;

    if (!c.first_clique_jacobian().is_dense()) {
      cliques[idx] = c.first_clique();
      jacobians[idx] = c.first_clique_jacobian();
      ++idx;
    } else if (indices_per_tree[c.first_clique()].size() > 0) {
      cliques[idx] = c.first_clique();
      jacobians[idx] = drake::multibody::internal::SelectCols(
          c.first_clique_jacobian(), indices_per_tree[c.first_clique()]);
      ++idx;
    }

    if (c.num_cliques() > 1) {
      if (!c.second_clique_jacobian().is_dense()) {
        cliques[idx] = c.second_clique();
        jacobians[idx] = c.second_clique_jacobian();
        ++idx;
      } else if (indices_per_tree[c.second_clique()].size() > 0) {
        cliques[idx] = c.second_clique();
        jacobians[idx] = drake::multibody::internal::SelectCols(
            c.second_clique_jacobian(), indices_per_tree[c.second_clique()]);
        ++idx;
      }
    }

    if (idx > 0) {
      c_clone->set_first_clique(cliques[0]);
      c_clone->set_first_clique_jacobian(jacobians[0]);
      if (idx > 1) {
        c_clone->set_first_clique(cliques[1]);
        c_clone->set_first_clique_jacobian(jacobians[1]);
      } else {
        c_clone->set_second_clique(-1);
      }
      problem->AddConstraint(std::move(c_clone));
    }
  }
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
