#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

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
