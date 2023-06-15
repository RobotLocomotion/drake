#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ssize.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/plant/slicing_and_indexing.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapContactProblem<T>::SapContactProblem(const T& time_step,
                                        std::vector<MatrixX<T>> A,
                                        VectorX<T> v_star)
    : time_step_(time_step),
      A_(std::move(A)),
      v_star_(std::move(v_star)),
      graph_(num_cliques()) {
  DRAKE_THROW_UNLESS(time_step > 0.0);
  velocities_start_.resize(A_.size(), 0);
  nv_ = 0;
  for (int i = 0; i < ssize(A_); ++i) {
    const auto& Ac = A_[i];
    DRAKE_THROW_UNLESS(Ac.rows() == Ac.cols());
    const int clique_nv = Ac.rows();
    if (i > 0) velocities_start_[i] = velocities_start_[i - 1] + clique_nv;
    nv_ += Ac.rows();
  }
  DRAKE_THROW_UNLESS(v_star_.size() == nv_);
}

template <typename T>
void SapContactProblem<T>::set_num_objects(int num_objects) {
  DRAKE_THROW_UNLESS(num_constraints() == 0);
  num_objects_ = num_objects;
}

template <typename T>
std::unique_ptr<SapContactProblem<T>> SapContactProblem<T>::Clone() const {
  auto clone = std::make_unique<SapContactProblem<T>>(time_step_, A_, v_star_);
  clone->set_num_objects(num_objects());
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = get_constraint(i);
    clone->AddConstraint(c.Clone());
  }
  return clone;
}

template <typename T>
std::unique_ptr<SapContactProblem<T>> SapContactProblem<T>::MakeReduced(
    const std::vector<int>& known_free_motion_dofs,
    const std::vector<std::vector<int>>& per_clique_known_free_motion_dofs,
    ReducedMapping* mapping) const {
  DRAKE_ASSERT_VOID(drake::multibody::internal::DemandIndicesValid(
      known_free_motion_dofs, num_velocities()));
  DRAKE_DEMAND(ssize(per_clique_known_free_motion_dofs) == num_cliques());
  for (int i = 0; i < num_cliques(); ++i) {
    DRAKE_ASSERT_VOID(drake::multibody::internal::DemandIndicesValid(
        per_clique_known_free_motion_dofs[i], num_velocities(i)));
  }
  DRAKE_DEMAND(mapping != nullptr);

  mapping->velocity_permutation = PartialPermutation(num_velocities());
  mapping->clique_permutation = PartialPermutation(num_cliques());
  mapping->constraint_permutation = PartialPermutation(num_constraints());

  auto it = known_free_motion_dofs.begin();
  for (int i = 0; i < num_velocities(); ++i) {
    if (it != known_free_motion_dofs.end() && i == *it) {
      ++it;
    } else {
      mapping->velocity_permutation.push(i);
    }
  }

  // Project v_star and A matrices to reduced DOFs.
  VectorX<T> v_star_reduced =
      drake::multibody::internal::ExcludeRows(v_star_, known_free_motion_dofs);

  std::vector<MatrixX<T>> A_reduced;
  A_reduced.reserve(A_.size());
  for (int i = 0; i < num_cliques(); ++i) {
    // Clique participates if at least one of its dofs is not locked.
    if (ssize(per_clique_known_free_motion_dofs[i]) < num_velocities(i)) {
      A_reduced.push_back(drake::multibody::internal::ExcludeRowsCols(
          A_[i], per_clique_known_free_motion_dofs[i]));
      mapping->clique_permutation.push(i);
    }
  }

  // Construct a problem with reduced parameters.
  std::unique_ptr<SapContactProblem> problem =
      std::make_unique<SapContactProblem<T>>(time_step(), std::move(A_reduced),
                                             std::move(v_star_reduced));
  problem->set_num_objects(num_objects());

  // Make reduced constraints.
  for (int i = 0; i < num_constraints(); ++i) {
    std::unique_ptr<SapConstraint<T>> c = get_constraint(i).MakeReduced(
        mapping->clique_permutation, per_clique_known_free_motion_dofs);

    if (c) {
      mapping->constraint_permutation.push(i);
      problem->AddConstraint(std::move(c));
    }
  }

  return problem;
}

template <typename T>
void SapContactProblem<T>::ExpandContactSolverResults(
    const contact_solvers::internal::ReducedMapping& reduced_mapping,
    const contact_solvers::internal::SapSolverResults<T>& reduced_results,
    contact_solvers::internal::SapSolverResults<T>* sap_results) const {
  DRAKE_DEMAND(reduced_mapping.velocity_permutation.domain_size() ==
               num_velocities());
  DRAKE_DEMAND(reduced_mapping.clique_permutation.domain_size() ==
               num_cliques());
  DRAKE_DEMAND(reduced_mapping.constraint_permutation.domain_size() ==
               num_constraints());
  DRAKE_DEMAND(sap_results != nullptr);

  // Zero out everything. Results data will be selectively filled in.
  sap_results->Resize(num_velocities(), num_constraint_equations());
  sap_results->v.setZero();
  sap_results->gamma.setZero();
  sap_results->vc.setZero();
  sap_results->j.setZero();

  // Copy v and j for participating velocities.
  for (int i = 0; i < num_velocities(); ++i) {
    if (reduced_mapping.velocity_permutation.participates(i)) {
      sap_results->v[i] =
          reduced_results
              .v[reduced_mapping.velocity_permutation.permuted_index(i)];
      sap_results->j[i] =
          reduced_results
              .j[reduced_mapping.velocity_permutation.permuted_index(i)];
    }
  }

  // Copy gamma and vc for participating constraints.
  int equation_index = 0;
  int reduced_equation_index = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = get_constraint(i);
    if (reduced_mapping.constraint_permutation.participates(i)) {
      sap_results->gamma.segment(equation_index, c.num_constraint_equations()) =
          reduced_results.gamma.segment(reduced_equation_index,
                                        c.num_constraint_equations());
      sap_results->vc.segment(equation_index, c.num_constraint_equations()) =
          reduced_results.vc.segment(reduced_equation_index,
                                     c.num_constraint_equations());
      reduced_equation_index += c.num_constraint_equations();
    }
    equation_index += c.num_constraint_equations();
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

  for (int object_index : c->objects()) {
    if (object_index < 0 || object_index >= num_objects())
      throw std::runtime_error(
          "Constraint object indices must be in the range [0, num_objects()).");
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

template <typename T>
void SapContactProblem<T>::CalcConstraintMultibodyForces(
    const VectorX<T>& gamma, VectorX<T>* generalized_forces,
    std::vector<SpatialForce<T>>* spatial_forces) const {
  DRAKE_THROW_UNLESS(gamma.size() == num_constraint_equations());
  DRAKE_THROW_UNLESS(generalized_forces != nullptr);
  DRAKE_THROW_UNLESS(generalized_forces->size() == num_velocities());
  DRAKE_THROW_UNLESS(spatial_forces != nullptr);
  DRAKE_THROW_UNLESS(ssize(*spatial_forces) == num_objects());
  // Initialize to zero before accumulating constraint contributions.
  generalized_forces->setZero();
  for (SpatialForce<T>& F : *spatial_forces) {
    F.SetZero();
  }
  int offset = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& constraint = get_constraint(i);
    const int ne = constraint.num_constraint_equations();
    const auto constraint_gamma = gamma.segment(offset, ne);

    // Compute generalized forces per clique.
    for (int c = 0; c < constraint.num_cliques(); ++c) {
      const int clique = constraint.clique(c);
      auto clique_forces = generalized_forces->segment(velocities_start(clique),
                                                       num_velocities(clique));
      constraint.AccumulateGeneralizedImpulses(c, constraint_gamma,
                                               &clique_forces);
    }

    // Accumulate spatial forces per object.
    for (int o = 0; o < constraint.num_objects(); ++o) {
      const int object = constraint.object(o);
      SpatialForce<T>& F = (*spatial_forces)[object];
      constraint.AccumulateSpatialImpulses(o, constraint_gamma, &F);
    }

    offset += ne;
  }

  // Conversion of impulses into forces.
  (*generalized_forces) /= time_step();
  for (SpatialForce<T>& F : *spatial_forces) {
    F.get_coeffs() /= time_step();
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapContactProblem)
