#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
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
    if (i > 0)
      velocities_start_[i] = velocities_start_[i - 1] + num_velocities(i - 1);
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
std::unique_ptr<SapContactProblem<double>> SapContactProblem<T>::ToDouble()
    const {
  const double time_step = ExtractDoubleOrThrow(time_step_);
  std::vector<MatrixX<double>> A;
  A.reserve(A_.size());
  for (int i = 0; i < ssize(A_); ++i) {
    A.push_back(math::DiscardGradient(A_[i]));
  }
  VectorX<double> v_star = math::DiscardGradient(v_star_);
  auto clone = std::make_unique<SapContactProblem<double>>(
      time_step, std::move(A), std::move(v_star));
  clone->set_num_objects(num_objects());
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = get_constraint(i);
    clone->AddConstraint(c.ToDouble());
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
  DRAKE_DEMAND(ssize(per_clique_known_free_motion_dofs) <= num_cliques());
  for (int i = 0; i < ssize(per_clique_known_free_motion_dofs); ++i) {
    DRAKE_ASSERT_VOID(drake::multibody::internal::DemandIndicesValid(
        per_clique_known_free_motion_dofs[i], num_velocities(i)));
  }
  DRAKE_DEMAND(mapping != nullptr);

  mapping->velocity_permutation = PartialPermutation(num_velocities());
  mapping->clique_permutation = PartialPermutation(num_cliques());
  mapping->constraint_equation_permutation =
      PartialPermutation(num_constraint_equations());

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

  // Map all cliques possibly having known DoFs.
  for (int i = 0; i < ssize(per_clique_known_free_motion_dofs); ++i) {
    // Clique participates if at least one of its dofs is not locked.
    if (ssize(per_clique_known_free_motion_dofs[i]) < num_velocities(i)) {
      A_reduced.push_back(drake::multibody::internal::ExcludeRowsCols(
          A_[i], per_clique_known_free_motion_dofs[i]));
      mapping->clique_permutation.push(i);
    }
  }
  // Copy the rest of the cliques.
  for (int i = ssize(per_clique_known_free_motion_dofs); i < num_cliques();
       ++i) {
    A_reduced.push_back(A_[i]);
    mapping->clique_permutation.push(i);
  }

  // Construct a problem with reduced parameters.
  std::unique_ptr<SapContactProblem> problem =
      std::make_unique<SapContactProblem<T>>(time_step(), std::move(A_reduced),
                                             std::move(v_star_reduced));
  problem->set_num_objects(num_objects());

  // Make reduced constraints.
  int constraint_equation_index = 0;
  for (const auto& c : constraints_) {
    std::unique_ptr<SapConstraint<T>> c_reduced = c->MakeReduced(
        mapping->clique_permutation, per_clique_known_free_motion_dofs);

    if (c_reduced) {
      problem->AddConstraint(std::move(c_reduced));
      for (int j = 0; j < c->num_constraint_equations(); ++j) {
        mapping->constraint_equation_permutation.push(
            constraint_equation_index + j);
      }
    }
    constraint_equation_index += c->num_constraint_equations();
  }

  return problem;
}

template <typename T>
void SapContactProblem<T>::ExpandContactSolverResults(
    const ReducedMapping& reduced_mapping,
    const SapSolverResults<T>& reduced_results,
    SapSolverResults<T>* results) const {
  DRAKE_DEMAND(reduced_mapping.velocity_permutation.domain_size() ==
               num_velocities());
  DRAKE_DEMAND(reduced_mapping.clique_permutation.domain_size() ==
               num_cliques());
  DRAKE_DEMAND(reduced_mapping.constraint_equation_permutation.domain_size() ==
               num_constraint_equations());
  DRAKE_DEMAND(reduced_results.v.size() ==
               reduced_mapping.velocity_permutation.permuted_domain_size());
  DRAKE_DEMAND(
      reduced_results.gamma.size() ==
      reduced_mapping.constraint_equation_permutation.permuted_domain_size());
  DRAKE_DEMAND(
      reduced_results.vc.size() ==
      reduced_mapping.constraint_equation_permutation.permuted_domain_size());
  DRAKE_DEMAND(reduced_results.j.size() ==
               reduced_mapping.velocity_permutation.permuted_domain_size());
  DRAKE_DEMAND(results != nullptr);

  // Zero out everything. Results data will be selectively filled in.
  results->Resize(num_velocities(), num_constraint_equations());
  results->v = v_star();
  results->gamma.setZero();
  results->vc.setZero();
  results->j.setZero();

  // Set vc to vc* for known DoFs. Unknown DoFs will be overwritten below.
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = get_constraint(i);

    Eigen::VectorBlock<VectorX<T>> vc_segment = results->vc.segment(
        constraint_equations_start(i), c.num_constraint_equations());

    c.first_clique_jacobian().MultiplyAndAddTo(
        results->v.segment(velocities_start(c.first_clique()),
                           num_velocities(c.first_clique())),
        &vc_segment);

    if (c.num_cliques() > 1) {
      c.second_clique_jacobian().MultiplyAndAddTo(
          results->v.segment(velocities_start(c.second_clique()),
                             num_velocities(c.second_clique())),
          &vc_segment);
    }
  }

  // Copy v and j for participating velocities.
  reduced_mapping.velocity_permutation.ApplyInverse(reduced_results.v,
                                                    &results->v);
  reduced_mapping.velocity_permutation.ApplyInverse(reduced_results.j,
                                                    &results->j);

  // Copy gamma and vc for participating constraints.
  reduced_mapping.constraint_equation_permutation.ApplyInverse(
      reduced_results.gamma, &results->gamma);
  reduced_mapping.constraint_equation_permutation.ApplyInverse(
      reduced_results.vc, &results->vc);
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
    if (object_index < 0 || object_index >= num_objects()) {
      throw std::runtime_error(
          "Constraint object indices must be in the range [0, num_objects()).");
    }
  }

  // Update graph.
  const int ni = c->num_constraint_equations();
  const int constraint_index =
      c->num_cliques() == 1
          ? graph_.AddConstraint(c->first_clique(), ni)
          : graph_.AddConstraint(c->first_clique(), c->second_clique(), ni);

  // Starting index for the next constraint's velocity.
  constraint_equations_start_.push_back(constraint_equations_start_.back() +
                                        c->num_constraint_equations());

  constraints_.push_back(std::move(c));

  return constraint_index;
}

template <typename T>
void SapContactProblem<T>::CalcConstraintGeneralizedForces(
    const VectorX<T>& gamma, int constraint_start, int constraint_end,
    VectorX<T>* generalized_forces) const {
  DRAKE_THROW_UNLESS(0 <= constraint_start &&
                     constraint_start < num_constraints());
  DRAKE_THROW_UNLESS(0 <= constraint_end && constraint_end < num_constraints());
  DRAKE_THROW_UNLESS(constraint_start <= constraint_end);
  DRAKE_THROW_UNLESS(gamma.size() == num_constraint_equations());
  DRAKE_THROW_UNLESS(generalized_forces != nullptr);
  DRAKE_THROW_UNLESS(generalized_forces->size() == num_velocities());

  generalized_forces->setZero();
  for (int i = constraint_start; i <= constraint_end; ++i) {
    const SapConstraint<T>& constraint = get_constraint(i);
    const int equation_start = constraint_equations_start(i);
    const int ne = constraint.num_constraint_equations();
    const auto constraint_gamma = gamma.segment(equation_start, ne);

    // Compute generalized forces per clique.
    for (int c = 0; c < constraint.num_cliques(); ++c) {
      const int clique = constraint.clique(c);
      auto clique_forces = generalized_forces->segment(velocities_start(clique),
                                                       num_velocities(clique));
      constraint.AccumulateGeneralizedImpulses(c, constraint_gamma,
                                               &clique_forces);
    }
  }

  // Conversion of impulses into forces.
  (*generalized_forces) /= time_step();
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
    class ::drake::multibody::contact_solvers::internal::SapContactProblem);
