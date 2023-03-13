#include "drake/multibody/contact_solvers/sap/sap_constraint_bundle.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraintBundle<T>::SapConstraintBundle(
    const SapContactProblem<T>* problem, const VectorX<T>& delassus_diagonal) {
  DRAKE_THROW_UNLESS(problem != nullptr);
  DRAKE_THROW_UNLESS(delassus_diagonal.size() == problem->num_constraints());

  // Create vector of constraints but not in the order they were enumerated in
  // the SapProblem, but in the computationally convenient order enumerated in
  // the ContactProblemGraph, where constraints between the same
  // pair of cliques are "clustered" together.
  constraints_.reserve(problem->num_constraints());

  // Vector of bias velocities and diagonal matrix R.
  vhat_.resize(problem->num_constraint_equations());
  R_.resize(problem->num_constraint_equations());
  int impulse_index_start = 0;
  for (const ContactProblemGraph::ConstraintCluster& e :
       problem->graph().clusters()) {
    for (int i : e.constraint_index()) {
      const SapConstraint<T>& c = problem->get_constraint(i);
      constraints_.push_back(&c);

      const int ni = c.num_constraint_equations();
      const T& wi = delassus_diagonal[i];

      vhat_.segment(impulse_index_start, ni) =
          c.CalcBiasTerm(problem->time_step(), wi);
      R_.segment(impulse_index_start, ni) =
          c.CalcDiagonalRegularization(problem->time_step(), wi);

      impulse_index_start += ni;
    }
  }
  Rinv_ = R_.cwiseInverse();

  MakeConstraintBundleJacobian(*problem);
}

template <typename T>
int SapConstraintBundle<T>::num_constraints() const {
  return constraints_.size();
}

template <typename T>
int SapConstraintBundle<T>::num_constraint_equations() const {
  return J().rows();
}

template <typename T>
void SapConstraintBundle<T>::MakeConstraintBundleJacobian(
    const SapContactProblem<T>& problem) {
  const ContactProblemGraph& graph = problem.graph();
  const PartialPermutation& cliques_permutation = graph.participating_cliques();

  // We have at most two blocks per row, and one row per cluster (edge) in the
  // graph.
  const int non_zero_blocks_capacity = 2 * graph.num_clusters();
  BlockSparseMatrixBuilder<T> builder(
      graph.num_clusters(), cliques_permutation.permuted_domain_size(),
      non_zero_blocks_capacity);

  // Add a block row (with one or two blocks) per cluster of constraints in the
  // graph.
  for (int block_row = 0; block_row < graph.num_clusters(); ++block_row) {
    const ContactProblemGraph::ConstraintCluster& cluster =
        graph.get_cluster(block_row);
    // N.B. These are clique indexes in the original contact problem (including
    // both participating and non-participating cliques).
    const int c0 = cluster.cliques().first();
    const int c1 = cluster.cliques().second();

    // Allocate Jacobian blocks for this cluster of constraints.
    const int num_rows = cluster.num_total_constraint_equations();
    const int nv0 = problem.num_velocities(c0);
    const int nv1 = problem.num_velocities(c1);
    std::vector<MatrixBlock<T>> J0_blocks, J1_blocks;

    // Constraints are added in the order set by the graph.
    for (int i : cluster.constraint_index()) {
      const SapConstraint<T>& c = problem.get_constraint(i);
      // N.B. Each edge stores its cliques as a sorted pair. However, the pair
      // of cliques in the original constraints can be in arbitrary order.
      // Therefore below we must check to what clique in the original constraint
      // the group's cliques correspond to.
      J0_blocks.emplace_back(c0 == c.first_clique()
                                 ? c.first_clique_jacobian()
                                 : c.second_clique_jacobian());
      if (c1 != c0) {
        J1_blocks.emplace_back(c1 == c.first_clique()
                                   ? c.first_clique_jacobian()
                                   : c.second_clique_jacobian());
      }
    }

    MatrixBlock<T> J0 = StackMatrixBlocks(J0_blocks);
    DRAKE_DEMAND(J0.cols() == nv0);
    DRAKE_DEMAND(J0.rows() == num_rows);
    const int participating_c0 = cliques_permutation.permuted_index(c0);
    builder.PushBlock(block_row, participating_c0, std::move(J0));
    if (c1 != c0) {
      MatrixBlock<T> J1 = StackMatrixBlocks(J1_blocks);
      DRAKE_DEMAND(J1.cols() == nv1);
      DRAKE_DEMAND(J1.rows() == num_rows);
      const int participating_c1 = cliques_permutation.permuted_index(c1);
      builder.PushBlock(block_row, participating_c1, std::move(J1));
    }
  }

  J_ = builder.Build();
}

template <typename T>
void SapConstraintBundle<T>::CalcUnprojectedImpulses(const VectorX<T>& vc,
                                                     VectorX<T>* y) const {
  DRAKE_DEMAND(vc.size() == num_constraint_equations());
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(y->size() == num_constraint_equations());
  *y = Rinv_.asDiagonal() * (vhat_ - vc);
}

template <typename T>
void SapConstraintBundle<T>::ProjectImpulses(
    const VectorX<T>& y, VectorX<T>* gamma,
    std::vector<MatrixX<T>>* dPdy) const {
  DRAKE_DEMAND(y.size() == num_constraint_equations());
  DRAKE_DEMAND(gamma != nullptr);
  DRAKE_DEMAND(gamma->size() == num_constraint_equations());
  if (dPdy != nullptr) {
    DRAKE_DEMAND(static_cast<int>(dPdy->size()) == num_constraints());
  }
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constraint_equations();
    const auto y_i = y.segment(constraint_start, ni);
    const auto R_i = R().segment(constraint_start, ni);
    auto gamma_i = gamma->segment(constraint_start, ni);
    if (dPdy != nullptr) {
      MatrixX<T>& dPdy_i = (*dPdy)[i];
      c.Project(y_i, R_i, &gamma_i, &dPdy_i);
    } else {
      c.Project(y_i, R_i, &gamma_i);
    }
    constraint_start += ni;
  }
}

template <typename T>
void SapConstraintBundle<T>::ProjectImpulsesAndCalcConstraintsHessian(
    const VectorX<T>& y, VectorX<T>* gamma, std::vector<MatrixX<T>>* G) const {
  DRAKE_DEMAND(y.size() == num_constraint_equations());
  DRAKE_DEMAND(gamma != nullptr);
  DRAKE_DEMAND(gamma->size() == num_constraint_equations());
  DRAKE_DEMAND(static_cast<int>(G->size()) == num_constraints());
  // G = dPdy after call to ProjectImpulses. We add in the R⁻¹ next.
  ProjectImpulses(y, gamma, G);

  // The regularizer Hessian is G = d²ℓ/dvc² = dP/dy⋅R⁻¹.
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constraint_equations();
    const auto Rinv_i = Rinv().segment(constraint_start, ni);
    const MatrixX<T>& dPdy_i = (*G)[i];
    (*G)[i] = dPdy_i * Rinv_i.asDiagonal();
    constraint_start += ni;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraintBundle)
