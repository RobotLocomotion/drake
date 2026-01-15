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
  DRAKE_THROW_UNLESS(delassus_diagonal.size() ==
                     problem->num_constraint_equations());

  // Create vector of constraints but not in the order they were enumerated in
  // the SapProblem, but in the computationally convenient order enumerated in
  // the ContactProblemGraph, where constraints between the same
  // pair of cliques are "clustered" together.
  constraints_.reserve(problem->num_constraints());

  // Store constraints in the order specified by the graph, i.e. by clusters.
  for (const ContactProblemGraph::ConstraintCluster& e :
       problem->graph().clusters()) {
    for (int i : e.constraint_index()) {
      const SapConstraint<T>& c = problem->get_constraint(i);
      constraints_.push_back(&c);
    }
  }

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
SapConstraintBundleData SapConstraintBundle<T>::MakeData(
    const T& time_step, const VectorX<T>& delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal.size() == num_constraint_equations());
  SapConstraintBundleData data;
  data.reserve(num_constraints());
  int constraint_start = 0;
  // N.B. Recall constraints_ stores constraints in cluster order, and therefore
  // entries in delassus_diagonal are required to be in cluster order.
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constraint_equations();
    const auto wi = delassus_diagonal.segment(constraint_start, ni);
    data.emplace_back(c.MakeData(time_step, wi));
    constraint_start += ni;
  }
  return data;
}

template <typename T>
void SapConstraintBundle<T>::CalcData(
    const VectorX<T>& vc, SapConstraintBundleData* bundle_data) const {
  DRAKE_DEMAND(bundle_data != nullptr);
  DRAKE_DEMAND(ssize(*bundle_data) == num_constraints());
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constraint_equations();
    const auto vc_i = vc.segment(constraint_start, ni);
    AbstractValue& data = *(*bundle_data)[i];
    c.CalcData(vc_i, &data);
    constraint_start += ni;
  }
}

template <typename T>
T SapConstraintBundle<T>::CalcCost(
    const SapConstraintBundleData& bundle_data) const {
  DRAKE_DEMAND(ssize(bundle_data) == num_constraints());
  T cost = 0.0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const AbstractValue& data = *bundle_data[i];
    cost += c.CalcCost(data);
  }
  return cost;
}

template <typename T>
void SapConstraintBundle<T>::CalcImpulses(
    const SapConstraintBundleData& bundle_data, VectorX<T>* gamma) const {
  DRAKE_DEMAND(ssize(bundle_data) == num_constraints());
  DRAKE_DEMAND(gamma != nullptr);
  DRAKE_DEMAND(gamma->size() == num_constraint_equations());
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constraint_equations();
    const AbstractValue& data = *bundle_data[i];
    auto gamma_i = gamma->segment(constraint_start, ni);
    c.CalcImpulse(data, &gamma_i);
    constraint_start += ni;
  }
}

template <typename T>
void SapConstraintBundle<T>::CalcImpulsesAndConstraintsHessian(
    const SapConstraintBundleData& bundle_data, VectorX<T>* gamma,
    std::vector<MatrixX<T>>* G) const {
  DRAKE_DEMAND(ssize(bundle_data) == num_constraints());
  DRAKE_DEMAND(gamma != nullptr);
  DRAKE_DEMAND(gamma->size() == num_constraint_equations());
  DRAKE_DEMAND(ssize(*G) == num_constraints());

  // The regularizer Hessian is G = d²ℓ/dvc² = dP/dy⋅R⁻¹.
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constraint_equations();
    const AbstractValue& data = *bundle_data[i];
    auto gamma_i = gamma->segment(constraint_start, ni);
    auto& Gi = (*G)[i];
    c.CalcImpulse(data, &gamma_i);
    c.CalcCostHessian(data, &Gi);
    constraint_start += ni;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraintBundle);
