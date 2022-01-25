#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap_contact_problem.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// For a given contact graph, not all of the cliques participate in a
// constraint. This method identifies those cliques in `graph` that appear at
// least once at one of the graph's edges.
PartialPermutation MakeParticipatingCliquesPermutation(
    const ContactProblemGraph& graph);

// TODO: this might be better to be a SapSolver method. `graph` must make
// reference to all constraints in `problem`. However, `graph` might only make
// reference to a subset of cliques in `problem` (for participating cliques
// only.)
template <typename T>
BlockSparseMatrix<T> MakeConstraintsBundleJacobian(
    const SapContactProblem<T>& problem,
    const ContactProblemGraph& graph,
    const PartialPermutation& cliques_permutation) {
  // We have at most two blocks per row, and one row per edge in the graph.
  const int non_zero_blocks_capacity = 2 * graph.num_constraint_groups();
  BlockSparseMatrixBuilder<T> builder(
      graph.num_constraint_groups(), cliques_permutation.permuted_domain_size(),
      non_zero_blocks_capacity);

  // DOFs per edge.
  // TODO: consider the graph storing "weights"; number of dofs per node
  // (clique) and number of constrained dofs per edge.
  std::vector<int> edge_dofs(graph.num_constraint_groups(), 0);
  for (int e = 0; e < graph.num_constraint_groups(); ++e) {
    const auto& edge = graph.get_constraint_group(e);
    for (int k : edge.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      edge_dofs[e] += c.num_constrained_dofs();
    }
  }                                      

  // Add a block row (with one or two blocks) per edge in the graph.
  for (int block_row = 0; block_row < graph.num_constraint_groups(); ++block_row) {
    const auto& e = graph.get_constraint_group(block_row);
    const int c0 = e.cliques.first();
    const int c1 = e.cliques.second();

    // Allocate Jacobian blocks for this edge.
    MatrixX<T> J0, J1;
    const int num_rows = edge_dofs[block_row];
    if (c0 >= 0) {
      //  && cliques_permutation.participates(c0)
      PRINT_VAR(c0);  
      const int nv0 = problem.num_velocities(c0);
      J0.resize(num_rows, nv0);
    }
    DRAKE_DEMAND(c1 >= 0);
    PRINT_VAR(c1);
    const int nv1 = problem.num_velocities(c1);
    J1.resize(num_rows, nv1);

    int row_start = 0;
    for (int k : e.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      const int nk = c.num_constrained_dofs();

      // N.B. Each edge stores its cliques as a sorted pair. However, the pair
      // of cliques in the original constraints can be in arbitrary order.
      // Therefore below we must check to what clique in the original constraint
      // the edge's cliques correspond to.

      if (c0 >= 0) {
        J0.middleRows(row_start, nk) =
            c0 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();
      }
      J1.middleRows(row_start, nk) =
          c1 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();

      row_start += nk;
    }

    if (c0 >= 0) {
      const int participating_c0 = cliques_permutation.permuted_index(c0);
      builder.PushBlock(block_row, participating_c0, J0);
    }
    const int participating_c1 = cliques_permutation.permuted_index(c1);
    builder.PushBlock(block_row, participating_c1, J1);
  }

  return builder.Build();

}

// TODO: this might be better to be a SapSolver method. `graph` must make
// reference to all constraints in `problem`. However, `graph` might only make
// reference to a subset of cliques in `problem` (for participating cliques
// only.)
template <typename T>
BlockSparseMatrix<T> MakeConstraintsBundleJacobian(
    const SapContactProblem<T>& problem,
    const std::vector<int>& participating_cliques,
    const ContactProblemGraph& participating_cliques_graph) {
  // We have at most two blocks per row, and one row per edge in the graph.
  const int non_zero_blocks_capacity =
      2 * participating_cliques_graph.num_constraint_groups();
  BlockSparseMatrixBuilder<T> builder(participating_cliques_graph.num_constraint_groups(),
                                      participating_cliques_graph.num_cliques(),
                                      non_zero_blocks_capacity);

  std::vector<int> edge_dofs(participating_cliques_graph.num_constraint_groups(), 0);
  for (int e = 0; e < participating_cliques_graph.num_constraint_groups(); ++e) {
    const auto& edge = participating_cliques_graph.get_constraint_group(e);
    for (int k : edge.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      edge_dofs[e] += c.num_constrained_dofs();
    }
  }

  for (int block_row = 0; block_row < participating_cliques_graph.num_constraint_groups();
       ++block_row) {
    const auto& e = participating_cliques_graph.get_constraint_group(block_row);
    const int participating_c0 = e.cliques.first();
    const int participating_c1 = e.cliques.second();
    const int c0 =
        participating_c0 >= 0 ? participating_cliques[participating_c0] : -1;
    // At least one clique must be valid per graph edge.
    // Since c0 < c1, then c1 must always be valid.
    DRAKE_DEMAND(participating_c1 >= 0);
    const int c1 = participating_cliques[participating_c1];

    // Allocate Jacobian blocks for this edge.
    MatrixX<T> J0, J1;
    const int num_rows = edge_dofs[block_row];
    if (c0 >= 0) {
      const int nv0 = problem.num_velocities(c0);
      J0.resize(num_rows, nv0);
    }
    const int nv1 = problem.num_velocities(c1);
    J1.resize(num_rows, nv1);

    int row_start = 0;
    for (int k : e.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      const int nk = c.num_constrained_dofs();

      // N.B. Each edge stores its cliques as a sorted pair. However, the pair
      // of cliques in the original constraints can be in arbitrary order.
      // Therefore below we must check to what clique in the original constraint
      // the edge's cliques correspond to.

      if (c0 >= 0) {
        J0.middleRows(row_start, nk) =
            c0 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();
      }

      J1.middleRows(row_start, nk) =
          c1 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();

      row_start += nk;
    }

    if (c0 >= 0) {
      builder.PushBlock(block_row, participating_c0, J0);
    }
    builder.PushBlock(block_row, participating_c1, J1);
  }

  return builder.Build();
}

template <typename T>
std::vector<MatrixX<T>> ExtractParticipatingDynamics(
    const SapContactProblem<T>& problem, const ContactProblemGraph& graph,
    const PartialPermutation& cliques_permutation) {
  const auto& A = problem.dynamics_matrix();
  std::vector<MatrixX<T>> A_participating(
      cliques_permutation.permuted_domain_size());
  cliques_permutation.Apply(A, &A_participating);
  return A_participating;
}

// TODO: Move this to the SapSolver source. It'd seem specific to SAP to
// rearrange constraints in the order it needs them. I just need to come up with
// a good way to document it and describe its invariants.
template <typename T>
class SapConstraintsBundle {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraintsBundle);

  // We keep a reference to `problem` and its data.
  SapConstraintsBundle(BlockSparseMatrix<T>&& J,
                       std::vector<SapConstraint<T>*>&& constraints)
      : J_(std::move(J)), constraints_(std::move(constraints)) {}

  int num_constraints() const { return constraints_.size(); }      

  void CalcConstraintVelocities(const VectorX<T>& v, VectorX<T>* vc) const {
    J_.Multiply(v, &vc);
  }

  // TODO: Consider this class storing R. That why R already would be in the
  // proper order.
  void Project(const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
               std::vector<MatrixX<T>>* dPdy) {
    int constraint_start = 0;
    for (int k = 0; k < num_constraints(); ++k) {
      const SapConstraint<T>* c = constraints_[k];
      const int nk = c->num_constrained_dofs();
      const auto y_k = y.segment(constraint_start, nk);
      const auto R_k = R.segment(constraint_start, nk);
      auto gamma_k = gamma.segment(constraint_start, nk);
      if (dPdy != nullptr) {
        MatrixX<T>& dPdy_k = (*dPdy)[k];
        c->Project(y_k, R_k, &gamma_k, &dPdy_k);
      } else {
        c->Project(y_k, R_k, &gamma_k);
      }
      constraint_start += nk;
    }
  }

  void CalcProjectImpulsesAndCalcConstraintsHessian(
      const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
      std::vector<MatrixX<T>>* G) {
    Project(y, R, gamma, G);  // G = dPdy.

    // The regularizer Hessian is G = ∇²ℓ = d²ℓ/dvc² = dP/dy⋅R⁻¹.
    int constraint_start = 0;
    for (int k = 0; k < num_constraints(); ++k) {
      const SapConstraint<T>* c = constraints_[k];
      const int nk = c->num_constrained_dofs();      
      const auto R_k = R.segment(constraint_start, nk);
      const MatrixUpTo6<T> dPdy_k = (*G)[k];
      (*G)[k] = dPdy_k * R_k.cwiseInverse().asDiagonal();
      constraint_start += nk;
    }
  }

 private:
  // Jacobian for the entire bundle, with graph_.num_constraint_groups() block rows and
  // graph_.num_cliques() block columns.
  BlockSparseMatrix<T> J_;
  // problem_ constraint references in the order dictated by the
  // ContactProblemGraph.
  std::vector<SapConstraint<T>*> constraints_;
};

template <typename T>
std::unique_ptr<SapConstraintsBundle<T>> MakeConstraintsBundle(
    const SapContactProblem<T>& problem,
    const ContactProblemGraph& graph,
    const PartialPermutation& cliques_permutation) {
  const BlockSparseMatrix<T> J =
      MakeConstraintsBundleJacobian(problem, graph, cliques_permutation);

  // Bundle constraints in the order specified in the graph (where constraints
  // are grouped by clique pairs)
  std::vector<SapConstraint<T>*> constraints(problem.num_constraints());
  for (const auto& e : graph.constraint_groups()) {
    for (int k : e.constraints_index){
      const auto& c = problem.get_constraint(k);
      constraints[k] = &c;
    }
  }

  return std::make_unique<SapConstraintsBundle<T>>(std::move(J),
                                                   std::move(constraints));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
