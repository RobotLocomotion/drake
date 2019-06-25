#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to csdp_solver_internal_test.cc

#include <unordered_map>
#include <vector>

namespace csdp {
extern "C" {
// TODO(Jeremy.Nimmer): include this header as <csdp/declarations.h>
#include <declarations.h>
}  // extern C
}  // namespace csdp

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/sdpa_free_format.h"

namespace drake {
namespace solvers {
namespace internal {
/**
 * For the following problem
 *
 *     max tr(C * X) + dᵀs
 *     s.t tr(Aᵢ*X) + bᵢᵀs = aᵢ
 *         X ≽ 0
 *         s is free,
 *
 * remove the free variable s by considering the nullspace of Bᵀ.
 *
 *     max tr((C-∑ᵢ ŷᵢAᵢ)*X̂) + aᵀŷ
 *     s.t tr(FᵢX̂) = (Nᵀa)(i)
 *         X̂ ≽ 0,
 * where Fᵢ  = ∑ⱼ NⱼᵢAⱼ, N is the null space of Bᵀ. Bᵀ * ŷ = d.
 * For more information, refer to CsdpSolver::RemoveFreeVariableMethod for the
 * derivation.
 */
void RemoveFreeVariableByNullspaceApproach(
    const SdpaFreeFormat& sdpa_free_format, Eigen::SparseMatrix<double>* C_hat,
    std::vector<Eigen::SparseMatrix<double>>* A_hat, Eigen::VectorXd* rhs_hat,
    Eigen::VectorXd* y_hat,
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>*
        QR_B);

/**
 * For a problem
 * max tr(C * X)
 * s.t tr(Ai * X) = rhs_i
 *     X ≽ 0
 * We are given C, Ai, rhs in the Eigen sparse matrix format, convert these data
 * to CSDP format.
 * This function allocates memory for the CSDP solver, by allocating memory to
 * @p C_csdp, @p rhs_csdp and @p constraints. The memory should be cleared by
 * calling FreeCsdpProblemData().
 */
void ConvertSparseMatrixFormatToCsdpProblemData(
    const std::vector<BlockInX>& X_blocks, const Eigen::SparseMatrix<double>& C,
    const std::vector<Eigen::SparseMatrix<double>> A,
    const Eigen::VectorXd& rhs, csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints);

/**
 * Converts to a CSDP problem data if `sdpa_free_format` has no free variables.
 * @throw a runtime error if sdpa_free_format has free variables.
 */
void GenerateCsdpProblemDataWithoutFreeVariables(
    const SdpaFreeFormat& sdpa_free_format, csdp::blockmatrix* C, double** b,
    csdp::constraintmatrix** constraints);

void ConvertCsdpBlockMatrixtoEigen(const csdp::blockmatrix& X_csdp,
                                   Eigen::SparseMatrix<double>* X);

void FreeCsdpProblemData(int num_constraints, csdp::blockmatrix C_csdp,
                         double* rhs_csdp, csdp::constraintmatrix* constraints);

/**
 * Csdp internally stores each block of the matrix as an array. This function
 * takes the row and column index inside this block (the row/column indices are
 * 0-indexed), and return the index of the entry in the array.
 */
int CsdpMatrixIndex(int row, int col, int num_rows);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
