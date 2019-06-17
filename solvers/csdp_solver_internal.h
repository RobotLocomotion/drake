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
 * If SdpaFreeFormat.num_free_variables() == 0 (i.e., it doesn't have free
 * variable s), then convert the problem to CSDP problem data format.
 */
void GenerateCsdpProblemDataWithoutFreeVariables(
    const SdpaFreeFormat& sdpa_free_format, csdp::blockmatrix* C, double** b,
    csdp::constraintmatrix** constraints);

/**
 * For a problem
 * max tr(C * X)
 * s.t tr(Ai * X) = rhs_i
 *     X ≽ 0
 * We are given C, Ai, rhs in the Eigen sparse matrix format, convert these data
 * to CSDP format.
 */
void ConvertSparseMatrixFormatToCsdpProblemData(
    const std::vector<BlockInX>& X_blocks, const Eigen::SparseMatrix<double>& C,
    const std::vector<Eigen::SparseMatrix<double>> A,
    const Eigen::VectorXd& rhs, csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints);

void ConvertCsdpBlockMatrixtoEigen(const csdp::blockmatrix& X_csdp,
                                   Eigen::SparseMatrix<double>* X);

void FreeCsdpProblemData(int num_constraints, csdp::blockmatrix C_csdp,
                         double* rhs_csdp, csdp::constraintmatrix* constraints);

}  // namespace internal
}  // namespace solvers
}  // namespace drake
