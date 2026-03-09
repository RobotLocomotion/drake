#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/matrix_block.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using BlockTriplet = BlockSparseMatrix<double>::BlockTriplet;

// A supernodal Cholesky solver for solving the symmetric positive definite
// system
//   H⋅x = b
// where H = M + Jᵀ G J. The matrices M and J are set at construction and the
// weight matrix G is set with SetWeightMatrix(), which can be called multiple
// times on a constructed object.
//
// Example use case:
//
//  auto solver = std::make_unique<ConcreteSolverType>( ... );
//  solver->SetWeightMatrix( ... );
//  solver->Factor();
//
//  // Solve H⋅x1 = b1.
//  x1 = solver->Solve(b1);
//  // Solve H⋅x2 = b2. This reuses the factorization (important for speed!).
//  x2 = solver->Solve(b2);
//
//  // Update weight matrix and refactor.
//  solver->SetWeightMatrix( ... );
//  solver->Factor();
//  // Solve H⋅x = b using updated factorization.
//  x = solver->Solve(b);
class SuperNodalSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuperNodalSolver);

  virtual ~SuperNodalSolver();

  // Sets the block-diagonal weight matrix G.  The block rows of J and G both
  // partition the set {1, 2, ..., num_rows(J)}. Similar to the mass_matrix,
  // the partition induced by G must refine the partition induced by J,
  // otherwise an exception is thrown.
  void SetWeightMatrix(const std::vector<Eigen::MatrixXd>& block_diagonal_G);

  // Returns the M + J^T G J as a dense matrix (for debugging).
  // Throws if Factor() has been called (since factorization
  // is done in place. Throws if SetWeightMatrix has not been called.
  Eigen::MatrixXd MakeFullMatrix() const;

  // Computes the supernodal LLT factorization. Returns true if factorization
  // succeeds, otherwise returns false.  Failure is triggered by an internal
  // failure of Eigen::LLT.  This can fail if, for instance, the input matrix M
  // + J^T G J is not positive definite. If failure is encountered, the user
  // should verify that the specified matrix M + J^T G J is positive definite
  // and not poorly conditioned.  Throws if SetWeightMatrix() has not been
  // called.
  bool Factor();

  // Solves the system H⋅x = b and returns x.
  // Throws if Factor() has not been called.
  Eigen::VectorXd Solve(const Eigen::VectorXd& b) const;

  // Solves the system H⋅x = b and writes the result in b.
  // Throws if Factor() has not been called.
  void SolveInPlace(Eigen::VectorXd* b) const;

  // Returns the size of the system being solved.
  int GetSize() const { return DoGetSize(); }

 protected:
  SuperNodalSolver() = default;

  // @name NVI implementations. Specific solvers must implement these
  // methods. Refer to the specific NVI documentation for details.
  // @{

  // Returns true iff the weight matrix is compatible with the Jacobian matrix
  // and the weight matrix is successfully set.
  virtual bool DoSetWeightMatrix(
      const std::vector<Eigen::MatrixXd>& block_diagonal_G) = 0;

  // @see MakeFullMatrix()
  virtual Eigen::MatrixXd DoMakeFullMatrix() const = 0;

  // Returns true iff the factorization is successful. See associated NVI
  // documentation for possible cause of failure for the factorization.
  virtual bool DoFactor() = 0;

  // `b` is guaranteed to be non-null and of correct size.
  virtual void DoSolveInPlace(Eigen::VectorXd* b) const = 0;

  // @see GetSize()
  virtual int DoGetSize() const = 0;

  // @}

 private:
  bool factorization_ready_ = false;
  bool matrix_ready_ = false;
};

// Returns a row to triplet index mapping as std::vector<std::vector<int>> and
// perform a validation check on the input `jacobian_blocks`.
//
// Each entry in `jacobian_blocks` is a triplet of the form (i, j, J), where J
// is the Jacobian block and i and j are its block row/column indices in the
// sparse Jacobian. The entries in `jacobian_blocks` must satisfy the
// following property:
//  1. {T.i | T ∈ `jacobian_blocks`} = {0, 1, ..., num_row_blocks-1} and
//  2. For each k in {0, 1, ..., num_row_blocks - 1}, 1 <= |{T|T.i==k}| <= 2.
// where we use T.i to denote the block row index of a triplet.
// Otherwise, throws an exception.
//
// The k-th entry in the result stores a vector of indices into
// `jacobian_blocks` that correspond to entries that has k as the block row
// index in the triplet. Each entry in the resulting vector contains at least
// one and at most two entries due to the precondtion on `jacobian_blocks`. */
std::vector<std::vector<int>> GetRowToTripletMapping(
    int num_row_blocks, const std::vector<BlockTriplet>& jacobian_blocks);

// Verifies the entries in `jacobian_blocks` are valid in the sense that they
// satisfy:
//  1. {T.j | T ∈ `jacobian_blocks`} = {0, 1, ..., max T.j}, and
//  2. if jacobian_blocks[a].j == jacobian_blocks[b].j, then
//  jacobian_blocks[a].J.cols() == jacobian_blocks[b].J.cols(), and
//  3. for each triplet T, T.J.rows() > 0 && T.J.cols() > 0.
// If not, throws an exception.
// Returns the number of columns in each column block of the Jacobian matrix.
std::vector<int> GetJacobianBlockSizesVerifyTriplets(
    const std::vector<BlockTriplet>& jacobian_blocks);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
