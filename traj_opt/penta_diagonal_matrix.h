#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace traj_opt {
namespace internal {

/** A sparse representation of a (square) banded penta-diagonal matrix. Denoting
 with A and B the lower sub-diagonals, with C the matrix's main diagonal and
 with D and E the upper sub-diagonals, a banded penta-diagonal matrix M of n×n
 blocks of size k×k each takes the form:

       [ C₀ D₀ E₀ 0  0  0  0  0 ... 0 ]
       [ B₁ C₁ D₁ E₁ 0  0  0  0 ... 0 ]
       [ A₂ B₂ C₂ D₂ E₂ 0  0  0 ... 0 ]
       [              ...             ]
   M = [              ...             ]
       [ 0 ... Aᵢ Bᵢ Cᵢ Dᵢ Eᵢ 0 ... 0 ]
       [              ...             ]
       [              ...             ]
       [ 0    ...    0 Aₙ₋₁ Bₙ₋₁ Cₙ₋₁ ]

 Notice that all blocks are of the same size k×k and that our indexing notation
 is defined such that the i-th block row is formed by blocks Aᵢ, Bᵢ, Cᵢ, Dᵢ and
 Eᵢ. Notice that blocks A₀, B₀, A₁, Eₙ₋₂, Dₙ₋₁, Eₙ₋₁ are not part of the matrix.
 However this class does store them and initializes them to zero for convenience
 when writting alorithms that operate on this matrix. */
template <typename T>
class PentaDiagonalMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PentaDiagonalMatrix);

  /* Constructor for a pentadiagonal matrix with num_blocks rows/columns blocks
  of size block_size x block_size each. That is, the main diagonal vector C()
  will have size num_blocks and rows() will equal num_blocks*block_size.

  The matrix will be symmetric if is_symmetric is true.

  @note Block Matrices in each diagonal initialized to zero, including A₀, B₀,
  A₁, Eₙ₋₂, Dₙ₋₁, Eₙ₋₁. */
  PentaDiagonalMatrix(int num_blocks, int block_size, bool is_symmetric = true);

  /* Constructs a pentadiagonal matrix from the given diagonals as described in
  this class's main documentation. The size n of the matrix is given by the size
  of the diagonal vectors, all required to be of the same size.

  @pre All vectors A, B, C, D and E must have the same size.
  @pre All blocks must be square of the same size k×k. This invariant is
  verified only in Debug builds.

  @note Notice that we use pass-by-copy semantics. This is to allow move
  semantics (with std::move) to avoid unnecessary heap allocation and copies
  when making a new matrix object (highly recommended if local copies are not
  needed). */
  PentaDiagonalMatrix(std::vector<MatrixX<T>> A, std::vector<MatrixX<T>> B,
                      std::vector<MatrixX<T>> C, std::vector<MatrixX<T>> D,
                      std::vector<MatrixX<T>> E);

  /* Convenience constructor for a symmetric penta-diagonal matrix. That is,
   Eᵢ = Aᵢ₊₂ᵀ and Dᵢ = Bᵢ₊₁ᵀ. Only the lower triangular part of Cᵢ is used,
   whether Cᵢ is symmetric or not.

   @pre All vectors A, B, C, D and E must have the same size.
   @pre All blocks must be square of the same size k×k. This invariant is
   verified only in Debug builds.

   @note Notice that we use pass-by-copy semantics. This is to allow move
   semantics (with std::move) to avoid unnecessary heap allocation and copies
   when making a new matrix object (highly recommended if local copies are not
   needed). */
  PentaDiagonalMatrix(std::vector<MatrixX<T>> A, std::vector<MatrixX<T>> B,
                      std::vector<MatrixX<T>> C);

  /* Copy the lower triangular part of this matrix to the upper triangular part,
   i.e., Eᵢ = Aᵢ₊₂ᵀ and Dᵢ = Bᵢ₊₁ᵀ, to make this a symmetric matrix.

   @pre All vectors A, B, C, D and E must have the same size.
   @pre All blocks must be square of the same size k×k. This invariant is
   verified only in Debug builds. */
  void MakeSymmetric();

  static PentaDiagonalMatrix<T> MakeIdentity(int num_blocks, int block_size);

  static PentaDiagonalMatrix<T> MakeSymmetricFromLowerDense(const MatrixX<T>& M,
                                                            int num_blocks,
                                                            int block_size);

  MatrixX<T> MakeDense() const;

  // The size k of each of the blocks in the diagonals. All blocks have the same
  // size k x k.
  int block_size() const { return A_.size() == 0 ? 0 : A_[0].rows(); }

  // Returns the total number of rows.
  int rows() const { return block_rows() * block_size(); }

  // Returns the total number of columns.
  int cols() const { return rows(); }

  // Returns the number of blocks in each row.
  int block_rows() const { return C_.size(); }

  // Returns the number of blocks in each column.
  int block_cols() const { return block_rows(); }

  // Returns a reference to the second lower diagonal.
  const std::vector<MatrixX<T>>& A() const { return A_; }

  // Mutable version of A().
  // @warning makes the matrix non-symmetric
  std::vector<MatrixX<T>>& mutable_A() {
    is_symmetric_ = false;
    return A_;
  }

  // Returns a reference to the first lower diagonal.
  const std::vector<MatrixX<T>>& B() const { return B_; }

  // Mutable version of B().
  // @warning makes the matrix non-symmetric
  std::vector<MatrixX<T>>& mutable_B() {
    is_symmetric_ = false;
    return B_;
  }

  // Returns a reference to the main diagonal.
  const std::vector<MatrixX<T>>& C() const { return C_; }

  // Mutable version of C().
  // @warning makes the matrix non-symmetric
  std::vector<MatrixX<T>>& mutable_C() {
    is_symmetric_ = false;
    return C_;
  }

  // Returns a reference to the first upper diagonal.
  const std::vector<MatrixX<T>>& D() const { return D_; }

  // Mutable version of D().
  // @pre matrix is not symmetric.
  std::vector<MatrixX<T>>& mutable_D() {
    DRAKE_THROW_UNLESS(!is_symmetric());
    return D_;
  }

  // Returns a reference to the second upper diagonal.
  const std::vector<MatrixX<T>>& E() const { return E_; }

  // Mutable version of E().
  // @pre matrix is not symmetric.
  std::vector<MatrixX<T>>& mutable_E() {
    DRAKE_THROW_UNLESS(!is_symmetric());
    return E_;
  }

  bool is_symmetric() const { return is_symmetric_; }

 private:
  static bool VerifyAllBlocksOfSameSize(const std::vector<MatrixX<T>>& X,
                                        int size);
  bool VerifySizes() const;

  std::vector<MatrixX<T>> A_;
  std::vector<MatrixX<T>> B_;
  std::vector<MatrixX<T>> C_;
  std::vector<MatrixX<T>> D_;
  std::vector<MatrixX<T>> E_;
  bool is_symmetric_{false};
};

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
