#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/parallelism.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

#ifndef DRAKE_DOXYGEN_CXX
namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Forward declaration to allow the specialization of the Eigen traits below. */
class EigenBlock3x3SparseSymmetricMatrix;

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

namespace Eigen {
namespace internal {

/* Gives the wrapper class a dense matrix trait so that Eigen chooses the GEMV
 multiplication in the CG solver. */
template <>
struct traits<drake::multibody::contact_solvers::internal::
                  EigenBlock3x3SparseSymmetricMatrix>
    : traits<MatrixX<double>> {};

}  // namespace internal
}  // namespace Eigen
#endif  // DRAKE_DOXYGEN_CXX

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Wrapper class around BlockSparseSymmetricMatrix3d that's compatible
 with Eigen::ConjugateGradient. This wrapper is supposed to be short-lived
 It should be constructed, used in an Eigen solver, and then discarded.

 Example usage:
 {
  // A_blocks is a BlockSparseSymmetricMatrix3d.
  EigenBlock3x3SparseSymmetricMatrix A(&A_blocks, Parallelism(true));
  Eigen::ConjugateGradient<EigenBlock3x3SparseSymmetricMatrix,
                            Eigen::Lower | Eigen::Upper>
      cg;
  cg.compute(eigen_A);
  DRAKE_THROW_UNLESS(cg.info() == Eigen::Success);
  // Solves Ax = b using CG.
  const VectorXd b = VectorXd::LinSpaced(A.cols(), 0.0, 10.0);
  const VectorXd x = cg.solve(b);
 }

 */
class EigenBlock3x3SparseSymmetricMatrix
    : public Eigen::EigenBase<EigenBlock3x3SparseSymmetricMatrix> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EigenBlock3x3SparseSymmetricMatrix);

  using Scalar = double;
  using RealScalar = double;
  using StorageIndex = int;

  enum {
    ColsAtCompileTime = Eigen::Dynamic,
    MaxColsAtCompileTime = Eigen::Dynamic,
    IsRowMajor = false
  };

  /* Constructs a matrix wrapper for BlockSparseSymmetricMatrix3d.
   @pre A is not null and outlives this object. */
  EigenBlock3x3SparseSymmetricMatrix(const BlockSparseSymmetricMatrix3d* A,
                                     Parallelism parallelism = false);

  /* Required by EigenBase */
  Eigen::Index rows() const { return A_->rows(); }
  Eigen::Index cols() const { return A_->cols(); }

  /* Lazy‐Product: builds the expression A * x */
  template <typename Rhs>
  auto operator*(const Eigen::MatrixBase<Rhs>& x) const {
    return Eigen::Product<EigenBlock3x3SparseSymmetricMatrix, Rhs,
                          Eigen::AliasFreeProduct>(*this, x.derived());
  }

  /* Performs y = A*x where A is this matrix.
   @pre x and y have sizes compatible with this matrix. */
  void Multiply(const VectorX<double>& x, EigenPtr<VectorX<double>> y) const;

  /* Eigen::ConjugateGradient required signature for diagonal preconditioner
   that returns the diagonal of the matrix. */
  const VectorX<double>& diagonal() const { return diagonal_; }

  /* Eigen::ConjugateGradient required signature for diagonal preconditioner
   that returns the number of columns of the matrix. */
  Eigen::Index outerSize() const { return cols(); }

  /* Eigen::ConjugateGradient requires an `InnerIterator` struct that returns
   the diagonal of the matrix. The "outer" loop is the loop over columns, and
   the "inner" iterator iterates over the entries in a single column. Since we
   only want the diagonal entries, the inner iterator is only valid for the
   first element (since only the lower triangular data is stored).  */
  struct InnerIterator {
    using Index = Eigen::Index;
    using ValueType = Scalar;

    InnerIterator(const EigenBlock3x3SparseSymmetricMatrix& mat_in, int col_in)
        : mat(&mat_in), col(col_in), done(false) {}

    /* Operator bool(): true for first element, false once incremented. */
    explicit operator bool() const { return !done; }

    /* The pre-increment operator that advances the iterator once moves it to
     the end. */
    InnerIterator& operator++() {
      done = true;
      return *this;
    }

    /* The only entry in this column that we need to loop over for the
     preconditioner is the diagonal entry. */
    Index index() const { return col; }

    /* Returns the diagonal value. */
    ValueType value() const { return mat->diagonal()(col); }

   private:
    const EigenBlock3x3SparseSymmetricMatrix* mat{};
    int col{};
    bool done{};
  };

 private:
  const BlockSparseSymmetricMatrix3d* const A_{};
  VectorX<double> diagonal_;
  /* The connectivity pattern of the blocks in A. More specifically,
   `row_neighbors_[i][k]` gives the block column index of the k-th non-zero
   block in the i-th block row. `row_neighbors_[i]` is sorted for each block row
   i and each entry in `row_neighbors_[i]` is less than or equal to i. In other
   words, only the lower triangular part of the sparsity pattern is specified.
   As a result, we have `row_neighbors_[i].back() = i` for each i because all
   diagonal blocks of A are nonzero. This is useful for the matrix-vector
   multiplication. */
  std::vector<std::vector<int>> row_neighbors_{};
  Parallelism parallelism_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
namespace Eigen {
namespace internal {

/* Implements Eigen's GEMV with the custom Multiply. */
template <typename Rhs>
struct generic_product_impl<drake::multibody::contact_solvers::internal::
                                EigenBlock3x3SparseSymmetricMatrix,
                            Rhs,
                            /* LHS kind = */ DenseShape,
                            /* RHS kind = */ DenseShape,
                            /* product = */ GemvProduct>
    : generic_product_impl_base<
          drake::multibody::contact_solvers::internal::
              EigenBlock3x3SparseSymmetricMatrix,
          Rhs,
          generic_product_impl<drake::multibody::contact_solvers::internal::
                                   EigenBlock3x3SparseSymmetricMatrix,
                               Rhs>> {
  using MatrixType = drake::multibody::contact_solvers::internal::
      EigenBlock3x3SparseSymmetricMatrix;
  using Scalar = double;

  template <typename Dest>
  // NOLINTNEXTLINE(runtime/references): Eigen-dictated signature.
  static void scaleAndAddTo(Dest& dst, const MatrixType& A, const Rhs& x,
                            const double&) {
    A.Multiply(x, &dst);
  }
};

}  // namespace internal
}  // namespace Eigen
#endif  // DRAKE_DOXYGEN_CXX
