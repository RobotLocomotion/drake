#pragma once

#include "drake/common/parallelism.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
class EigenBlock3x3SparseSymmetricMatrix;
}
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

#include <Eigen/Core>
namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
/* Wrapper class around EigenBlock3x3SparseSymmetricMatrix that's compatible
 with Eigen::ConjugateGradient. */
class EigenBlock3x3SparseSymmetricMatrix
    : public Eigen::EigenBase<EigenBlock3x3SparseSymmetricMatrix> {
 public:
  using Scalar = double;
  using RealScalar = double;
  using StorageIndex = int;

  enum {
    ColsAtCompileTime = Eigen::Dynamic,
    MaxColsAtCompileTime = Eigen::Dynamic,
    IsRowMajor = false
  };

  EigenBlock3x3SparseSymmetricMatrix(const Block3x3SparseSymmetricMatrix* A,
                                     Parallelism parallelism)
      : A_(A), parallelism_(parallelism) {}

  /* Required by EigenBase */
  Eigen::Index rows() const { return A_->rows(); }
  Eigen::Index cols() const { return A_->cols(); }

  /* Lazy‐Product: builds the expression A * x */
  template <typename Rhs>
  Eigen::Product<EigenBlock3x3SparseSymmetricMatrix, Rhs,
                 Eigen::AliasFreeProduct>
  operator*(const Eigen::MatrixBase<Rhs>& x) const {
    return Eigen::Product<EigenBlock3x3SparseSymmetricMatrix, Rhs,
                          Eigen::AliasFreeProduct>(*this, x.derived());
  }

  /* Wraps around the Multiply() function. */
  void Multiply(const VectorX<double>& x, VectorX<double>* y) const {
    A_->Multiply(x, y, parallelism_);
  }

  VectorX<double> diagonal() const {
    VectorX<double> result(rows());
    for (int j = 0; j < A_->block_cols(); ++j) {
      const Eigen::Matrix3d& block = A_->diagonal_block(j);
      result.segment<3>(3 * j) = block.diagonal();
    }
    return result;
  }

  // 1) Tell Eigen how many "outers" exist (for column-major sparse: #cols)
  Eigen::Index outerSize() const { return cols(); }

  // 2) A minimal InnerIterator that returns exactly the diagonal entry, then
  // ends.
  struct InnerIterator {
    using Index = Eigen::Index;
    using ValueType = Scalar;

    InnerIterator(const EigenBlock3x3SparseSymmetricMatrix& mat, int col)
        : mat_(&mat), col_(col), done_(false) {}

    // operator bool(): true for first element, false once incremented.
    explicit operator bool() const { return !done_; }

    // ++it moves to "end"
    InnerIterator& operator++() {
      done_ = true;
      return *this;
    }

    // we pretend the only entry in this column is at row=col
    Index index() const { return col_; }

    // return the diagonal value
    ValueType value() const {
      // you already have diagonal(); each call is O(#blocks) but preconditioner
      // is called only once, so it's OK.
      return mat_->diagonal()[col_];
    }

   private:
    const EigenBlock3x3SparseSymmetricMatrix* mat_{};
    int col_{};
    bool done_{};
  };

 private:
  const Block3x3SparseSymmetricMatrix* A_{};
  Parallelism parallelism_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

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
  using Mat = drake::multibody::contact_solvers::internal::
      EigenBlock3x3SparseSymmetricMatrix;
  using Scalar = typename Mat::Scalar;

  template <typename Dest>
  // NOLINTNEXTLINE(runtime/references)
  static void scaleAndAddTo(Dest& dst, Mat const& A, Rhs const& x,
                            const Scalar&) {
    A.Multiply(x, &dst);
  }
};

}  // namespace internal
}  // namespace Eigen
