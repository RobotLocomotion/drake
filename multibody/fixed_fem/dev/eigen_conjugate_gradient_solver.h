#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/linear_operator.h"
#include "drake/multibody/fixed_fem/dev/linear_system_solver.h"
namespace drake {
namespace multibody {
namespace fem {
namespace internal {
/* In order to implement the linear solver for contact_solvers::LinearOperator,
 we implement the matrix-free concept in
 https://eigen.tuxfamily.org/dox/group__MatrixfreeSolverExample.html through the
 EigenMatrixProxy class. It wraps around the linear operator and turns it into
 an Eigen object. It provides the methods rows(), cols(), and a bespoke
 multiplication operator. */
template <typename T>
class EigenMatrixProxy : public Eigen::EigenBase<EigenMatrixProxy<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EigenMatrixProxy);
  /* Required typedefs, constants, and methods for EigenMatrixProxy to be used
    in Eigen::ConjugateGradient. */
  using Traits = Eigen::internal::traits<EigenMatrixProxy<T>>;
  using Scalar = typename Traits::Scalar;
  using RealScalar = typename Traits::Scalar;
  using StorageIndex = typename Traits::StorageIndex;
  enum {
    ColsAtCompileTime = Traits::ColsAtCompileTime,
    MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
    IsRowMajor = false
  };

  StorageIndex rows() const {
    DRAKE_THROW_UNLESS(linear_operator_ != nullptr);
    return linear_operator_->rows();
  }

  StorageIndex cols() const {
    DRAKE_THROW_UNLESS(linear_operator_ != nullptr);
    return linear_operator_->cols();
  }

  /* Implements multiplication for EigenMatrixProxy by returning an
   Eigen::Product.
   @tparam Rhs    The type of the Eigen vector multiplying `this` matrix on the
   right. */
  template <typename Rhs>
  Eigen::Product<EigenMatrixProxy<T>, Rhs, Eigen::AliasFreeProduct> operator*(
      const Eigen::MatrixBase<Rhs>& x) const {
    DRAKE_ASSERT(linear_operator_ != nullptr);
    return Eigen::Product<EigenMatrixProxy<T>, Rhs, Eigen::AliasFreeProduct>(
        *this, x.derived());
  }

  /* Custom APIs */
  /* Constructs an EigenMatrixProxy wrapping around the input "linear_operator".
   This class keeps a reference to the input `linear_operator` and therefore it
   is required that `linear_operator` outlives this object. */
  EigenMatrixProxy(
      const contact_solvers::internal::LinearOperator<T>* linear_operator)
      : linear_operator_(linear_operator) {}

  /* Resize the scratch vector to match the size of the linear operator. */
  void Resize() { scratch_vector_.resize(linear_operator_->rows()); }

  ~EigenMatrixProxy() = default;

  /* Scales the product of `this` EigenMatrixProxy and `rhs` by `scale` and adds
   the result to `dest`. */
  void ScaleProductAndAdd(const T& scale,
                          const Eigen::Ref<const VectorX<T>>& rhs,
                          EigenPtr<VectorX<T>> dest) const {
    DRAKE_ASSERT(linear_operator_ != nullptr);
    linear_operator_->Multiply(rhs, &scratch_vector_);
    *dest += scale * scratch_vector_;
  }

 private:
  const contact_solvers::internal::LinearOperator<T>* linear_operator_;
  /* A scratch space to store the result of A*x to avoid repeated allocation. */
  mutable VectorX<T> scratch_vector_;
};
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

namespace Eigen {
namespace internal {
/* Minimal required traits for a custom linear operator to be used in a Eigen
  sparse iterative solver. */
template <typename T>
struct traits<drake::multibody::fem::internal::EigenMatrixProxy<T>> {
  using Scalar = T;
  using StorageIndex = int;
  /* It doesn't matter whether StorageKind is Sparse or Dense as long as it
   matches the "Shape" of the matrix in generic_product_impl below so that the
   correct template specialization takes place. */
  using StorageKind = Sparse;
  enum {
    RowsAtCompileTime = Dynamic,
    ColsAtCompileTime = Dynamic,
    MaxRowsAtCompileTime = Dynamic,
    MaxColsAtCompileTime = Dynamic,
    /* Compile time properties of the matrix. See
       https://eigen.tuxfamily.org/dox-devel/group__flags.html. */
    Flags = 0x0
  };
};

/* Implements multiplications between EigenMatrixProxy and Eigen dense vectors
 by specializing the template Eigen::generic_product_impl. */
template <typename Rhs, typename T>
struct generic_product_impl<
    drake::multibody::fem::internal::EigenMatrixProxy<T>, Rhs,
    SparseShape, DenseShape,
    GemvProduct>  // GEMV stands for matrix-vector
    : generic_product_impl_base<
          drake::multibody::fem::internal::EigenMatrixProxy<T>, Rhs,
          generic_product_impl<
              drake::multibody::fem::internal::EigenMatrixProxy<T>,
              Rhs>> {
  template <typename Dest>
  static void scaleAndAddTo(
      // NOLINTNEXTLINE(runtime/references) Eigen internal signature.
      Dest& dst,
      const drake::multibody::fem::internal::EigenMatrixProxy<T>& lhs,
      const Rhs& rhs, const T& alpha) {
    /* This method implements "dst += alpha * lhs * rhs" inplace. */
    lhs.ScaleProductAndAdd(alpha, rhs, &dst);
  }
};
}  // namespace internal
}  // namespace Eigen

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
/* Implements LinearSystemSolver with an underlying Eigen::ConjugateGradient
 solver. The linear operator A in the system Ax = b must be symmetric positive
 definite (SPD). This solver, however, will not verify that A is SPD as the
 verification process is usually expensive. The user of this class therefore has
 to be careful in passing in a SPD operator. Failure to do so may lead to
 incorrect solutions.
 @tparam_nonsymbolic_scalar T */
template <typename T>
class EigenConjugateGradientSolver : public LinearSystemSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EigenConjugateGradientSolver)

  /* Constructs an EigenConjugateGradientSolver and set the tolerance of the
  iterative solver to the given `tol`. Being an iterative solver, the underlying
  Eigen::ConjudateGradient solver will iterate until the relative error
  ||Ax-b||/||b|| is smaller than the set tolerance or when the max number of
  iterations is reached.  */
  EigenConjugateGradientSolver(
      const contact_solvers::internal::LinearOperator<T>* A, double tol = 1e-4)
      : LinearSystemSolver<T>(A), matrix_proxy_(A) {
    set_tolerance(tol);
  }

  ~EigenConjugateGradientSolver() {}

  /* Implements LinearSystemSolver::Solve(). */
  void Solve(const Eigen::Ref<const VectorX<T>>& b,
             EigenPtr<VectorX<T>> x) const final {
    *x = cg_.solve(b);
  }

  /* Returns Eigen::ConjugateGradient::maxIterations(). */
  void max_iterations() const { cg_.maxIterations(); }

  /* Forwards to Eigen::ConjugateGradient::setMaxIterations(). */
  void set_max_iterations(int max_iterations) {
    cg_.setMaxIterations(max_iterations);
  }

  /* Returns Eigen::ConjugateGradient::tolerance(). */
  double tolerance() const { return cg_.tolerance(); }

  /* Forwards to Eigen::ConjugateGradient::setTolerance(). */
  void set_tolerance(const T& tol) final { cg_.setTolerance(tol); }

 private:
  void DoCompute() final {
    matrix_proxy_.Resize();
    cg_.compute(matrix_proxy_);
  }

  EigenMatrixProxy<T> matrix_proxy_;
  /* TODO(xuchenhan-tri): Allow for custom preconditioner if needed in the
   future. */
  Eigen::ConjugateGradient<EigenMatrixProxy<T>, Eigen::Lower | Eigen::Upper,
                           Eigen::IdentityPreconditioner>
      cg_;
};
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
