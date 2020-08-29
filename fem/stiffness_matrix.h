#pragma once

#include <Eigen/SparseCore>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace fem {
template <typename T>
class StiffnessMatrix;
}
}  // namespace drake

namespace Eigen {
namespace internal {
/* Minimum required trait for a custom sparse matrix to be used in a Eigen
 * sparse iterative solver. */
template <typename T>
struct traits<drake::fem::StiffnessMatrix<T>> {
  typedef T Scalar;
  typedef typename SparseMatrix<T>::StorageIndex StorageIndex;
  typedef Sparse StorageKind;
  typedef MatrixXpr XprKind;
  enum {
    RowsAtCompileTime = Dynamic,
    ColsAtCompileTime = Dynamic,
    MaxRowsAtCompileTime = Dynamic,
    MaxColsAtCompileTime = Dynamic,
    Flags = 0
  };
};
}  // namespace internal
}  // namespace Eigen

namespace drake {
namespace fem {
/** A Sparse/Matrix-free representation of the stiffness matrix for the FEM
 * solver that consists of mass, elastic stiffness and damping terms. It
 * supplies a `Multiply` method that facilitates iterative linear solvers.
 */
template <typename T>
class StiffnessMatrix : public Eigen::EigenBase<StiffnessMatrix<T>> {
 public:
  // Required typedefs, constants, and methods by Eigen:
  typedef T Scalar;
  typedef T RealScalar;
  typedef int StorageIndex;
  typedef typename Eigen::SparseMatrix<T>::InnerIterator InnerIterator;
  enum {
    ColsAtCompileTime = Eigen::Dynamic,
    MaxColsAtCompileTime = Eigen::Dynamic,
    IsRowMajor = false
  };

  StorageIndex rows() const {
    DRAKE_THROW_UNLESS(!matrix_free_);
    return matrix_.rows();
  }

  StorageIndex cols() const {
    DRAKE_THROW_UNLESS(!matrix_free_);
    return matrix_.cols();
  }

  // TODO(xuchenhan-tri): Replace when objective is ready.
  /*
  StorageIndex rows() const { return objective_.get_num_dofs(); }
  StorageIndex cols() const { return objective_.get_num_dofs(); }
  */

  /** Eigen iterative solver requires a specific operator* that returns a
   * Eigen::Product type. */
  template <typename Rhs>
  Eigen::Product<StiffnessMatrix<T>, Rhs, Eigen::AliasFreeProduct> operator*(
      const Eigen::MatrixBase<Rhs>& x) const {
    return Eigen::Product<StiffnessMatrix<T>, Rhs, Eigen::AliasFreeProduct>(
        *this, x.derived());
  }

  // Custom API:
  /* TODO(xuchenhan-tri): The constructor should also take in an objective
   reference that provides the multiplication and builds the matrix when
   necessary. */
  explicit StiffnessMatrix(
      /* const BackwardEulerObjective<T>& objective, */ bool matrix_free =
          false)
      /* : objective_(objective_), */
      : matrix_free_(matrix_free) {
    if (!matrix_free_) Reinitialize();
  }

  VectorX<T> Multiply(const Eigen::Ref<const VectorX<T>>& x) const;

  // TODO(xuchenhan-tri): use the objective to perform matrix-free multiply.
  /*
    void Multiply(const Eigen::Ref<const VectorX<T>>& x,
                  EigenPtr<Matrix3X<T>> b) const;
  */

  void set_matrix_free(bool matrix_free) {
    matrix_free_ = matrix_free;
    Reinitialize();
  }

  bool is_matrix_free() const { return matrix_free_; }

  /**
   Resize the matrix according to information provided by the objective and
   allocate memory for the matrix. This is expensive but only needs to be called
   when the number of degrees of freedom changes, or when the sparsity pattern
   changes. Currently, the number of degrees of freedom and the sparsity pattern
   will remain unchanged after initialization . In the future, that might not be
   true (e.g. when we support fracture or adding objects dynamically).
  */
  void Reinitialize();

  // TODO(xuchenhan-tri): get the underlying objective.
  /*
  const BackwardEulerObjective<T>& get_objective() const { return objective_; }
  */

  /** Get the matrix representation of the stiffness matrix.
    @throws std::runtime_error if matrix_free_ is true.
   */
  const Eigen::SparseMatrix<T>& get_matrix() const {
    DRAKE_THROW_UNLESS(!matrix_free_);
    return matrix_;
  }

  /**  Fill the matrix using Jacobian from the objective. Needs to be called
   every time step when the physics state changes. It's a no-op when
   matrix_free_ is true.
   */
  void BuildMatrix() {
    if (!matrix_free_) {
      // TODO(xuchenhan-tri): Enable when objective is ready.
      /* objective_.BuildJacobian(&matrix_); */
    }
  }

 private:
    friend class StiffnessMatrixTester;
  // TODO(xuchenhan-tri): Enable when objective is ready.
  // const BackwardEulerObjective<T>& objective_;
  bool matrix_free_;
  Eigen::SparseMatrix<T> matrix_;
};
}  // namespace fem
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
        class ::drake::fem::StiffnessMatrix)

/* Implementation of StiffnessMatrix * Eigen::DenseVector though a
 specialization of internal::generic_product_impl:
*/
namespace Eigen {
namespace internal {
template <typename Rhs, typename T>
struct generic_product_impl<drake::fem::StiffnessMatrix<T>, Rhs, SparseShape,
                            DenseShape,
                            GemvProduct>  // GEMV stands for matrix-vector
    : generic_product_impl_base<
          drake::fem::StiffnessMatrix<T>, Rhs,
          generic_product_impl<drake::fem::StiffnessMatrix<T>, Rhs>> {
  typedef typename Product<drake::fem::StiffnessMatrix<T>, Rhs>::Scalar Scalar;

  // This method should implement "dst += alpha * lhs * rhs" inplace.
  template <typename Dest>
  static void scaleAndAddTo(Dest& dst, //NOLINT
                            const drake::fem::StiffnessMatrix<T>& lhs,
                            const Rhs& rhs, const Scalar& alpha) {
    dst.noalias() += alpha * lhs.Multiply(rhs);
  }
};

}  // namespace internal
}  // namespace Eigen
