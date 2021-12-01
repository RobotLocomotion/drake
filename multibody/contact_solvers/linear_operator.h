#pragma once

#include <exception>
#include <string>

#include <Eigen/SparseCore>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
class BlockSparseMatrix;

// This abstract class provides a generic interface for linear operators
// A ∈ ℝⁿˣᵐ defined by their application from ℝᵐ into ℝⁿ, y = A⋅x.
// Derived classes must provide an implementation for this application
// with specifics that exploit the operator's structure, e.g. sparsity, block
// diagonal, etc.
// Since most solvers will only need the multiplication operator, subclasses
// are required to implement this operation in DoMultiply() for both dense and
// sparse multiplies.
// Some operators, typically contact Jacobians for instance, do require
// additional operations such as multiplication by their transpose. This will
// generally be a requirement documented by specific solvers but not enforced
// by this class. Therefore, the implementation of that operation in
// DoMultiplyByTranspose() is optional, with the default implementation
// provided by this class throwing a runtime exception. Similarly for other
// operations.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class LinearOperator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearOperator)

  // Creates an operator with a given `name`.
  explicit LinearOperator(const std::string& name) : name_(name) {}

  virtual ~LinearOperator() = default;

  const std::string& name() const { return name_; }
  virtual int rows() const = 0;
  virtual int cols() const = 0;

  // Performs y = A⋅x for `this` operator A.
  // Derived classes must provide an implementation of the virtual interface
  // DoMultiply().
  // @throws if y is nullptr.
  // @throws if x.size() does not equal this->cols() or if y->size() does not
  // equal this->rows().
  void Multiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                Eigen::SparseVector<T>* y) const {
    DRAKE_DEMAND(y != nullptr);
    DRAKE_DEMAND(x.size() == cols());
    DRAKE_DEMAND(y->size() == rows());
    DoMultiply(x, y);
  }

  // Alternative signature that operates on dense vectors.
  void Multiply(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
    DRAKE_DEMAND(y != nullptr);
    DRAKE_DEMAND(x.size() == cols());
    DRAKE_DEMAND(y->size() == rows());
    DoMultiply(x, y);
  }

  // For `this` operator A, performs y = Aᵀ⋅x.
  // The default implementation throws a std::exception.
  // Derived classes can provide an implementation through the virtual
  // interface DoMultiplyByTranspose().
  // @throws if y is nullptr.
  // @throws if x.size() does not equal this->rows() or if y->size() does not
  // equal this->cols().
  void MultiplyByTranspose(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                           Eigen::SparseVector<T>* y) const {
    DRAKE_DEMAND(y != nullptr);
    DRAKE_DEMAND(x.size() == rows());
    DRAKE_DEMAND(y->size() == cols());
    DoMultiplyByTranspose(x, y);
  }

  // Alternative signature that operates on dense vectors.
  void MultiplyByTranspose(const Eigen::Ref<const VectorX<T>>& x,
                           VectorX<T>* y) const {
    DRAKE_DEMAND(y != nullptr);
    DRAKE_DEMAND(x.size() == rows());
    DRAKE_DEMAND(y->size() == cols());
    DoMultiplyByTranspose(x, y);
  }

  // Assembles this operator into an Eigen::SparseMatrix.
  // @throws if A is nullptr.
  void AssembleMatrix(Eigen::SparseMatrix<T>* A) const {
    DRAKE_DEMAND(A != nullptr);
    DoAssembleMatrix(A);
  }

  // Assembles this operator into a BlockSparseMatrix.
  // @throws if A is nullptr.
  void AssembleMatrix(BlockSparseMatrix<T>* A) const {
    DRAKE_DEMAND(A != nullptr);
    DoAssembleMatrix(A);
  }

  // TODO(amcastro-tri): expand operations as needed, e.g.:
  // - MultiplyAndAdd(): z = y + A * x
  // - AXPY(): Y = Y + a * X
  // - Norm(): Matrix norm for a particular norm type.
  // - GetDiagonal(): e.g. to be used in iterative solvers with preconditioning.

 protected:
  // Performs y = A⋅x for `this` operator A.
  // Its NVI already performed checks for valid arguments.
  virtual void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                          Eigen::SparseVector<T>* y) const = 0;

  // Alternate signature to operate on dense vectors.
  // Its NVI already performed checks for valid arguments.
  virtual void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                          VectorX<T>* y) const = 0;

  // For `this` operator A, performs y = Aᵀ⋅x.
  // The default implementation throws a std::exception.
  // Its NVI already performed checks for valid arguments.
  virtual void DoMultiplyByTranspose(
      const Eigen::Ref<const Eigen::SparseVector<T>>& x,
      Eigen::SparseVector<T>* y) const;

  // Alternate signature to operate on dense vectors.
  // Its NVI already performed checks for valid arguments.
  virtual void DoMultiplyByTranspose(const Eigen::Ref<const VectorX<T>>& x,
                                     VectorX<T>* y) const;

  // TODO(amcastro-tri): For these assembly methods, consider a default
  // implementation based on repeated multiplies by unit vectors.

  // Assembles `this` operator into an Eigen::SparseMatrix.
  // The default implementation throws a std::exception.
  // The NVI already checked for a valid non-null pointer. The implementation
  // must properly re-size the output matrix on assembly.
  virtual void DoAssembleMatrix(Eigen::SparseMatrix<T>* A) const;

  // Assembles `this` operator into a BlockSparseMatrix.
  // The default implementation throws a std::exception.
  // The NVI already checked for a valid non-null pointer. The
  // implementation must properly re-size the output matrix on assembly.
  virtual void DoAssembleMatrix(BlockSparseMatrix<T>* A) const;

 private:
  std::string name_;

  // Helper to throw a specific exception when a given override was not
  // implemented.
  void ThrowIfNotImplemented(const char* source_method) const {
    throw std::runtime_error(std::string(source_method) + "(): Instance '" +
                           name_ + "' of type '" + NiceTypeName::Get(*this) +
                           "' must provide an implementation.");
  }
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::LinearOperator)
