#pragma once

#include <exception>
#include <string>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {
namespace solvers {

// TODO(amcastro-tri): Consider allowing Eigen:Ref, EigenPtr. Thus far not
// needed.
#define STATIC_ASSERT_DENSE_OR_SPARSE_VECTOR_TYPE(V)                     \
  static_assert(                                                         \
      std::is_same<V, VectorX<T>>{} ||                                   \
          std::is_same<V, Eigen::SparseVector<T>>{},                     \
      "Only drake::VectorX<T> and Eigen::SparseVector<T> are supported " \
      "argument types.");

/// This abstract class provides a generic interface for linear operators
/// A ∈ ℝⁿˣᵐ defined by their application from ℝᵐ into ℝⁿ, y = A⋅x.
/// Derived classes must provide an implementation for this application
/// with specifics that exploit the operator's structure, e.g. sparsity, block
/// diagonal, etc.
template <typename T>
class LinearOperator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearOperator)

  /// Creates an operator with a given `name`.
  explicit LinearOperator(const std::string& name) : name_(name) {}

  virtual ~LinearOperator() = default;

  const std::string& name() const { return name_; }
  virtual int rows() const = 0;
  virtual int cols() const = 0;

  /// Performs y = A⋅x for `this` operator A.
  /// Derived classes must provide an implementation of the virtual interface
  /// DoMultiply().
  template <class VectorType>
  void Multiply(const VectorType& x, VectorType* y) const {
    STATIC_ASSERT_DENSE_OR_SPARSE_VECTOR_TYPE(VectorType);
    DRAKE_DEMAND(y != nullptr);
    DRAKE_DEMAND(x.size() == cols());
    DRAKE_DEMAND(y->size() == rows());
    DoMultiply(x, y);
  }

  /// For `this` operator A, performs y = Aᵀ⋅x.
  /// The default implementation throws a std::runtime_error exception.
  /// Derived classes can provide an implementation through the virtual
  /// interface DoMultiplyByTranspose().
  template <class VectorType>
  void MultiplyByTranspose(const VectorType& x, VectorType* y) const {
    STATIC_ASSERT_DENSE_OR_SPARSE_VECTOR_TYPE(VectorType);
    DRAKE_DEMAND(y != nullptr);
    DRAKE_DEMAND(x.size() == rows());
    DRAKE_DEMAND(y->size() == cols());
    DoMultiplyByTranspose(x, y);
  }

  /// Assembles the explicit matrix form for `this` operator into matrix A.
  /// Some solvers might require this operation in order to use direct methods.
  /// Particularly useful for debugging of sessions.
  /// The default implementation throws a std::runtime_error exception.
  virtual void AssembleMatrix(Eigen::SparseMatrix<T>* A) const;

  // TODO(amcastro-tri): expand operations as needed, e.g.:
  // - MultiplyAndAdd(): z = y + A * x
  // - AXPY(): Y = Y + a * X
  // - Norm(): Matrix norm for a particular norm type.
  // - GetDiagonal(): e.g. to be used in iterative solvers with preconditioning.

 protected:
  /// Performs y = A⋅x for `this` operator A.
  /// Derived classes must provide an implementation.
  /// It's NVI already performed checks for valid arguments.
  virtual void DoMultiply(const VectorX<T>& x, VectorX<T>* y) const = 0;

  /// Alternate signature to operate on sparse vectors.
  /// Derived classes must provide an implementation.
  /// It's NVI already performed checks for valid arguments.
  virtual void DoMultiply(const Eigen::SparseVector<T>& x,
                          Eigen::SparseVector<T>* y) const = 0;

  /// For `this` operator A, performs y = Aᵀ⋅x.
  /// The default implementation throws a std::runtime_error exception.
  /// It's NVI already performed checks for valid arguments.
  virtual void DoMultiplyByTranspose(const VectorX<T>& x, VectorX<T>* y) const;

  /// Alternate signature to operate on sparse vectors.
  /// It's NVI already performed checks for valid arguments.
  virtual void DoMultiplyByTranspose(const Eigen::SparseVector<T>& x,
                                     Eigen::SparseVector<T>* y) const;

 private:
  std::string name_;

  // Helper to throw a specific exception when a given override was not
  // implemented.
  void ThrowIfNotImplemented(const char* source_method) const {
    throw std::logic_error(std::string(source_method) + "()': Instance '" +
                           name_ + "' of type '" + NiceTypeName::Get(*this) +
                           "' must provide an implementation.");
  }
};

// Remove macro symbol to avoid pollution into files including this header.
#undef STATIC_ASSERT_DENSE_OR_SPARSE_VECTOR_TYPE

}  // namespace solvers
}  // namespace multibody
}  // namespace drake
