#pragma once

#include <memory>

#include "drake/systems/framework/vector_base.h"
#include "drake/system1/vector.h"

namespace drake {
namespace automotive {

/// Models the drake::Vector concept based on some existing VectorBase<T>,
/// named BaseVector.
template <typename BaseVector, typename ScalarType>
class System1Vector : public BaseVector {
 public:
  System1Vector() {}
  System1Vector(const System1Vector<BaseVector, ScalarType>& other) {
    this->set_value(other.get_value());
  }
  System1Vector& operator=(const System1Vector<BaseVector, ScalarType>& other) {
    this->set_value(other.get_value());
    return *this;
  }

  /// @name Implement the drake::Vector concept.
  //@{

  // Even though in practice we have a fixed size, we must declare ourselves
  // dynamically sized for compatibility with the System2 APIs.  The size()
  // method from the drake::Vector concept is provided by BaseVector.
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  // Implicit Eigen::Matrix conversion.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit) per Drake::Vector.
  System1Vector(const Eigen::MatrixBase<Derived>& value) {
    this->set_value(value);
  }

  // Eigen::Matrix assignment.
  template <typename Derived>
  System1Vector& operator=(const Eigen::MatrixBase<Derived>& value) {
    this->set_value(value);
    return *this;
  }

  // Magic conversion specialization back to Eigen.
  friend EigenType toEigen(const System1Vector<BaseVector, ScalarType>& vec) {
    return vec.get_value();
  }

  //@}
};

}  // namespace automotive
}  // namespace drake
