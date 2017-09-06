#pragma once

/// @file
/// This file contains abbreviated definitions for certain specializations of
/// Eigen::Matrix that are commonly used in Drake.
/// These convenient definitions are templated on the scalar type of the Eigen
/// object. While Drake uses `<T>` for scalar types across the entire code base
/// we decided in this file to use `<Scalar>` to be more consistent with the
/// usage of `<Scalar>` in Eigen's code base.
/// @see also eigen_autodiff_types.h

#include <memory>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/drake_copyable.h"

namespace drake {

/// A column vector of size 1 (that is, a scalar), templated on scalar type.
template <typename Scalar>
using Vector1 = Eigen::Matrix<Scalar, 1, 1>;

/// A column vector of size 1 of doubles.
using Vector1d = Eigen::Matrix<double, 1, 1>;

/// A column vector of size 2, templated on scalar type.
template <typename Scalar>
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;

/// A column vector of size 3, templated on scalar type.
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

/// A column vector of size 4, templated on scalar type.
template <typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

/// A column vector of size 6.
template <typename Scalar>
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

/// A column vector of any size, templated on scalar type.
template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

/// A vector of dynamic size templated on scalar type, up to a maximum of 6
/// elements.
template <typename Scalar>
using VectorUpTo6 = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, 6, 1>;

/// A matrix of 2 rows and 2 columns, templated on scalar type.
template <typename Scalar>
using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;

/// A matrix of 3 rows and 3 columns, templated on scalar type.
template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

/// A matrix of 4 rows and 4 columns, templated on scalar type.
template <typename Scalar>
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

/// A matrix of 6 rows and 6 columns, templated on scalar type.
template <typename Scalar>
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

/// A matrix of 2 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix2X = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;

/// A matrix of 3 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

/// A matrix of 4 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix4X = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>;

/// A matrix of 6 rows, dynamic columns, templated on scalar type.
template <typename Scalar>
using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

/// A matrix of dynamic size, templated on scalar type.
template <typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

/// A matrix of dynamic size templated on scalar type, up to a maximum of 6 rows
/// and 6 columns. Rectangular matrices, with different number of rows and
/// columns, are allowed.
template <typename Scalar>
using MatrixUpTo6 =
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6>;

/// A quaternion templated on scalar type.
template <typename Scalar>
using Quaternion = Eigen::Quaternion<Scalar>;

/// An AngleAxis templated on scalar type.
template <typename Scalar>
using AngleAxis = Eigen::AngleAxis<Scalar>;

/// An Isometry templated on scalar type.
template <typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

/// A translation in 3D templated on scalar type.
template <typename Scalar>
using Translation3 = Eigen::Translation<Scalar, 3>;

/// A column vector of dynamic size, up to a maximum of 73 elements.
using VectorUpTo73d = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73, 1>;

/// A column vector consisting of one twist.
template <typename Scalar>
using TwistVector = Eigen::Matrix<Scalar, kTwistSize, 1>;

/// A matrix with one twist per column, and dynamically many columns.
template <typename Scalar>
using TwistMatrix = Eigen::Matrix<Scalar, kTwistSize, Eigen::Dynamic>;

/// A six-by-six matrix.
template <typename Scalar>
using SquareTwistMatrix = Eigen::Matrix<Scalar, kTwistSize, kTwistSize>;

/// A column vector consisting of one wrench (spatial force) = `[r X f; f]`,
/// where f is a force (translational force) applied at a point `P` and `r` is
/// the position vector from a point `O` (called the "moment center") to point
/// `P`.
template <typename Scalar>
using WrenchVector = Eigen::Matrix<Scalar, 6, 1>;

/// A column vector consisting of a concatenated rotational and translational
/// force.  The wrench is a special case of a SpatialForce.  For a general
/// SpatialForce the rotational force can be a pure torque or the accumulation
/// of moments and need not necessarily be a function of the force term.
template <typename Scalar>
using SpatialForce = Eigen::Matrix<Scalar, 6, 1>;

/// EigenSizeMinPreferDynamic<a, b>::value gives the min between compile-time
/// sizes @p a and @p b. 0 has absolute priority, followed by 1, followed by
/// Dynamic, followed by other finite values.
///
/// Note that this is a type-trait version of EIGEN_SIZE_MIN_PREFER_DYNAMIC
/// macro in "Eigen/Core/util/Macros.h".
template <int a, int b>
struct EigenSizeMinPreferDynamic {
  // clang-format off
  static constexpr int value = (a == 0 || b == 0) ? 0 :
                               (a == 1 || b == 1) ? 1 :
     (a == Eigen::Dynamic || b == Eigen::Dynamic) ? Eigen::Dynamic :
                                           a <= b ? a : b;
  // clang-format on
};

/// EigenSizeMinPreferFixed is a variant of EigenSizeMinPreferDynamic. The
/// difference is that finite values now have priority over Dynamic, so that
/// EigenSizeMinPreferFixed<3, Dynamic>::value gives 3.
///
/// Note that this is a type-trait version of EIGEN_SIZE_MIN_PREFER_FIXED macro
/// in "Eigen/Core/util/Macros.h".
template <int a, int b>
struct EigenSizeMinPreferFixed {
  // clang-format off
  static constexpr int value = (a == 0 || b == 0) ? 0 :
                               (a == 1 || b == 1) ? 1 :
     (a == Eigen::Dynamic && b == Eigen::Dynamic) ? Eigen::Dynamic :
                            (a == Eigen::Dynamic) ? b :
                            (b == Eigen::Dynamic) ? a :
                                           a <= b ? a : b;
  // clang-format on
};

/// MultiplyEigenSizes<a, b> gives a * b if both of a and b are fixed
/// sizes. Otherwise it gives Eigen::Dynamic.
template <int a, int b>
struct MultiplyEigenSizes {
  static constexpr int value =
      (a == Eigen::Dynamic || b == Eigen::Dynamic) ? Eigen::Dynamic : a * b;
};

/*
 * Determines if a type is derived from EigenBase<> (e.g. ArrayBase<>,
 * MatrixBase<>).
 */
template <typename Derived>
struct is_eigen_type : std::is_base_of<Eigen::EigenBase<Derived>, Derived> {};

/*
 * Determines if an EigenBase<> has a specific scalar type.
 */
template <typename Derived, typename Scalar>
struct is_eigen_scalar_same
    : std::integral_constant<
          bool, is_eigen_type<Derived>::value &&
                    std::is_same<typename Derived::Scalar, Scalar>::value> {};

/*
 * Determines if an EigenBase<> type is a compile-time (column) vector.
 * This will not check for run-time size.
 */
template <typename Derived>
struct is_eigen_vector
    : std::integral_constant<bool, is_eigen_type<Derived>::value &&
                                       Derived::ColsAtCompileTime == 1> {};

/*
 * Determines if an EigenBase<> type is a compile-time (column) vector of a
 * scalar type. This will not check for run-time size.
 */
template <typename Derived, typename Scalar>
struct is_eigen_vector_of
    : std::integral_constant<
          bool, is_eigen_scalar_same<Derived, Scalar>::value &&
                    is_eigen_vector<Derived>::value> {};

/*
 * Determines if a EigenBase<> type is a compile-time non-column-vector matrix
 * of a scalar type. This will not check for run-time size.
 * @note For an EigenBase<> of the correct Scalar type, this logic is
 * exclusive to is_eigen_vector_of<> such that distinct specializations are not
 * ambiguous.
 */
// TODO(eric.cousineau): A 1x1 matrix will be disqualified in this case, and
// this logic will qualify it as a vector. Address the downstream logic if this
// becomes an issue.
template <typename Derived, typename Scalar>
struct is_eigen_nonvector_of
    : std::integral_constant<
          bool, is_eigen_scalar_same<Derived, Scalar>::value &&
                    !is_eigen_vector<Derived>::value> {};

// TODO(eric.cousineau): Add alias is_eigen_matrix_of = is_eigen_scalar_same if
// appropriate.

/// This wrapper class provides a way to write non-template functions taking raw
/// pointers to Eigen objects as parameters while limiting the number of copies,
/// similar to `Eigen::Ref`. Internally, it keeps an instance of `Eigen::Ref<T>`
/// and provides access to it via `operator*` and `operator->`.
///
/// The motivation of this class is to follow <a
/// href="https://google.github.io/styleguide/cppguide.html#Reference_Arguments">GSG's
/// "output arguments should be pointers" rule</a> while taking advantage of
/// using `Eigen::Ref`. Here is an example.
///
/// @code
/// // This function is taking an Eigen::Ref of a matrix and modifies it in
/// // the body. This violates GSG's rule on output parameters.
/// void foo(Eigen::Ref<Eigen::MatrixXd> M) {
///    M(0, 0) = 0;
/// }
/// // At Call-site, we have:
/// foo(M);
/// foo(M.block(0, 0, 2, 2));
///
/// // We can rewrite the above function into the following using EigenPtr.
/// void foo(EigenPtr<Eigen::MatrixXd> M) {
///    (*M)(0, 0) = 0;
/// }
/// // Note that, call sites should be changed to:
/// foo(&M);

/// // We need tmp to avoid taking the address of a temporary object such as the
/// // return value of .block().
/// auto tmp = M.block(0, 0, 2, 2);
/// foo(&tmp);
/// @endcode
///
/// @note This class provides a way to avoid the `const_cast` hack introduced in
/// <a
/// href="https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing">Eigen's
/// documentation</a>.
template <typename PlainObjectType>
class EigenPtr {
 public:
  typedef Eigen::Ref<PlainObjectType> RefType;

  EigenPtr() : EigenPtr(nullptr) {}

  /// Overload for `nullptr`.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  EigenPtr(std::nullptr_t) {}

  /// Constructs with a reference to the given matrix type.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  EigenPtr(const EigenPtr& other) { assign(other); }

  /// Constructs with a reference to another matrix type.
  /// May be `nullptr`.
  template <typename PlainObjectTypeIn>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  EigenPtr(PlainObjectTypeIn* m) {
    if (m) {
      m_.reset(new RefType(*m));
    }
  }

  /// Constructs from another EigenPtr.
  template <typename PlainObjectTypeIn>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  EigenPtr(const EigenPtr<PlainObjectTypeIn>& other) {
    // Cannot directly construct `m_` from `other.m_`.
    assign(other);
  }

  EigenPtr& operator=(const EigenPtr& other) {
    // We must explicitly override this version of operator=.
    // The template below will not take precedence over this one.
    return assign(other);
  }

  template <typename PlainObjectTypeIn>
  EigenPtr& operator=(const EigenPtr<PlainObjectTypeIn>& other) {
    return assign(other);
  }

  /// @throws std::runtime_error if this is a null dereference.
  RefType& operator*() const { return *get_reference(); }

  /// @throws std::runtime_error if this is a null dereference.
  RefType* operator->() const { return get_reference(); }

  /// Returns whether or not this contains a valid reference.
  operator bool() const { return is_valid(); }

  bool operator==(std::nullptr_t) const { return !is_valid(); }

  bool operator!=(std::nullptr_t) const { return is_valid(); }

 private:
  // Use unique_ptr<> so that we may "reconstruct" the reference, making this
  // a pointer-like type.
  // TODO(eric.cousineau): Consider using a stack-based implementation if
  // performance is a concern, possibly with a mutable member.
  std::unique_ptr<RefType> m_;

  // Consolidate assignment here, so that both the copy constructor and the
  // construction from another type may be used.
  template <typename PlainObjectTypeIn>
  EigenPtr& assign(const EigenPtr<PlainObjectTypeIn>& other) {
    if (other) {
      m_.reset(new RefType(*other));
    } else {
      m_.reset();
    }
    return *this;
  }

  // Consolidate getting a reference here.
  RefType* get_reference() const {
    if (!m_) throw std::runtime_error("EigenPtr: nullptr dereference");
    return m_.get();
  }

  bool is_valid() const {
    return static_cast<bool>(m_);
  }
};

}  // namespace drake
