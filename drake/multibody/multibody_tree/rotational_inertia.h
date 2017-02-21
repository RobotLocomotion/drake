#pragma once

#include <algorithm>
#include <memory>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

#include <Eigen/Eigenvalues>

namespace drake {
namespace multibody {

// Implementation details go into this namespace. Users never see this.
namespace internal {
// Helper method to swap (i, j) indexes when needed.
template <int TriangularPart> struct check_and_swap;

// Specialization when using the upper-diagonal elements.
template <> struct check_and_swap<Eigen::Upper> {
  static void swap(int& i, int &j) {
    if (i > j) std::swap(i, j);
  }
};

// Specialization when using the lower-diagonal elements.
template <> struct check_and_swap<Eigen::Lower> {
  static void swap(int& i, int &j) {
    if (i < j) std::swap(i, j);
  }
};
}  // namespace internal

/// This class provides an abstraction for the physical concept of the mass
/// distribution of a body about a particular point. Given a point, the mass
/// distribution of a body is generally described by the first three mass
/// weighted moments about that point. These moments are the mass of the body
/// (or zeroth moment), the center of mass vector (or first order moment) and
/// finally the rotational inertia (or second order moment).
/// We choose to use the term **rotational inertia** as used by [Jain 2010] to
/// distinguish from the more general concept of **inertia** of a body.
/// A rotational inertia can be represented by the six scalar elements of a
/// symmetric 3x3 matrix often referred also as **the inertia matrix** or also
/// as **the inertia tensor**. We can therefore think of a rotational inertia
/// `I` as the matrix: <pre>
///     | Ixx Ixy Ixz |
/// I = | Ixy Iyy Iyz |
///     | Ixz Iyz Izz |
/// </pre>
/// where diagonal elements of this matrix are referred to as the **moments of
/// inertia** while the off-diagonal elements are referred to as the **products
/// of inertia**. These scalar elements are the numerical values of the
/// rotational inertia components measured with respect to the axes of a given
/// frame and therefore this frame needs to be explicitly stated. These scalar
/// elements have no meaning if a reference frame is not specified.
/// For a given point on the rigid body there exists a set of axes, called
/// **principal axes of inertia** in which the inertia tensor is diagonal. The
/// resulting diagonal elements are the **principal moments of inertia** about
/// that point. The corresponding directions are an orthogonal basis for the
/// inertia tensor with those principal moments.
///
/// @note This class does not implement any mechanism to track the frame in
/// which an inertia is expressed or about what point is computed. Methods and
/// operators on this class have no means to determine frame consistency through
/// operations. It is therefore the responsability of users of this class to
/// keep track of frames in which operations are performed.
///
/// In code we use the monogram notation as described
/// in @ref multibody_notation_basics. For a rotational inertia computed about
/// point `Bo` and expressed in frame `E` the monogram notation would
/// read `I_Bo_E`.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class RotationalInertia {
 public:
  enum {
    // This class internally uses a full 3x3 Eigen matrix to store the six
    // elements that are needed to represent it. However, only one triangular
    // portion is used leaving redundant elements set to NaN so that operations
    // using them fail fast, allowing fast bug detection.
    // By default RotationalInertia only works on the lower part of the
    // underlying Eigen matrix.
    // This particular choice is arbitrary. However, it was observed however
    // that Eigen sometimes uses the lower part of a symmetric dense matrix.
    // See Eigen::SelfAdjointEigenSolver. This is used by
    // RotationalInertia::CalcPrincipalMomentsOfInertia().
    TriangularViewInUse = Eigen::Lower,
    // The strictly lower part is set to NaN to quickly detect when used by
    // error.
    TriangularViewNotInUse = Eigen::StrictlyUpper
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationalInertia)

  /// Default RotationalInertia constructor. All entries are set to NaN for a
  /// quick detection of un-initialized values.
  RotationalInertia() {}

  /// Creates a principal rotational inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  explicit RotationalInertia(const T& I) {
    SetZero();
    I_Bo_F_.diagonal().setConstant(I);
  }

  /// Creates a principal axes rotational inertia matrix for which the products
  /// of inertia are zero and the moments of inertia are given by `Ixx`, `Iyy`
  /// and `Izz`.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz) {
    SetZero();
    I_Bo_F_.diagonal() = Vector3<T>(Ixx, Iyy, Izz);
  }

  /// Creates a general rotational inertia matrix with non-zero off-diagonal
  /// elements where the six components of the rotational intertia on a given
  /// frame `E` need to be provided.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz) {
    // The TriangularViewNotInUse is left initialized to NaN.
    auto& Iref = *this;
    // Let the operator(i, j) decide on what portion (upper/lower) to write on.
    Iref(0, 0) = Ixx; Iref(1, 1) = Iyy; Iref(2, 2) = Izz;
    Iref(0, 1) = Ixy; Iref(0, 2) = Ixz; Iref(1, 2) = Iyz;
  }

  /// For consistency with Eigen's API this method returns the number of rows
  /// in the inertia matrix.
  int rows() const { return 3;}

  /// For consistency with Eigen's API this method returns the number of columns
  /// in the inertia matrix.
  int cols() const { return 3;}

  /// Returns a three-dimensional vector containing the diagonal elements of
  /// this rotational inertia.
  /// @retval moments The vector of principal moments `[Ixx Iyy Izz]`.
  Vector3<T> get_moments() const { return I_Bo_F_.diagonal(); }

  /// Returns a three-dimensional vector containing the products of inertia of
  /// this rotational inertia.
  /// @retval products The vector of products of inertia `[Ixy, Ixz, Iyz]`.
  Vector3<T> get_products() const {
    // Let operator(int ,int) decide what portion (upper/lower) to use.
    const auto& Iref = *this;
    return Vector3<T>(Iref(0, 1), Iref(0, 2), Iref(1, 2));
  }

  /// Const access to the `(i, j)` element of this rotational inertia.
  /// This operator performs checks on the pair `(i, j)` to determine the
  /// appropriate mapping to the internal in-memory representation of a
  /// symmetric rotational inertia. Therefore this accessor is not meant for
  /// speed but rather as a convenience method. Users should use supplied
  /// built-in operations for fast computations.
  /// Notice that the mutable counterpart of this accessor is not provided to
  /// prevent the creation of unphysical inertias by setting one element at a
  /// time.
  const T& operator()(int i, int j) const {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_Bo_F_(i, j);
  }

  /// Get a copy to a full Matrix3 representation for this rotational inertia
  /// including both lower and upper triangular parts.
  Matrix3<T> CopyToFullMatrix3() const { return get_symmetric_matrix_view(); }

  /// Compares `this` inertia to @p other rotationl inertia within the specified
  /// @p precision.
  /// The comparison is performed using the fuzzy comparison provided by Eigen's
  /// method isApprox() returning `true` if: <pre>
  ///   get_moments().isApprox(other.get_moments(), precision) &&
  ///   get_products().isApprox(other.get_products(), precision);
  /// </pre>
  /// @returns `true` if `other` is within the specified @p precision. Returns
  /// `false` otherwise.
  bool IsApprox(const RotationalInertia& other,
                double precision = Eigen::NumTraits<T>::epsilon()) {
    return get_moments().isApprox(other.get_moments(), precision) &&
           get_products().isApprox(other.get_products(), precision);
  }

  /// Adds rotational inertia @p `I_Bo_F` to this rotational inertia. This
  /// operation is only valid if both inertias are computed about the same
  /// center `Bo` and expressed in the same frame `F`.
  /// @param[in] I_Bo_F A rotational inertia to be added to this inertia.
  /// @returns A reference to `this` rotational inetia.
  RotationalInertia& operator+=(const RotationalInertia<T>& I_Bo_F) {
    this->get_mutable_symmetric_matrix_view() += I_Bo_F.get_matrix();
    return *this;
  }

  /// Computes the product from the right between this rotational inertia with
  /// vector @p w.
  /// This inertia and vector @p w must both be expressed in the same frame.
  /// @param[in] w Vector to multiply from the right.
  /// @returns The product from the right of `this` inertia with @p w.
  Vector3<T> operator*(const Vector3<T>& w) const {
    return Vector3<T>(get_symmetric_matrix_view() * w);
  }

  /// Sets this inertia to have NaN entries. Typically used to quickly detect
  /// uninitialized values since NaN will trigger a chain of invalid
  /// computations that can then be tracked to the source.
  void SetToNaN() {
    I_Bo_F_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  /// Sets this rotational inertia to have zero entries. This results in a
  /// non-physical inertia that could only mathematically correspond to the mass
  /// distribution of a point. However this method is useful when performing
  /// initializations for a given computation.
  void SetZero() {
    // The part corresponding to RotationalInertia::TriangularViewNotInUse still
    // is left initialized to NaN to quickly detect if this part is
    // mistakenly used.
    I_Bo_F_.template triangularView<TriangularViewInUse>() = Matrix3<T>::Zero();
  }

  /// Constructs a RotationalInertia from an Eigen matrix expression. The matrix
  /// expression @p m must be represent a valid 3x3 matrix with the measures for
  /// a rotational inertia computed about a given point on a given frame. This
  /// method does not check for the physical validity of the resulting
  /// rotational inertia for fast constructions. Users can still verify the
  /// validity of the newly created inertia with IsPhysicallyValid().
  template<typename Derived>
  explicit RotationalInertia(const Eigen::MatrixBase<Derived>& m) :
      I_Bo_F_(m) {}

  /// Assignment operator from a general Eigen expression.
  /// This method allows to assign Eigen expressions to a RotationalInertia.
  /// The matrix
  /// expression @p m must be represent a valid 3x3 matrix with the measures for
  /// a rotational inertia computed about a given point on a given frame. This
  /// method does not check for the physical validity of the resulting
  /// rotational inertia for fast constructions. Users can still verify the
  /// validity of the newly created inertia with IsPhysicallyValid().
  template<typename Derived>
  RotationalInertia& operator=(const Eigen::MatrixBase<Derived>& m) {
    // Static asserts that EigenMatrix is of fixed size 3x3.
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    this->get_mutable_symmetric_matrix_view() = m;
    return *this;
  }

  /// Returns `true` if any of the elements in this rotational inertia is NaN
  /// and `false` otherwise.
  bool IsNaN() const {
    using std::isnan;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        // We use operator()(int, int) here to automatically only check the
        // portion in use according to TriangularViewInUse.
        if (isnan(operator()(i, j))) return true;
      }
    }
    // Another alternative is to use:
    // get_moments().array().isNaN().any() &&
    // get_products().array().isNaN().any()
    return false;
  }

  /// For `this` inertia about a given point `P` and expressed in a frame `E`,
  /// this method computes the principal moments of inertia of `this` rotational
  /// inertia about the same point `P` and expressed on a frame with origin at
  /// `P` and aligned with the principal axes.
  /// The computed principal moments are placed into @p principal_moments sorted
  /// in ascending order.
  /// @returns `true` if succesful and `false` otherwise.
  bool CalcPrincipalMomentsOfInertia(Vector3<T>* principal_moments) const {
    DRAKE_ASSERT(principal_moments != nullptr);
    // Eigen's SelfAdjointEigenSolver only works with the lower diagonal part
    // of the matrix. To avoid future issues in case we decide to use
    // RotationalInertia::TriangularViewInUse = Eigen::Upper, here we use a
    // local copy to a full matrix.
    Eigen::SelfAdjointEigenSolver<Matrix3<T>> solver(
        CopyToFullMatrix3(), Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) return false;
    *principal_moments = solver.eigenvalues();
    return true;
  }

  /// Performs a number of checks to verify that this is a physically valid
  /// rotational inertia.
  /// The chekcs performed are:
  /// - No NaN entries.
  /// - Strictly positive principal moments.
  /// - Principal moments must satisfy the triangle inequality:
  ///   - `Ixx + Iyy >= Izz`
  ///   - `Ixx + Izz >= Iyy`
  ///   - `Iyy + Izz >= Ixx`
  /// @returns `true` for a physically valid rotational inertia passing the
  ///          above checks and `false` otherwise.
  bool IsPhysicallyValid() const {
    if (IsNaN()) return false;

    // Compute principal moments of inertia.
    Vector3<T> d;
    if (!CalcPrincipalMomentsOfInertia(&d)) return false;

    // Principal moments must be strictly positive.
    if ((d.array() <= T(0)).any() ) return false;

    // Checks triangle inequality
    if (!( d[0] + d[1] >= d[2] && d[0] + d[2] >= d[1] && d[1] + d[2] >= d[0]))
      return false;

    return true;  // All tests passed.
  }

  /// Given this rotational inertia `I_Bo_F` about `Bo` and expressed in frame
  /// `F`, this method computes the same inertia re-expressed in another
  /// frame `A`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A reference to `this` rotational inertia about `Bo` but now
  /// re-expressed in frame `A`.
  RotationalInertia& ReExpressInPlace(const Matrix3<T>& R_AF) {
    // Note: using triangularView<TriangularViewInUse>() to only write on the
    // triangular part in use causes serious aliasing problems.
    // There is an interesting discussion on Eigen's forum here:
    // https://forum.kde.org/viewtopic.php?f=74&t=97282
    // That discussion tell us that really here we don't have a significant
    // performance gain for small matrices when writing on one of the triangular
    // parts only. Then gain is in accuracy, by having RotationalInertia to only
    // deal with one triangular portion of the matrix.
    I_Bo_F_ = R_AF *
        I_Bo_F_.template selfadjointView<TriangularViewInUse>() *
        R_AF.transpose();
    return *this;
  }

  /// Given this rotational inertia `I_Bo_F` about `Bo` and expressed in
  /// frame `F`, this method computes the same inertia re-expressed in another
  /// frame `A`.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns I_Bo_A The same rotational inertia bout `Bo` expressed in frame
  /// `A`.
  RotationalInertia ReExpress(const Matrix3<T>& R_AF) const {
    return RotationalInertia(*this).ReExpressInPlace(R_AF);
  }

  /// Multiplication of a RotationalInertia @p I_Bo_F from the left by a
  /// scalar @p s.
  friend RotationalInertia<T> operator*(const T& s,
                                        const RotationalInertia<T>& I_Bo_F) {
    RotationalInertia<T> sxI;
    sxI.get_mutable_symmetric_matrix_view() = s * I_Bo_F.get_matrix();
    return sxI;
  }

 private:
  // Utility method used to swap matrix indexes (i, j) depending on the
  // TriangularViewInUse portion of this inertia. The swap is performed so that
  // we only use the triangular portion corresponding to TriangularViewInUse.
  static void check_and_swap(int* i, int* j) {
    internal::check_and_swap<TriangularViewInUse>::swap(*i , *j);
  }

  // Mutable access to the `(i, j)` element of this rotational inertia.
  // This operator performs checks on the pair `(i, j)` to determine the
  // appropriate mapping to the internal in-memory representation of a
  // symmetric rotational inertia. Therefore this accessor is not meant for
  // speed but rather as a convenience method. Users should use supplied
  // built-in operations for fast computations.
  // This is made private to prevent users from creating unphysical inertias by
  // setting one element at a time.
  T& operator()(int i, int j) {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_Bo_F_(i, j);
  }

  // Returns a constant reference to the underlying Eigen matrix. Notice that
  // since RotationalInertia only uses the
  // RotationalInertia::TriangularViewInUse portion of this
  // matrix, the RotationalInertia::TriangularViewNotInUse part will be set to
  // have NaN entries. Most users won't call this method. This method is
  // generally used in the implementation of class methods and
  // it's only useful to users for debugging.
  const Matrix3<T>& get_matrix() const { return I_Bo_F_; }

  /// Returns a const Eigen view expression to the symmetric part of the matrix
  /// in use by this RotationalInertia. Most users won't call this method.
  /// This method is generally used in the implementation of class methods and
  /// it's only useful to users for debugging.
  const Eigen::SelfAdjointView<const Matrix3<T>, TriangularViewInUse>
  get_symmetric_matrix_view() const {
    return I_Bo_F_.template selfadjointView<TriangularViewInUse>();
  }

  /// Returns a mutable Eigen view expression to the symmetric part of the
  /// matrix in use by RotationalInertia. Most users won't call this method.
  /// This method is generally used in the implementation of class methods and
  /// it's only useful to users for debugging.
  // Note: operator=() is not defined for Eigen::SelfAdjointView and therefore
  // we need to return a TriangularView here.
  Eigen::TriangularView<Matrix3<T>, TriangularViewInUse>
  get_mutable_symmetric_matrix_view() {
    return I_Bo_F_.template triangularView<TriangularViewInUse>();
  }

  // Inertia matrix about frame B's origin Bo expressed in frame F.
  // Frame F and origin Bo are implicit here, RotationalInertia only keeps track
  // of the inertia measures in this frame F. Users are responsible for keeping
  // track of the frame in which a particular inertia is expressed in.
  // Initially set to NaN to aid finding when by mistake we use the strictly
  // TriangularViewNotInUse portion of the matrix. Only the TriangularViewInUse
  // portion should be used.
  Matrix3<T> I_Bo_F_{Matrix3<T>::Constant(std::numeric_limits<
      typename Eigen::NumTraits<T>::Literal>::quiet_NaN())};
};

/// Insertion operator to write RotationalInertia's into a `std::ostream`.
/// Especially useful for debugging.
template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const RotationalInertia<T>& I) {
  // This allow us to set the number of decimal places to print.
  // 0 uses default precision.
  const std::streamsize precision = 0;

  int width = 0;
  std::streamsize old_precision = 0;
  std::ios_base::fmtflags old_flags = o.flags();
  if (precision) {
    old_precision = o.precision(precision);
    o << std::fixed;
  }

  // Computes largest width so that we can align columns for a prettier format.
  // Idea taken from: Eigen::internal::print_matrix() in Eigen/src/Core/IO.h
  for (int j = 0; j < I.cols(); ++j) {
    for (int i = 0; i < I.rows(); ++i) {
      std::stringstream sstr;
      sstr.copyfmt(o);
      sstr << I(i, j);
      width = std::max<int>(width, static_cast<int>(sstr.str().length()));
    }
  }

  // Outputs to stream.
  for (int i = 0; i < I.rows(); ++i) {
    o << "[";
    if (width) o.width(width);
    o << I(i, 0);
    for (int j = 1; j < I.cols(); ++j) {
      o << ", ";
      if (width) o.width(width);
      o << I(i, j);
    }
    o << "]" << std::endl;
  }
  if (precision) {
    o.precision(old_precision);
    o.flags(old_flags);
  }
  return o;
}

}  // namespace multibody
}  // namespace drake
