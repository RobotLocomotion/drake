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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationalInertia)

  /// Default RotationalInertia constructor. All entries are set to NaN for a
  /// quick detection of un-initialized values.
  RotationalInertia() {}

  /// Creates a principal rotational inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  /// Aborts if `I` is non-positive.
  explicit RotationalInertia(const T& I) {
    DRAKE_DEMAND(I > T(0));
    SetZero();
    I_Bo_F_.diagonal().setConstant(I);
  }

  /// Creates a principal axes rotational inertia matrix for which the products
  /// of inertia are zero and the moments of inertia are given by `Ixx`, `Iyy`
  /// and `Izz`.
  /// Aborts if any of the provided moments is non-positive.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz) {
    DRAKE_DEMAND(Ixx > T(0));
    DRAKE_DEMAND(Iyy > T(0));
    DRAKE_DEMAND(Izz > T(0));
    SetZero();
    I_Bo_F_.diagonal() = Vector3<T>(Ixx, Iyy, Izz);
  }

  /// Creates a general rotational inertia matrix with non-zero off-diagonal
  /// elements where the six components of the rotational intertia on a given
  /// frame `E` need to be provided.
  /// Aborts if the resulting inertia is invalid according to
  /// IsPhysicallyValid().
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz) {
    // The upper part is left initialized to NaN.
    I_Bo_F_(0, 0) = Ixx; I_Bo_F_(1, 1) = Iyy; I_Bo_F_(2, 2) = Izz;
    I_Bo_F_(1, 0) = Ixy; I_Bo_F_(2, 0) = Ixz; I_Bo_F_(2, 1) = Iyz;
    DRAKE_DEMAND(IsPhysicallyValid());
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
    // The strictly-upper triangle is left initialized to NaN to quickly detect
    // if this part is mistakenly used.
    I_Bo_F_.template triangularView<Eigen::Lower>() = Matrix3<T>::Zero();
  }

  /// Returns `true` if any of the elements in this rotational inertia is NaN
  /// and `false` otherwise.
  bool IsNaN() const {
    using std::isnan;
    // Only check the lower entries.
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j <= i; ++j) {
        if (isnan(I_Bo_F_(i, j))) return true;
      }
    }
    return false;
  }

  /// For `this` inertia about a given point `P` and expressed in a frame `E`,
  /// this method computes the principal moments of inertia of `this` rotational
  /// inertia about the same point `P` and expressed on a frame with origin at
  /// `P` and aligned with the principal axes.
  /// The computed principal moments are placed into @p principal_moments sorted
  /// in ascending order.
  /// @returns `true` if succesful and `false` otherwise.
  Vector3<T> CalcPrincipalMomentsOfInertia() const {
    // Note: Eigen's SelfAdjointEigenSolver only works with the lower diagonal
    // part of the matrix.
    Eigen::SelfAdjointEigenSolver<Matrix3<T>> solver(
        get_matrix(), Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error(
          "Error: In RotationalInertia::CalcPrincipalMomentsOfInertia(). "
              "Most likely your rotational inertia is not positive definite.");
    }
    return solver.eigenvalues();
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
    Vector3<T> d = CalcPrincipalMomentsOfInertia();

    // Principal moments must be strictly positive.
    if ((d.array() <= T(0)).any() ) return false;

    // Checks triangle inequality
    if (!( d[0] + d[1] >= d[2] && d[0] + d[2] >= d[1] && d[1] + d[2] >= d[0]))
      return false;

    return true;  // All tests passed.
  }

  /// Given this rotational inertia `I_Bo_F` about `Bo` and expressed in frame
  /// `F`, this method computes the same inertia re-expressed in another
  /// frame `A` as `I_Bo_A = R_AF * I_Bo_F * (R_AF)ᵀ`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A reference to `this` rotational inertia about `Bo` but now
  /// re-expressed in frame `A`.
  RotationalInertia& ReExpressInPlace(const Matrix3<T>& R_AF) {
    // There is an interesting discussion on Eigen's forum here:
    // https://forum.kde.org/viewtopic.php?f=74&t=97282
    // That discussion tell us that really here we don't have a significant
    // performance gain for small matrices when writing on one of the triangular
    // parts only. Then gain is in accuracy, by having RotationalInertia to only
    // deal with one triangular portion of the matrix.

    // Local copy to avoid aliasing since there is aliasing when using the
    // triangular view.
    Matrix3<T> I_Bo_A;
    I_Bo_A.noalias() =
        R_AF * I_Bo_F_.template selfadjointView<Eigen::Lower>() *
            R_AF.transpose();
    this->get_mutable_symmetric_matrix_view() = I_Bo_A;
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
  static void check_and_swap(int* i, int* j) { if (*i < *j) std::swap(*i, *j); }

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
  const Eigen::SelfAdjointView<const Matrix3<T>, Eigen::Lower>
  get_symmetric_matrix_view() const {
    return I_Bo_F_.template selfadjointView<Eigen::Lower>();
  }

  /// Returns a mutable Eigen view expression to the symmetric part of the
  /// matrix in use by RotationalInertia. Most users won't call this method.
  /// This method is generally used in the implementation of class methods and
  /// it's only useful to users for debugging.
  // Note: operator=() is not defined for Eigen::SelfAdjointView and therefore
  // we need to return a TriangularView here.
  Eigen::TriangularView<Matrix3<T>, Eigen::Lower>
  get_mutable_symmetric_matrix_view() {
    return I_Bo_F_.template triangularView<Eigen::Lower>();
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
