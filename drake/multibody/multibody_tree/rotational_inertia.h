#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

#include <Eigen/Eigenvalues>

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

template <typename T>
class RotationalInertia {
 public:
  enum {
    // By default RotationalInertia only works on the lower part of the
    // underlying Eigen matrix.
    // There is no strong reason for this particular choice.
    // It was observed however that Eigen sometimes uses the lower part of a
    // symmetric dense matrix. See Eigen::SelfAdjointEigenSolver. This is used
    // by RotationalInertia::CalcPrincipalMomentsOfInertia().
    TriangularViewInUse = Eigen::Lower,
    // The strictly lower part is set to NaN to quickly detect when used by
    // error.
    TriangularViewNotInUse = Eigen::StrictlyUpper
  };

  /// Default RotationalInertia constructor. Everything is left initialiezed to
  /// NaN for a quick detection of un-initialized values.
  RotationalInertia() {}

  // Default copy constructor and copy assignment.
  RotationalInertia(const RotationalInertia<T>& other) = default;
  RotationalInertia& operator=(const RotationalInertia<T>& other) = default;

  /// Creates a principal rotational inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  /// @see RotationalInertia::SolidSphere() and RotationalInertia::cube().
  RotationalInertia(const T& I) {
    SetZero();
    I_Bo_F_.diagonal().setConstant(I);
  }

  /// Create a principal axes rotational inertia matrix for wich off-diagonal
  /// elements are zero.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz) {
    SetZero();
    I_Bo_F_.diagonal() = Vector3<T>(Ixx, Iyy, Izz);
  }

  /// Creates a general rotational inertia matrix with non-zero off-diagonal
  /// elements.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz) {
    // The TriangularViewNotInUse is left initialized to NaN.
    auto& Iref = *this;
    // Let the operator(i, j) decide on what portion (upper/lower) to write on.
    Iref(0, 0) = Ixx; Iref(1, 1) = Iyy; Iref(2, 2) = Izz;
    Iref(0, 1) = Ixy; Iref(0, 2) = Ixz; Iref(1, 2) = Iyz;
  }

  int rows() const { return 3;}

  int cols() const { return 3;}

  Vector3<T> get_moments() const { return I_Bo_F_.diagonal(); }

  Vector3<T> get_products() const {
    // Let operator(int ,int) decide what portion (upper/lower) to use.
    const auto& Iref = *this;
    return Vector3<T>(Iref(0,1), Iref(0,2), Iref(1,2));
  }

  T& operator()(int i, int j) {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_Bo_F_(i, j);
  }

  const T& operator()(int i, int j) const {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_Bo_F_(i, j);
  }

  /// Returns a view to the symmetric part of the matrix in use by
  /// RotationalInertia.
  const Eigen::SelfAdjointView<const Matrix3<T>, TriangularViewInUse>
  get_symmetric_matrix_view() const {
    return I_Bo_F_.template selfadjointView<TriangularViewInUse>();
  }

  /// Returns a view to the symmetric part of the matrix in use by
  /// RotationalInertia.
  // Note: operator=() is not defined for Eigen::SelfAdjointView and therefore
  // we prefer to return a TriangularView here.
  Eigen::TriangularView<Matrix3<T>, TriangularViewInUse>
  get_mutable_symmetric_matrix_view() {
    return I_Bo_F_.template triangularView<TriangularViewInUse>();
  }

  /// Returns a constant reference to the underlying Eigen matrix. Notice that
  /// since RotationalInertia only uses the
  /// RotationalInertia::TriangularViewInUse portion of this
  /// matrix, the RotationalInertia::TriangularViewNotInUse part will be set to
  /// have NaN entries.
  const Matrix3<T>& get_matrix() const { return I_Bo_F_; }

  /// Get a copy to a full Matrix3 representation for this rotational inertia
  /// including both lower and upper triangular parts.
  Matrix3<T> CopyToFullMatrix3() const { return get_symmetric_matrix_view(); }

  bool IsApprox(const RotationalInertia& M_Bo_F, double tolerance) {
    return get_moments().isApprox(M_Bo_F.get_moments(), tolerance) &&
           get_products().isApprox(M_Bo_F.get_products(), tolerance);
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

  /// Computes the product from the right between this inertia with the
  /// vector @p w.
  /// This inertia and vector @p w must be expressed in the same frame.
  /// @param[in] w Vector to multiply from the right.
  /// @returns The product from the right of `this` inertia with @p w.
  Vector3<T> operator*(const Vector3<T>& w) const
  {
    return Vector3<T>(get_symmetric_matrix_view() * w);
  }

  /// Sets this inertia to have NaN entries. Typically used to quickly detect
  /// uninitialized values since NaN will trigger a chain of invalid
  /// computations that then can be tracked to the source.
  void SetToNaN() {
    I_Bo_F_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  void SetZero() {
    //I_Bo_F_.setConstant(std::numeric_limits<
    //    typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
    // RotationalInertia only works with the upper-triangular portion of the
    // underlying Eigen matrix. The lower part is set to NaN to quickly detect
    // when the lower part is mistakenly used.
    I_Bo_F_.template triangularView<TriangularViewInUse>() = Matrix3<T>::Zero();
  }

  /// Constructs a RotationalInertia from an Eigen matrix expression.
  template<typename Derived>
  RotationalInertia(const Eigen::MatrixBase<Derived>& m) : I_Bo_F_(m) {}

  /// Assignment operator from a general Eigen expression.
  // This method allows you to assign Eigen expressions to a RotationalInertia.
  template<typename Derived>
  RotationalInertia& operator=(const Eigen::MatrixBase<Derived>& EigenMatrix)
  {
    // Static asserts that EigenMatrix is of fixed size 3x3.
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    I_Bo_F_ = EigenMatrix;
    return *this;
  }

  bool IsNaN() const {
    using std::isnan;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        // We use operator()(int, int) here to automatically only check the
        // portion in use according to TriangularViewInUse.
        if (isnan(operator()(i, j))) return true;
      }
    }
    return false;
  }

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

  /// Performs a number of chekcs to verify that this is a physically valid
  /// rotational inertia.
  /// The chekcs performed are:
  /// - No NaN entries.
  /// - Non-negative diagonals.
  /// - Must satisfy triangle inequality.
  /// - Products of inertia are limited by moments (diagonal entries).
  bool IsPhysicallyValid() const {
    if (IsNaN()) return false;

    // Compute principal moments of inertia.
    Vector3<T> d;
    if (!CalcPrincipalMomentsOfInertia(&d)) return false;

    // Diagonals must be non-negative.
    if ((d.array() < T(0)).any() ) return false;

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

 private:
  static void check_and_swap(int* i, int* j) {
    const bool swap =
        (int(TriangularViewInUse) == int(Eigen::Upper) && *i > *j) ||
        (int(TriangularViewInUse) == int(Eigen::Lower) && *i < *j);
    if (swap) std::swap(*i , *j);
  }
  // Inertia matrix about frame B's origin Bo expressed in frame F.
  // Frame F and origin Bo are implicit here, RotationalInertia only keeps track
  // of the inertia measures in this frame F. Users are responsible for keeping 
  // track of the frame in which a particular inertia is expressed in.
  // Initially set to NaN to aid finding when by mistake we use the strictly
  // lower part of the matrix. Only the upper part should be used.
  Matrix3<T> I_Bo_F_{Matrix3<T>::Constant(std::numeric_limits<
      typename Eigen::NumTraits<T>::Literal>::quiet_NaN())};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <typename T>
inline RotationalInertia<T> operator*(
    const T& s, const RotationalInertia<T>& I_Bo_F) {
  RotationalInertia<T> sxI;
  sxI.get_mutable_symmetric_matrix_view() = s * I_Bo_F.get_matrix();
  return sxI;
}

template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const RotationalInertia<T>& I) {
  // This allow us to set the number of decimal places to print.
  // 0 uses default precision.
  const std::streamsize precision = 0;

  int width = 0;
  std::streamsize old_precision = 0;
  std::ios_base::fmtflags old_flags = o.flags();
  if(precision) {
    old_precision = o.precision(precision);
    o << std::fixed;
  }

  // Computes largest width so that we can align columns for a prettier format.
  // Idea taken from: Eigen::internal::print_matrix() in Eigen/src/Core/IO.h
  for(int j = 0; j < I.cols(); ++j) {
    for (int i = 0; i < I.rows(); ++i) {
      std::stringstream sstr;
      sstr.copyfmt(o);
      sstr << I(i, j);
      width = std::max<int>(width, int(sstr.str().length()));
    }
  }

  // Outputs to stream.
  for(int i = 0; i < I.rows(); ++i) {
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
  if(precision) {
    o.precision(old_precision);
    o.flags(old_flags);
  }
  return o;
}

}  // namespace multibody
}  // namespace drake
