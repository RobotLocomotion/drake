#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

/// This class represents the physical concept of a _Articulated Body Inertia_.
/// An articulated body inertia encapsulates the mass distribution, first
/// moment distribution, and rotational inertia of a body or composite
/// body S. Unlike spatial inertia, articulated body inertia does not a center
/// of mass.
///
/// An articulated body inertia is an element of ℝ⁶ˣ⁶ that is symmetric, and
/// positive semi-definite. While spatial inertia requires only 10 parameters
/// to describe, an articulated body inertia requires 21 parameters [Jain 2010].
/// This is represented internally as a `6x6` matrix whose entires above the
/// diagonal are set to NaN. This ensures that all computations operate
/// efficiently on a symmetric matrix.
///
/// In typeset material we use the symbol @f$ [P^{S/A}]_E @f$ to represent the
/// spatial inertia of a body or composite body S about point A, expressed in
/// frame E. For this inertia, the monogram notation reads `P_SA_E`.
///
/// @note This class does not implement any mechanism to track the frame E in
/// which an articulated body inertia is expressed or about what point is
/// computed. Methods and operators on this class have no means to determine
/// frame consistency through operations. It is therefore the responsibility of
/// users of this class to keep track of frames in which operations are
/// performed. We suggest doing that using disciplined notation, as described
/// above.
///
/// - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
///                algorithms. Springer Science & Business Media.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template<typename T>
class ArticulatedBodyInertia {
  // Typedef for SelfAdjointView.
  typedef Eigen::SelfAdjointView<const Matrix6<T>, Eigen::Lower> AdjointView;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyInertia)

  /// Default ArticulatedBodyInertia constructor initializes all matrix values
  /// to NaN for a quick detection of uninitialized values.
  ArticulatedBodyInertia() = default;

  /// Constructs an articulated body inertia for a physical body or composite
  /// body S, about a point A, and expressed in frame E from a `6x6` matrix.
  /// This method does not check for the physical validity of the input matrix.
  ///
  /// @param[in] P_SA_E The articulated body inertia of a body or composite body
  ///                   S about point A and expressed in frame E.
  ///
  /// @note Only the values in the lower triangular part of the matrix will be
  /// used to construct the articulated body inertia.
  explicit ArticulatedBodyInertia(const Matrix6<T>& P_SA_E) {
    matrix_.template triangularView<Eigen::Lower>() = P_SA_E;
  }

  /// Constructs an articulated body inertia from the spatial inertia of a
  /// physical body or composite body S, about a point A, and expressed in
  /// frame E. This method does not check for the physical validity of the input
  /// matrix.
  ///
  /// @param[in] M_SA_E The spatial inertia of a body or composite body S about
  ///                   point A and expressed in frame E.
  explicit ArticulatedBodyInertia(const SpatialInertia<T>& M_SA_E) {
    matrix_.template triangularView<Eigen::Lower>() =
        M_SA_E.CopyToFullMatrix6();
  }

  /// Copy to a full 6x6 matrix representation.
  Matrix6<T> CopyToFullMatrix6() const {
    Matrix6<T> P;
    P.template triangularView<Eigen::Lower>() = matrix_;
    P.template triangularView<Eigen::StrictlyUpper>() = matrix_.transpose();
    return P;
  }

  /// Given `this` articulated body inertia `M_SA_E` for some body or composite
  /// body S, computed about point A, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// articulated body inertia about a new point B. The result still is
  /// expressed in frame E.
  ///
  /// This operation is performed in-place modifying the original object.
  /// @see Shift() which does not modify this object.
  ///
  /// For details see Section 2.1.2, p. 20 of [Jain 2010].
  ///
  /// @param[in] p_AB_E Vector from the original about point A to the new
  ///                   about point B, expressed in the same frame E `this`
  ///                   articulated body inertia is expressed in.
  /// @returns A reference to `this` articulated body inertia for body or
  ///          composite body S but now computed about about a new point B.
  ArticulatedBodyInertia<T>& ShiftInPlace(const Vector3<T>& p_AB_E) {
    using math::VectorToSkewSymmetric;

    // Construct shift matrix.
    Matrix6<T> phi_AB = Matrix6<T>::Identity();
    phi_AB.template block<3, 3>(0, 3) = VectorToSkewSymmetric(-p_AB_E);

    // Perform shift.
    matrix_.template triangularView<Eigen::Lower>() = phi_AB *
        matrix_.template selfadjointView<Eigen::Lower>() * phi_AB.transpose();

    return *this;
  }

  /// Given `this` articulated body inertia `M_SA_E` for some body or composite
  /// body S, computed about point A, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// articulated body inertia about a new point B. The result still is
  /// expressed in frame E.
  ///
  /// @see ShiftInPlace() for more details.
  ///
  /// @param[in] p_AB_E Vector from the original about point A to the new
  ///                   about point B, expressed in the same frame E `this`
  ///                   articulated body inertia is expressed in.
  /// @retval `P_SB_E` This same articulated body inertia for body or composite
  ///                  body S but computed about about a new point B.
  ArticulatedBodyInertia Shift(const Vector3<T>& p_AB_E) const {
    return ArticulatedBodyInertia(*this).ShiftInPlace(p_AB_E);
  }

  /// Given `this` articulated body inertia `P_SA_E` for some body or composite
  /// body S, taken about a point A, and expressed in frame E, this method
  /// computes the same inertia re-expressed in another frame F.
  ///
  /// This operation is performed in-place modifying the original object.
  /// @see ReExpress() which does not modify this object.
  ///
  /// @param[in] R_FE Rotation matrix from frame E to frame F.
  /// @returns A reference to `this` articulated body inertia about the same
  ///          point A but now re-expressed in frame F, that is, `P_SA_F`.
  ///
  /// @warning This method does not check whether the input matrix `R_FE`
  /// represents a valid rotation or not. It is the responsibility of users to
  /// provide valid rotation matrices.
  ArticulatedBodyInertia<T>& ReExpressInPlace(const Matrix3<T>& R_FE) {
    // Construct shift matrix.
    Matrix6<T> phi_AB = Matrix6<T>::Zero();
    phi_AB.template block<3, 3>(0, 0) = R_FE;
    phi_AB.template block<3, 3>(3, 3) = R_FE;

    // Perform shift.
    matrix_.template triangularView<Eigen::Lower>() = phi_AB *
        matrix_.template selfadjointView<Eigen::Lower>() * phi_AB.transpose();

    return *this;
  }

  /// Given `this` articulated body inertia `P_SA_E` for some body or composite
  /// body S, taken about a point A, and expressed in frame E, this method
  /// computes the same inertia re-expressed in another frame F.
  ///
  /// @see ReExpressInPlace() for more details.
  ///
  /// @param[in] R_FE Rotation matrix from frame E to frame F.
  /// @retval P_SA_F The same spatial inertia of S about A but now
  ///                re-expressed in frame F.
  ArticulatedBodyInertia<T> ReExpress(const Matrix3<T>& R_FE) const {
    return ArticulatedBodyInertia(*this).ReExpressInPlace(R_FE);
  }

  /// Adds in an articulated body inertia to `this` articulated body inertia.
  /// @param[in] P_TA_E An articulated body inertia of some body T to be added
  ///                   to `this` articulated body inertia. It must be defined
  ///                   about the same point A as `this` inertia, and expressed
  ///                   in the same frame E.
  /// @returns A reference to `this` artculated body inertia, which has been
  ///          updated to include the given articulated body inertia `S_TA_E`.
  ///
  /// @warning This operation is only valid if both articulated body inertias
  /// are computed about the same point A and expressed in the same frame E.
  ArticulatedBodyInertia<T>& operator+=(const ArticulatedBodyInertia<T>& P_BP_E)
  {
    matrix_.template triangularView<Eigen::Lower>() = matrix_ + P_BP_E.matrix_;
    return *this;
  }

  /// Multiplies `this` articulated body inertia on the right by a matrix or
  /// vector. This method does not construct the full inertia matrix, as it
  /// only operates on the lower triangular region.
  ///
  /// @note This method does not evaulate the product immediately. Instead, it
  /// returns an intermediate Eigen quantity that can be optimized automatically
  /// during compile time.
  template<typename OtherDerived>
  const Eigen::Product<AdjointView, OtherDerived>
  operator*(const Eigen::MatrixBase<OtherDerived>& rhs) const {
    return matrix_.template selfadjointView<Eigen::Lower>() * rhs;
  }

  /// Multiplies `this` articulated body inertia on the left by a matrix or
  /// vector. This method does not construct the full inertia matrix, as it
  /// only operates on the lower triangular region.
  ///
  /// @note This method does not evaulate the product immediately. Instead, it
  /// returns an intermediate Eigen quantity that can be optimized automatically
  /// during compile time.
  template<typename OtherDerived> friend
  const Eigen::Product<OtherDerived, AdjointView>
  operator*(const Eigen::MatrixBase<OtherDerived>& lhs,
            const ArticulatedBodyInertia& rhs) {
    return lhs * rhs.matrix_.template selfadjointView<Eigen::Lower>();
  }

 private:
  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Internal matrix representation.
  Matrix6<T> matrix_{Matrix6<T>::Constant(nan())};
};

} // namespace multibody
} // namespace drake