#pragma once

#include <limits>

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
/// - [Featherstone 2008] Featherstone, R., 2008. Rigid body dynamics
///                       algorithms. Springer.
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
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyInertia)

  // Typedef for SelfAdjointView.
  typedef Eigen::SelfAdjointView<const Matrix6<T>, Eigen::Lower> AdjointView;

  /// Default ArticulatedBodyInertia constructor initializes all matrix values
  /// to NaN for a quick detection of uninitialized values.
  ArticulatedBodyInertia() = default;

  /// Constructs an articulated body inertia for an articulated body consisting
  /// of a single rigid body given its spatial inertia. From an input spatial
  /// inertia M_SQ_E for a body or composite body S, about point Q, and
  /// expressed in a frame E, this constructor creates an articulated body
  /// inertia about the same point Q and expressed in the same frame E.
  ///
  /// @param[in] M_SQ_E The spatial inertia of a body or composite body S about
  ///                   point Q and expressed in frame E.
  explicit ArticulatedBodyInertia(const SpatialInertia<T>& M_SQ_E) {
    matrix_.template triangularView<Eigen::Lower>() =
        M_SQ_E.CopyToFullMatrix6();
  }

  /// Copy to a full 6x6 matrix representation.
  Matrix6<T> CopyToFullMatrix6() const {
    Matrix6<T> P;
    P.template triangularView<Eigen::Lower>() = matrix_;
    P.template triangularView<Eigen::StrictlyUpper>() = matrix_.transpose();
    return P;
  }

  /// Given `this` articulated body inertia `P_AQ_E` for some articulated body
  /// A, computed about point Q, and expressed in frame E, this method uses the
  /// rigid body shift operator to compute the same articulated body inertia
  /// about a new point R. The result still is expressed in frame E.
  ///
  /// Mathematically, this is equivalent to:
  /// <pre>
  /// P_AR_E = Φ(P_QR_E) P_AQ_E Φ(p_QR_E)ᵀ
  /// </pre>
  /// where `Φ(p_QR_E)` is the rigid body shift operator as defined by
  /// [Jain 2010]. The definition of `Φ(p_QR_E)` uses `p_QR_E×`, which is the
  /// skew-symmetric cross product matrix (defined such that
  /// `a× b = a.cross(b)`).
  /// <pre>
  /// Φ(p_QR_E) =
  /// | I₃ p_QR_E× |
  /// | 0       I₃ |
  /// </pre>
  ///
  /// This operation is performed in-place modifying the original object.
  /// @see Shift() which does not modify this object.
  ///
  /// For details see Section 6.2.5, Page 105 of [Jain 2010].
  ///
  /// @param[in] p_QR_E Vector from the original about point Q to the new
  ///                   about point R, expressed in the same frame E `this`
  ///                   articulated body inertia is expressed in.
  /// @returns A reference to `this` articulated body inertia for articulated
  ///          body or system of articulated bodies A but now computed about
  ///          about a new point R.
  ArticulatedBodyInertia<T>& ShiftInPlace(const Vector3<T>& p_QR_E) {
    // We want to compute P_AR_E = Φ(P_QR_E) P_AQ_E Φ(p_QR_E)ᵀ. This can be
    // done efficiently by computing the lower left 3x3 block of P_SB_E and then
    // using that to compute the upper left 3x3 block. The lower right 3x3 block
    // remains the same.

    // Extract coefficients of p_QR_E.
    const T a = p_QR_E(0), b = p_QR_E(1), c = p_QR_E(2);

    // Partially update the upper left 3x3 block first to avoid copying data to
    // a new matrix.
    matrix_(0, 0) += matrix_(4, 0) * c - matrix_(5, 0) * b;
    matrix_(1, 0) += matrix_(4, 1) * c - matrix_(5, 1) * b;
    matrix_(2, 0) += matrix_(4, 2) * c - matrix_(5, 2) * b;
    matrix_(1, 1) += matrix_(5, 1) * a - matrix_(3, 1) * c;
    matrix_(2, 1) += matrix_(5, 2) * a - matrix_(3, 2) * c;
    matrix_(2, 2) += matrix_(3, 2) * b - matrix_(4, 2) * a;

    // Update lower left 3x3 block.
    matrix_(3, 0) += matrix_(4, 3) * c - matrix_(5, 3) * b;
    matrix_(4, 0) += matrix_(4, 4) * c - matrix_(5, 4) * b;
    matrix_(5, 0) += matrix_(5, 4) * c - matrix_(5, 5) * b;
    matrix_(3, 1) += matrix_(5, 3) * a - matrix_(3, 3) * c;
    matrix_(4, 1) += matrix_(5, 4) * a - matrix_(4, 3) * c;
    matrix_(5, 1) += matrix_(5, 5) * a - matrix_(5, 3) * c;
    matrix_(3, 2) += matrix_(3, 3) * b - matrix_(4, 3) * a;
    matrix_(4, 2) += matrix_(4, 3) * b - matrix_(4, 4) * a;
    matrix_(5, 2) += matrix_(5, 3) * b - matrix_(5, 4) * a;

    // Finish updating the upper left 3x3 block. Only consider values that are
    // in the lower triangular part of the matrix.
    matrix_(0, 0) += matrix_(4, 0) * c - matrix_(5, 0) * b;
    matrix_(1, 0) += matrix_(5, 0) * a - matrix_(3, 0) * c;
    matrix_(2, 0) += matrix_(3, 0) * b - matrix_(4, 0) * a;
    matrix_(1, 1) += matrix_(5, 1) * a - matrix_(3, 1) * c;
    matrix_(2, 1) += matrix_(3, 1) * b - matrix_(4, 1) * a;
    matrix_(2, 2) += matrix_(3, 2) * b - matrix_(4, 2) * a;

    return *this;
  }

  /// Given `this` articulated body inertia `P_AQ_E` for some articulated body
  /// A, computed about point Q, and expressed in frame E, this method uses the
  /// rigid body shift operator to compute the same articulated body inertia
  /// about a new point R. The result still is expressed in frame E.
  ///
  /// @see ShiftInPlace() for more details.
  ///
  /// @param[in] p_QR_E Vector from the original about point Q to the new
  ///                   about point R, expressed in the same frame E `this`
  ///                   articulated body inertia is expressed in.
  /// @retval `P_AR_E` This same articulated body inertia for articulated
  ///          body or system of articulated bodies A but now computed about
  ///          about a new point R.
  ArticulatedBodyInertia Shift(const Vector3<T>& p_QR_E) const {
    return ArticulatedBodyInertia(*this).ShiftInPlace(p_QR_E);
  }

  /// Adds in to this articulated body inertia `P_AQ_E` for an articulated body
  /// A about a point Q and expressed in a frame E the articulated body inertia
  /// `P_BQ_E` for a second articulated body B about the same point Q and
  /// expressed in the same frame E. The result is equivalent to the articulated
  /// body inertia `P_CQ_E` for the composite articulated body C which has at
  /// its base a rigid body composed of the bases of A and B welded together
  /// [Featherstone 2008]. The composite articulated body inertia `P_CQ_E` is
  /// also about the same point Q and expressed in the same frame E as the
  /// addends.
  ///
  /// @param[in] P_BQ_E An articulated body inertia of some articulated body B
  ///                   to be added to `this` articulated body inertia. It must
  ///                   be defined about the same point Q as `this` inertia,
  ///                   and expressed in the same frame E.
  /// @returns A reference to `this` articulated body inertia, which has been
  ///          updated to include the given articulated body inertia `P_BQ_E`.
  ///
  /// @warning This operation is only valid if both articulated body inertias
  ///          are computed about the same point Q and expressed in the same
  ///          frame E.
  ArticulatedBodyInertia<T>&
  operator+=(const ArticulatedBodyInertia<T>& P_BQ_E) {
    matrix_.template triangularView<Eigen::Lower>() = matrix_ + P_BQ_E.matrix_;
    return *this;
  }

  /// Multiplies `this` articulated body inertia on the right by a matrix or
  /// vector.
  ///
  /// @note This method does not evaluate the product immediately. Instead, it
  /// returns an intermediate Eigen quantity that can be optimized automatically
  /// during compile time.
  template<typename OtherDerived>
  const Eigen::Product<AdjointView, OtherDerived>
  operator*(const Eigen::MatrixBase<OtherDerived>& rhs) const {
    return matrix_.template selfadjointView<Eigen::Lower>() * rhs;
  }

  /// Multiplies `this` articulated body inertia on the left by a matrix or
  /// vector.
  ///
  /// @note This method does not evaluate the product immediately. Instead, it
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

  // The 6x6 internal matrix representation. Since the articulated body inertia
  // matrix is symmetric, only the lower triangular part of the matrix is used.
  // All elements of the articulated body inertia matrix are initially set to
  // NaN which helps ensure the strictly upper triangular part is never used.
  Matrix6<T> matrix_{Matrix6<T>::Constant(nan())};
};

}  // namespace multibody
}  // namespace drake
