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
/// moment distribution, and rotational inertia of an articulated body A.
/// Unlike spatial inertia, articulated body inertia does not provide a center
/// of mass.
///
/// An articulated body inertia is an element of ℝ⁶ˣ⁶ that is symmetric and
/// positive semi-definite. While spatial inertia requires only 10 parameters
/// to describe, an articulated body inertia requires 21 parameters [Jain 2010].
///
/// In typeset material we use the symbol @f$ [P^{A/Q}]_E @f$ to represent the
/// articulated body inertia of an articulated body A, about point Q, and
/// expressed in frame E. For this inertia, the monogram notation reads
/// `P_AQ_E`.
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

  /// Returns a new %ArticulatedBodyInertia object templated on `Scalar` with
  /// casted values of `this` articulated body inertia.
  ///
  /// @tparam Scalar The scalar type on which the new articulated body inertia
  /// will be templated.
  ///
  /// @note `ArticulatedBodyInertia<From>::cast<To>()` creates a new
  /// `ArticulatedBodyInertia<To>` from an `ArticulatedBodyInertia<From>` but
  /// only if type `To` is constructible from type `From`. As an example of
  /// this, `ArticulatedBodyInertia<double>::cast<AutoDiffXd>()` is valid since
  /// `AutoDiffXd a(1.0)` is valid. However,
  /// `ArticulatedBodyInertia<AutoDiffXd>::cast<double>()` is not.
  template <typename Scalar>
  ArticulatedBodyInertia<Scalar> cast() const {
    ArticulatedBodyInertia<Scalar> P = ArticulatedBodyInertia<Scalar>();
    P.matrix_.template triangularView<Eigen::Lower>() =
        matrix_.template cast<Scalar>();
    return P;
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
  /// P_AR_E = Φ(P_RQ_E) P_AQ_E Φ(p_RQ_E)ᵀ
  /// </pre>
  /// where `Φ(p_RQ_E)` is the rigid body shift operator as defined by
  /// [Jain 2010]. The definition of `Φ(p_RQ_E)` uses `p_QR_E×`, which is the
  /// skew-symmetric cross product matrix (defined such that
  /// `a× b = a.cross(b)`).
  /// <pre>
  /// Φ(p_RQ_E) =
  /// | I₃  p_RQ_E× |
  /// | 0        I₃ |
  /// </pre>
  /// where `p_RQ_E× = -p_QR_E×`.
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
  ///          body A but now computed about a new point R.
  ArticulatedBodyInertia<T>& ShiftInPlace(const Vector3<T>& p_QR_E) {
    // We want to compute P_AR_E = Φ(P_RQ_E) P_AQ_E Φ(p_RQ_E)ᵀ. This can be
    // done efficiently using block multiplication.
    //
    // This articulated body inertia has the form
    // P_AQ_E =
    // | J   F |
    // | Fᵀ  M |.
    //
    // Using block multiplication, we arrive at
    // Φ(P_RQ_E) P_AQ_E Φ(p_RQ_E)ᵀ =
    // | J + (p×)Fᵀ - Fp(p×)  Fp |
    // | Fpᵀ                   M |
    // where p× is used to denote p_RQ_E× and Fp = F + (p×)M.

    // Some aliases definitions.
    const Vector3<T> px = -p_QR_E;
    Eigen::TriangularView<Eigen::Block<Matrix6<T>, 3, 3>, Eigen::Lower> J =
        matrix_.template block<3, 3>(0, 0)
            .template triangularView<Eigen::Lower>();
    Eigen::Transpose<Eigen::Block<Matrix6<T>, 3, 3>> F =
        matrix_.template block<3, 3>(3, 0).transpose();
    Eigen::Block<Matrix6<T>, 3, 3> Fp = matrix_.template block<3, 3>(0, 3);
    Matrix3<T> M = matrix_.template block<3, 3>(3, 3)
        .template selfadjointView<Eigen::Lower>();

    // Compute common term Fp = F + (p×)M.
    // The minus on p× is needed because we are doing each column times p×, in
    // that order.
    // Costs 36 flops (27 for cross product and 9 for addition).
    Fp = F + M.colwise().cross(-px);

    // Update J according to J + (p×)Fᵀ - Fp(p×) = J - ((F(p×))ᵀ + Fp(p×)).
    // Costs 66 flops (54 for cross product and 12 for triangular addition).
    // TODO(bobbyluig): Optimize to only compute lower triangular region.
    J -= (F.rowwise().cross(px).transpose() + Fp.rowwise().cross(px));

    // Overwrite F (in the lower left) with Fp. M doesn't change.
    F = Fp;

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
  /// @retval P_AR_E This same articulated body inertia for articulated body
  ///         A but now computed about about a new point R.
  ArticulatedBodyInertia<T> Shift(const Vector3<T>& p_QR_E) const {
    return ArticulatedBodyInertia<T>(*this).ShiftInPlace(p_QR_E);
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

 private:
  // Make ArticulatedBodyInertia templated on every other scalar type a friend
  // of ArticulatedBodyInertia<T> so that cast<Scalar>() can access private
  // members of ArticulatedBodyInertia<T>.
  template <typename> friend class ArticulatedBodyInertia;

  // Typedef for SelfAdjointView.
  typedef Eigen::SelfAdjointView<const Matrix6<T>, Eigen::Lower> AdjointView;

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // The 6x6 internal matrix representation. Since the articulated body inertia
  // matrix is symmetric, only the lower triangular part of the matrix is used.
  // All elements of the articulated body inertia matrix are initially set to
  // NaN.
  Matrix6<T> matrix_{Matrix6<T>::Constant(nan())};

 public:
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
};

}  // namespace multibody
}  // namespace drake
