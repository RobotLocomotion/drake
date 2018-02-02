#pragma once

#include <limits>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/cross_product.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

/// _Articulated %Body Inertia_ is the inertia that a body appears to have when
/// it is the base (or root) of a rigid-body system, also referred to as
/// _Articulated %Body_ in the context of articulated body algorithms.
/// The _Articulated %Body Inertia_ is a very useful multibody dynamics concept
/// that was introduced by Featherstone [Featherstone 1983] to develop the
/// remarkable `O(n)` Articulated Body Algorithm (ABA) for solving forward
/// dynamics. Recall that the Newton-Euler equations allow us to describe the
/// combined rotational and translational dynamics of a rigid body: <pre>
///   F_BBo_W = M_B_W * A_WB + b_Bo_W                                    (1)
/// </pre>
/// where the spatial inertia (see SpatialInertia) `M_B_W` of body B expressed
/// in the world frame W linearly relates the spatial acceleration (see
/// SpatialAcceleration) of body B in the world frame with the total applied
/// spatial forces (see SpatialForce) `F_BBo` on body B and where `b_Bo_W`
/// contains the velocity dependent gyroscopic terms.
///
/// A similar relationship is found for an articulated body with a rigid body B
/// at the base (or root). Even though the bodies in this multibody system are
/// allowed to have relative motions among them, there still is a linear
/// relationship between the spatial force `F_BBo_W` applied on this body and
/// the resulting acceleration `A_WB`: <pre>
///   F_BBo_W = P_B_W * A_WB + z_Bo_W                                       (2)
/// </pre>
/// where `P_B_W` is the articulated body inertia of body B and `z_Bo_W` is a
/// bias force that includes the gyroscopic and Coriolis forces and becomes zero
/// when all body velocities and all applied generalized forces outboard
/// from body B are zero [Jain 2010, §7.2.1]. The articulated body inertia
/// `P_B_W` is related to the multibody subsystem consisting only of bodies that
/// are outboard of body B. We refer to this subsystem as the
/// _articulated body subsystem_ associated with body B. Equation (2) describes
/// the acceleration response of body B, but also taking into account all
/// outboard bodies connected to B. A special case is that of an articulated
/// body composed of a single rigid body. For this special case, Eq. (2) reduces
/// to Eq. (1) for the dynamics of rigid body B. In other words, the ABI for an
/// articulated body consisting of a single rigid body exactly equals the
/// spatial inertia of that body.
///
/// Articulated body inertias are elements of ℝ⁶ˣ⁶ that, as for spatial
/// inertias, are symmetric and positive semi-definite. However, ABI objects
/// **are not** spatial inertias. The spatial inertia of a rigid body can be
/// described by a reduced set of ten parameters, namely the mass, center of
/// mass and the six components of the rotational inertia for that body.
/// However, this parametrization by ten parameters is just not possible for an
/// ABI and the full 21 elements of the symmetric `6x6` matrix must be specified
/// [Jain 2010, §6.4]. As a result ABI objects can have different properties
/// than spatial inertia objects. As an example, the apparent mass of an
/// articulated body will in general depend on the direction of the applied
/// force. That is, the simple relationship `F = m * a` is no longer valid for
/// an articulated body's center of mass (refer to the excellent example 7.1 in
/// [Featherstone 2008]).
///
/// We adopt the notation introduced by [Jain 2010] and generally we will use
/// an uppercase P to represent an ABI. Thus, in typeset material we use the
/// symbol @f$ [P^{A/Q}]_E @f$ to represent the spatial inertia of an
/// articulated body A, about a point Q, expressed in a frame E. For this
/// inertia, the monogram notation reads `P_AQ_E`.
///
/// @note This class does not implement any mechanism to track the frame E in
/// which an articulated body inertia is expressed or about what point is
/// computed. Methods and operators on this class have no means to determine
/// frame consistency through operations. It is therefore the responsibility of
/// users of this class to keep track of frames in which operations are
/// performed. We suggest doing that using disciplined notation, as described
/// above.
///
/// - [Featherstone 1983] Featherstone, R., 1983.
///     The calculation of robot dynamics using articulated-body inertias. The
///     International Journal of Robotics Research, 2(1), pp.13-30.
/// - [Featherstone 2008] Featherstone, R., 2008.
///     Rigid body dynamics algorithms. Springer.
/// - [Jain 2010]  Jain, A., 2010.
///     Robot and multibody dynamics: analysis and algorithms.
///     Springer Science & Business Media.
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
  /// inertia `M_SQ_E` for a body or composite body S, about point Q, and
  /// expressed in a frame E, this constructor creates an articulated body
  /// inertia about the same point Q and expressed in the same frame E.
  ///
  /// @param[in] M_SQ_E The spatial inertia of a body or composite body S about
  ///                   point Q and expressed in frame E.
  explicit ArticulatedBodyInertia(const SpatialInertia<T>& M_SQ_E) {
    matrix_.template triangularView<Eigen::Lower>() =
        M_SQ_E.CopyToFullMatrix6();
  }

  /// Constructs an articulated body inertia from an input matrix.
  ///
  /// This constructor checks for the physical validity of the resulting
  /// %ArticulatedBodyInertia with IsPhysicallyValid() and throws a
  /// std::runtime_error exception in the event the provided input matrix leads
  /// to a non-physically viable articulated body inertia.
  ///
  /// @param[in] matrix A matrix representing the articulated body inertia.
  explicit ArticulatedBodyInertia(const Matrix6<T>& matrix) {
    CheckInvariants(matrix);
    matrix_.template triangularView<Eigen::Lower>() = matrix;
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

  /// Performs a number of checks to verify that the input matrix is a
  /// physically valid articulated body inertia.
  ///
  /// The checks performed are:
  /// - The matrix is symmetric.
  /// - The matrix is semi-positive definite.
  ///
  /// @param[in] matrix A 6x6 matrix representing this articulated body inertia.
  bool IsPhysicallyValid(const Matrix6<T>& matrix) const {
    // Check if the matrix is symmetric.
    if (!(matrix - matrix.transpose()).isZero(1e-10)) return false;

    // Attempt the Robust Cholesky decomposition to test if the matrix is
    // positive semi-definite.
    const auto ldlt = matrix.ldlt();
    return ldlt.info() == Eigen::Success && ldlt.isPositive();
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
  /// [Featherstone 2008, example 7.1]. The composite articulated body inertia
  /// `P_CQ_E` is also about the same point Q and expressed in the same frame E
  /// as the addends.
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
  const Eigen::Product<Eigen::SelfAdjointView<const Matrix6<T>, Eigen::Lower>,
                       OtherDerived>
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
  const Eigen::Product<OtherDerived,
                       Eigen::SelfAdjointView<const Matrix6<T>, Eigen::Lower>>
  operator*(const Eigen::MatrixBase<OtherDerived>& lhs,
            const ArticulatedBodyInertia& rhs) {
    return lhs * rhs.matrix_.template selfadjointView<Eigen::Lower>();
  }

 private:
  // Make ArticulatedBodyInertia templated on every other scalar type a friend
  // of ArticulatedBodyInertia<T> so that cast<Scalar>() can access private
  // members of ArticulatedBodyInertia<T>.
  template <typename> friend class ArticulatedBodyInertia;

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

  // Checks that the ArticulatedBodyInertia is physically valid and throws an
  // exception if not. This is mostly used in Debug builds to throw an
  // appropriate exception.
  void CheckInvariants(const Matrix6<T>& matrix) const {
    if (!IsPhysicallyValid(matrix)) {
      throw std::runtime_error(
          "The resulting articulated body inertia is not physically valid. "
              "See ArticulatedBodyInertia::IsPhysicallyValid()");
    }
  }
};

}  // namespace multibody
}  // namespace drake
