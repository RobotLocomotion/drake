#pragma once

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/math/cross_product.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {

/// This class represents the physical concept of a _Spatial Inertia_. A
/// spatial inertia (or spatial mass matrix) encapsulates the mass, center of
/// mass, and rotational inertia of the mass distribution of a body or composite
/// body S, where with "composite body" we mean a collection of bodies welded
/// together containing at least one body (throughout this documentation "body"
/// is many times used instead of "composite body" but the same concepts apply
/// to a collection of bodies as well.)
/// A spatial inertia is an element of ℝ⁶ˣ⁶ that is symmetric, and positive
/// semi-definite. It logically consists of `3x3` sub-matrices arranged like
/// so, [Jain 2010]:
/// <pre>
///              Spatial mass matrix
///           ------------ ------------
///        0 |            |            |
///        1 |    I_SP    | m p_PScm×  |
///        2 |            |            |
///           ------------ ------------
///        3 |            |            |
///        4 | -m p_PScm× |     m Id   |
///        5 |            |            |
///           ------------ ------------
///                Symbol: M
/// </pre>
/// where, with the monogram notation described in
/// @ref multibody_spatial_inertia, `I_SP` is the rotational inertia of body or
/// composite body S computed about a point P, m is the mass of this composite
/// body, `p_PScm` is the position vector from point P to the center of mass
/// `Scm` of the composite body S with `p_PScm×` denoting its skew-symmetric
/// cross product matrix (defined such that `a× b = a.cross(b)`), and `Id` is
/// the identity matrix in ℝ³ˣ³. See Section 2.1, p. 17 of [Jain 2010].
/// The logical arrangement as shown above is chosen to be consistent with our
/// logical arrangement for spatial vectors as documented in
/// @ref multibody_spatial_algebra for which the rotational component comes
/// first followed by the translational component.
///
/// In typeset material we use the symbol @f$ [M^{S/P}]_E @f$ to represent the
/// spatial inertia of a body or composite body S about point P, expressed in
/// frame E. For this inertia, the monogram notation reads `M_SP_E`. If the
/// point P is fixed to a body B, we write that point as @f$ B_P @f$ which
/// appears in code and comments as `Bp`. So if the body or composite body is B
/// and the about point is `Bp`, the monogram notation reads `M_BBp_E`, which
/// can be abbreviated to `M_Bp_E` since the about point `Bp` also identifies
/// the body. Common cases are that the about point is the origin `Bo` of the
/// body, or it's the center of mass `Bcm` for which the rotational inertia in
/// monogram notation would read as `I_Bo_E` and `I_Bcm_E`, respectively.
/// Given `M_BP_E` (@f$[M^{B/P}]_E@f$), the rotational inertia of this spatial
/// inertia is `I_BP_E` (@f$[I^{B/P}]_E@f$) and the position vector of the
/// center of mass measured from point P and expressed in E is `p_PBcm_E`
/// (@f$[^Pp^{B_{cm}}]_E@f$).
///
/// @note This class does not implement any mechanism to track the frame E in
/// which a spatial inertia is expressed or about what point is computed.
/// Methods and operators on this class have no means to determine frame
/// consistency through operations. It is therefore the responsibility of users
/// of this class to keep track of frames in which operations are performed. We
/// suggest doing that using disciplined notation, as described above.
///
/// @note Several methods in this class throw a std::exception for invalid
/// rotational inertia operations in debug releases only.  This provides speed
/// in a release build while facilitating debugging in debug builds.
/// In addition, these validity tests are only performed for scalar types for
/// which drake::scalar_predicate<T>::is_bool is `true`. For instance, validity
/// checks are not performed when T is symbolic::Expression.
///
/// - [Jain 2010]  Jain, A., 2010. Robot and multibody dynamics: analysis and
///                algorithms. Springer Science & Business Media.
///
/// @tparam_default_scalar
template <typename T>
class SpatialInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialInertia)

  /// Creates a spatial inertia for a physical body or composite body S about a
  /// point P from a given mass, center of mass, and central rotational inertia.
  /// For example, this method creates a body's SpatialInertia about its body
  /// origin Bo from the body's mass, position vector from Bo to the body's
  /// center of mass, and rotational inertia about the body's center of mass.
  ///
  /// This method checks for the physical validity of the resulting
  /// %SpatialInertia with IsPhysicallyValid() and throws a std::runtime_error
  /// exception in the event the provided input parameters lead to a
  /// non-physically viable spatial inertia.
  ///
  /// @param[in] mass The mass of the body or composite body S.
  /// @param[in] p_PScm_E The position vector from point P to point `Scm`
  ///                     (S's center of mass), expressed in a frame E.
  /// @param[in] I_SScm_E S's RotationalInertia about Scm, expressed in frame E.
  /// @retval M_SP_E S's spatial inertia about point P, expressed in frame E.
  static SpatialInertia MakeFromCentralInertia(const T& mass,
      const Vector3<T>& p_PScm_E, const RotationalInertia<T>& I_SScm_E) {
    const RotationalInertia<T> I_SP_E =
        I_SScm_E.ShiftFromCenterOfMass(mass, p_PScm_E);
    UnitInertia<T> G_SP_E;
    G_SP_E.SetFromRotationalInertia(I_SP_E, mass);
    return SpatialInertia(mass, p_PScm_E, G_SP_E);
  }

  /// Default SpatialInertia constructor initializes mass, center of mass and
  /// rotational inertia to invalid NaN's for a quick detection of
  /// uninitialized values.
  SpatialInertia() {}

  /// Constructs a spatial inertia for a physical body or composite body S about
  /// a point P from a given mass, center of mass and rotational inertia. The
  /// center of mass is specified by the position vector `p_PScm_E` from point P
  /// to the center of mass point `Scm`, expressed in a frame E.
  /// The rotational inertia is provided as the UnitInertia `G_SP_E` of the body
  /// or composite body S computed about point P and expressed in frame E.
  ///
  /// @note The third argument of this constructor is unusual in that it is an
  /// UnitInertia (not a traditional RotationalInertia) and its inertia is about
  /// the arbitrary point P (not Scm -- S's center of mass).
  /// @see MakeFromCentralInertia a factory method with traditional utility.
  ///
  /// This constructor checks for the physical validity of the resulting
  /// %SpatialInertia with IsPhysicallyValid() and throws a std::runtime_error
  /// exception in the event the provided input parameters lead to
  /// non-physically viable spatial inertia.
  ///
  /// @param[in] mass The mass of the body or composite body S.
  /// @param[in] p_PScm_E The position vector from point P to the center of mass
  ///                     of body or composite body S expressed in frame E.
  /// @param[in] G_SP_E UnitInertia of the body or composite body S computed
  ///                   about origin point P and expressed in frame E.
  SpatialInertia(
      const T& mass, const Vector3<T>& p_PScm_E, const UnitInertia<T>& G_SP_E) :
      mass_(mass), p_PScm_E_(p_PScm_E), G_SP_E_(G_SP_E) {
    CheckInvariants();
  }

  /// Returns a new %SpatialInertia object templated on `Scalar` initialized
  /// from the value of `this` spatial inertia.
  ///
  /// @tparam Scalar The scalar type on which the new spatial inertia will
  /// be templated.
  ///
  /// @note `SpatialInertia<From>::cast<To>()` creates a new
  /// `SpatialInertia<To>` from a `SpatialInertia<From>` but only if
  /// type `To` is constructible from type `From`.
  /// This cast method works in accordance with Eigen's cast method for Eigen's
  /// objects that underlie this %SpatialInertia.  For example, Eigen
  /// currently allows cast from type double to AutoDiffXd, but not vice-versa.
  template <typename Scalar>
  SpatialInertia<Scalar> cast() const {
    return SpatialInertia<Scalar>(
        get_mass(),
        get_com().template cast<Scalar>(),
        get_unit_inertia().template cast<Scalar>());
  }

  /// Get a constant reference to the mass of this spatial inertia.
  const T& get_mass() const { return mass_;}

  /// Get a constant reference to the position vector `p_PScm_E` from the
  /// _about point_ P to the center of mass `Scm` of the body or composite body
  /// S, expressed in frame E. See the documentation of this class for details.
  const Vector3<T>& get_com() const { return p_PScm_E_;}

  /// Computes the center of mass moment vector `mass * p_PScm_E` given the
  /// position vector `p_PScm_E` from the _about point_ P to the center of mass
  /// `Scm` of the body or composite body S, expressed in frame E. See the
  /// documentation of this class for details.
  Vector3<T> CalcComMoment() const { return mass_ * p_PScm_E_;}

  /// Get a constant reference to the unit inertia `G_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this class for details.
  const UnitInertia<T>& get_unit_inertia() const { return G_SP_E_;}

  /// Computes the rotational inertia `I_SP_E = mass * G_SP_E` of this
  /// spatial inertia, computed about point P and expressed in frame E. See the
  /// documentation of this class for details.
  RotationalInertia<T> CalcRotationalInertia() const { return mass_ * G_SP_E_;}

  /// Returns `true` if any of the elements in this spatial inertia is NaN
  /// and `false` otherwise.
  boolean<T> IsNaN() const {
    using std::isnan;
    return isnan(mass_) || G_SP_E_.IsNaN() ||
        any_of(p_PScm_E_, [](auto x){ return isnan(x); });
  }

  /// Performs a number of checks to verify that this is a physically valid
  /// spatial inertia.
  /// The checks performed are:
  ///
  /// - No NaN entries.
  /// - Non-negative mass.
  /// - Non-negative principal moments about the center of mass.
  /// - Principal moments about the center of mass must satisfy the triangle
  ///   inequality:
  ///   - `Ixx + Iyy >= Izz`
  ///   - `Ixx + Izz >= Iyy`
  ///   - `Iyy + Izz >= Ixx`
  ///
  /// These are the tests performed by
  /// RotationalInertia::CouldBePhysicallyValid() which become a sufficient
  /// condition when performed on a rotational inertia about a body's center of
  /// mass.
  /// @see RotationalInertia::CouldBePhysicallyValid().
  boolean<T> IsPhysicallyValid() const {
    // The tests in RotationalInertia become a sufficient condition when
    // performed on a rotational inertia computed about a body's center of mass.
    const UnitInertia<T> G_SScm_E = G_SP_E_.ShiftToCenterOfMass(p_PScm_E_);
    return !IsNaN() && mass_ >= T(0) && G_SScm_E.CouldBePhysicallyValid();
  }

  /// Copy to a full 6x6 matrix representation.
  Matrix6<T> CopyToFullMatrix6() const {
    using drake::math::VectorToSkewSymmetric;
    Matrix6<T> M;
    M.template block<3, 3>(0, 0) = mass_ * G_SP_E_.CopyToFullMatrix3();
    M.template block<3, 3>(0, 3) = mass_ * VectorToSkewSymmetric(p_PScm_E_);
    M.template block<3, 3>(3, 0) = -M.template block<3, 3>(0, 3);
    M.template block<3, 3>(3, 3) = mass_ * Matrix3<T>::Identity();
    return M;
  }

  /// Sets `this` spatial inertia to have NaN entries. Typically used for quick
  /// detection of uninitialized values.
  void SetNaN() {
    mass_ = nan();
    p_PScm_E_.setConstant(nan());
    G_SP_E_.SetToNaN();
  }

  /// Adds in a spatial inertia to `this` spatial inertia.
  /// @param[in] M_BP_E A spatial inertia of some body B to be added to
  ///                  `this` spatial inertia. It must be defined about the
  ///                   same point P as `this` inertia, and expressed in the
  ///                   same frame E.
  /// @returns A reference to `this` spatial inertia, which has been updated
  ///          to include the given spatial inertia `M_BP_E`.
  ///
  /// @note Given that the composition of spatial inertias is not well defined
  /// for massless bodies, this composition of the spatial inertias performs the
  /// arithmetic average of the center of mass position vector (get_com()) and
  /// unit inertia (get_unit_inertia()) when the two spatial inertias have zero
  /// mass (get_mass()). This is only valid in the limit to zero mass for two
  /// bodies with the same mass. This special case allows the composition of
  /// spatial inertias in the common case of a kinematic chain of massless
  /// bodies.
  ///
  /// @warning This operation is only valid if both spatial inertias are
  /// computed about the same point P and expressed in the same frame E.
  /// Considering `this` spatial inertia to be `M_SP_E` for some body or
  /// composite body S, about some point P, the supplied spatial inertia
  /// `M_BP_E` must be for some other body or composite body B about the _same_
  /// point P; B's inertia is then included in S.
  SpatialInertia& operator+=(const SpatialInertia<T>& M_BP_E) {
    const T total_mass = get_mass() + M_BP_E.get_mass();
    if (total_mass != 0) {
      p_PScm_E_ = (CalcComMoment() + M_BP_E.CalcComMoment()) / total_mass;
      G_SP_E_.SetFromRotationalInertia(
          CalcRotationalInertia() + M_BP_E.CalcRotationalInertia(), total_mass);
    } else {
      // Compose the spatial inertias of two massless bodies in the limit when
      // the two bodies have the same mass. In this limit, p_PScm_E_ and G_SP_E_
      // are the arithmetic mean of the constituent COMs and unit inertias.
      p_PScm_E_ = 0.5 * (get_com() + M_BP_E.get_com());
      G_SP_E_.SetFromRotationalInertia(
          get_unit_inertia() + M_BP_E.get_unit_inertia(), 2.0);
    }
    mass_ = total_mass;
    return *this;
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// taken about a point P and expressed in frame E, this method computes the
  /// same inertia re-expressed in another frame A.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AE Rotation matrix from frame E to frame A.
  /// @returns A reference to `this` rotational inertia about the same point P
  ///          but now re-expressed in frame A, that is, `M_SP_A`.
  SpatialInertia& ReExpressInPlace(const math::RotationMatrix<T>& R_AE) {
    p_PScm_E_ = R_AE * p_PScm_E_;    // Now p_PScm_A
    G_SP_E_.ReExpressInPlace(R_AE);  // Now I_SP_A
    return *this;                    // Now M_SP_A
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// taken about a point P and expressed in frame E, this method computes the
  /// same inertia re-expressed in another frame A.
  /// @param[in] R_AE RotationMatrix relating frames A and E.
  /// @retval M_SP_A The same spatial inertia of S about P but now
  ///                re-expressed in frame A.
  /// @see ReExpressInPlace() for details.
  SpatialInertia ReExpress(const math::RotationMatrix<T>& R_AE) const {
    return SpatialInertia(*this).ReExpressInPlace(R_AE);
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// computed about point P, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// spatial inertia about a new point Q. The result still is expressed in
  /// frame E.
  /// This operation is performed in-place modifying the original object.
  /// @see Shift() which does not modify this object.
  ///
  /// For details see Section 2.1.2, p. 20 of [Jain 2010].
  ///
  /// @param[in] p_PQ_E Vector from the original about point P to the new
  ///                   about point Q, expressed in the same frame E `this`
  ///                   spatial inertia is expressed in.
  /// @returns A reference to `this` spatial inertia for body or composite body
  ///          S but now computed about about a new point Q.
  SpatialInertia& ShiftInPlace(const Vector3<T>& p_PQ_E) {
    const Vector3<T> p_QScm_E = p_PScm_E_ - p_PQ_E;
    // The following two lines apply the parallel axis theorem (in place) so
    // that:
    //   G_SQ = G_SP + px_QScm² - px_PScm²
    G_SP_E_.ShiftFromCenterOfMassInPlace(p_QScm_E);
    G_SP_E_.ShiftToCenterOfMassInPlace(p_PScm_E_);
    p_PScm_E_ = p_QScm_E;
    // This would only mean a bug in the implementation. The Shift operation
    // should always lead to a valid spatial inertia.
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  /// Given `this` spatial inertia `M_SP_E` for some body or composite body S,
  /// computed about point P, and expressed in frame E, this method uses
  /// the _Parallel Axis Theorem_ for spatial inertias to compute the same
  /// spatial inertia about a new point Q. The result still is expressed in
  /// frame E.
  /// @see ShiftInPlace() for more details.
  ///
  /// @param[in] p_PQ_E Vector from the original about point P to the new
  ///                   about point Q, expressed in the same frame E `this`
  ///                   spatial inertia is expressed in.
  /// @retval M_SQ_E    This same spatial inertia for body or composite body S
  ///                   but computed about about a new point Q.
  SpatialInertia Shift(const Vector3<T>& p_PQ_E) const {
    return SpatialInertia(*this).ShiftInPlace(p_PQ_E);
  }

  /// Multiplies `this` spatial inertia `M_Bo_E` of a body B about its frame
  /// origin `Bo` by the spatial acceleration of the body frame B in a frame W.
  /// Mathematically: <pre>
  ///   F_Bo_E = M_Bo_E * A_WB_E
  /// </pre>
  /// or, in terms of its rotational and translational components (see this
  /// class's documentation for the block form of a rotational inertia): <pre>
  ///   t_Bo = I_Bo * alpha_WB + m * p_BoBcm x a_WBo
  ///   f_Bo = -m * p_BoBcm x alpha_WB + m * a_WBo
  /// </pre>
  /// where `alpha_WB` and `a_WBo` are the rotational and translational
  /// components of the spatial acceleration `A_WB`, respectively.
  ///
  /// @note
  /// The term `F_Bo_E` computed by this operator appears in the equations of
  /// motion for a rigid body which, when written about the origin `Bo` of the
  /// body frame B (which does not necessarily need to coincide with the body's
  /// center of mass), read as: <pre>
  ///   Ftot_BBo = M_Bo_W * A_WB + b_Bo
  /// </pre>
  /// where `Ftot_BBo` is the total spatial force applied on body B at at `Bo`
  /// that corresponds to the body spatial acceleration `A_WB` and `b_Bo`
  /// contains the velocity dependent gyroscopic terms (see Eq. 2.26, p. 27,
  /// in A. Jain's book).
  SpatialForce<T> operator*(const SpatialAcceleration<T>& A_WB_E) const {
    const Vector3<T>& alpha_WB_E = A_WB_E.rotational();
    const Vector3<T>& a_WBo_E = A_WB_E.translational();
    const Vector3<T>& mp_BoBcm_E = CalcComMoment();  // = m * p_BoBcm
    // Return (see class's documentation):
    // ⌈ tau_Bo_E ⌉   ⌈    I_Bo_E     | m * p_BoBcm× ⌉   ⌈ alpha_WB_E ⌉
    // |          | = |               |              | * |            |
    // ⌊  f_Bo_E  ⌋   ⌊ -m * p_BoBcm× |   m * Id     ⌋   ⌊  a_WBo_E   ⌋
    return SpatialForce<T>(
        /* rotational */
        CalcRotationalInertia() * alpha_WB_E + mp_BoBcm_E.cross(a_WBo_E),
        /* translational: notice the order of the cross product is the reversed
         * of the documentation above and thus no minus sign is needed. */
        alpha_WB_E.cross(mp_BoBcm_E) + get_mass() * a_WBo_E);
  }

  /// Multiplies `this` spatial inertia `M_BP_E` of a body B about a point P
  /// by the spatial velocity `V_WBp`, in a frame W, of the body frame B shifted
  /// to point P. Mathematically: <pre>
  ///   L_WBp_E = M_BP_E * V_WBp_E
  /// </pre>
  /// or, in terms of its rotational and translational components (see this
  /// class's documentation for the block form of a rotational inertia): <pre>
  ///   h_WB  = I_Bp * w_WB + m * p_BoBcm x v_WP
  ///   l_WBp = -m * p_BoBcm x w_WB + m * v_WP
  /// </pre>
  /// where `w_WB` and `v_WP` are the rotational and translational components of
  /// the spatial velocity `V_WBp`, respectively and, `h_WB` and `l_WBp` are the
  /// angular and linear components of the spatial momentum `L_WBp`,
  /// respectively.
  ///
  /// @note
  /// It is possible to show that `M_BP_E.Shift(p_PQ_E) * V_WBp_E.Shift(p_PQ_E)`
  /// exactly equals `L_WBp_E.Shift(p_PQ_E)`.
  SpatialMomentum<T> operator*(const SpatialVelocity<T>& V_WBp_E) const {
    const Vector3<T>& w_WB_E = V_WBp_E.rotational();
    const Vector3<T>& v_WP_E = V_WBp_E.translational();
    const Vector3<T>& mp_BoBcm_E = CalcComMoment();  // = m * p_BoBcm
    // Return (see class's documentation):
    // ⌈ h_WB  ⌉   ⌈     I_Bp      | m * p_BoBcm× ⌉   ⌈ w_WB ⌉
    // |       | = |               |              | * |      |
    // ⌊ l_WBp ⌋   ⌊ -m * p_BoBcm× |   m * Id     ⌋   ⌊ v_WP ⌋
    return SpatialMomentum<T>(
        /* rotational */
        CalcRotationalInertia() * w_WB_E + mp_BoBcm_E.cross(v_WP_E),
        /* translational: notice the order of the cross product is the reversed
         * of the documentation above and thus no minus sign is needed. */
        w_WB_E.cross(mp_BoBcm_E) + get_mass() * v_WP_E);
  }

  /// Multiplies `this` spatial inertia by a set of spatial vectors in M⁶ stored
  /// as columns of input matrix `Mmatrix`. The top three rows of Mmatrix are
  /// expected to store the rotational components while the bottom three rows
  /// are expected to store the translational components.
  /// The output matrix is of the same size as `Mmatrix` and each j-th column
  /// stores the spatial vector in F⁶ result of multiplying `this` spatial
  /// inertia with the j-th column of `Mmatrix`.
  template <typename Derived>
  Eigen::Matrix<T, 6, Derived::ColsAtCompileTime> operator*(
      const Eigen::MatrixBase<Derived>& Mmatrix) const {
    static_assert(is_eigen_scalar_same<Derived, T>::value,
                  "Derived must be templated on the same scalar type as this "
                  "spatial inertia.");
    if (Mmatrix.rows() != 6) {
      throw std::logic_error("Mmatrix must hold spatial vectors in M⁶.");
    }
    const auto& Vrotational = Mmatrix.template topRows<3>();
    const auto& Vtranslational = Mmatrix.template bottomRows<3>();
    const Vector3<T>& mp_BoBcm_E = CalcComMoment();  // = m * p_BoBcm
    const Matrix3<T> I_SP_E = CalcRotationalInertia().CopyToFullMatrix3();

    Eigen::Matrix<T, 6, Derived::ColsAtCompileTime> F_Bo_E(6, Mmatrix.cols());

    // Rotational component.
    F_Bo_E.template topRows<3>() =
        I_SP_E * Vrotational - Vtranslational.colwise().cross(mp_BoBcm_E);

    // Translational component.
    F_Bo_E.template bottomRows<3>() =
        Vrotational.colwise().cross(mp_BoBcm_E) + get_mass() * Vtranslational;
    return F_Bo_E;
  }

 private:
  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Checks that the SpatialInertia is physically valid and throws an
  // exception if not. This is mostly used in Debug builds to throw an
  // appropriate exception.
  // Since this method is used within assertions or demands, we do not try to
  // attempt a smart way throw based on a given symbolic::Formula but instead we
  // make these methods a no-op for non-numeric types.
  template <typename T1 = T>
  typename std::enable_if_t<scalar_predicate<T1>::is_bool> CheckInvariants()
      const {
    if (!IsPhysicallyValid()) {
      throw std::runtime_error(fmt::format(
          "The resulting spatial inertia:{} is not physically valid. "
          "See SpatialInertia::IsPhysicallyValid()", *this));
    }
  }

  // SFINAE for non-numeric types. See documentation in the implementation for
  // numeric types.
  template <typename T1 = T>
  typename std::enable_if_t<!scalar_predicate<T1>::is_bool> CheckInvariants()
      const {}

  // Mass of the body or composite body.
  T mass_{nan()};
  // Position vector from point P to the center of mass of body or composite
  // body S, expressed in a frame E.
  Vector3<T> p_PScm_E_{Vector3<T>::Constant(nan())};
  // Rotational inertia of body or composite body S computed about point P and
  // expressed in a frame E.
  UnitInertia<T> G_SP_E_{};  // Defaults to NaN initialized inertia.
};

/// Insertion operator to write SpatialInertia objects into a `std::ostream`.
/// Especially useful for debugging.
/// @relates SpatialInertia
template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const SpatialInertia<T>& M) {
  return o << std::endl
      << " mass = " << M.get_mass() << std::endl
      << " com = [" << M.get_com().transpose() << "]ᵀ" << std::endl
      << " I =" << std::endl
      // Like M.CalcRotationalInertia(), but without the IsPhysicallyValid
      // checks, so that we can use operator<< in error messages.
      << (M.get_mass() * M.get_unit_inertia().CopyToFullMatrix3())
      << std::endl;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::SpatialInertia)
