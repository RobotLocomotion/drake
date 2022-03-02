#pragma once

#ifndef DRAKE_SPATIAL_ALGEBRA_HEADER
#error Please include "drake/multibody/math/spatial_algebra.h", not this file.
#endif

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/convert_time_derivative.h"
#include "drake/multibody/math/spatial_vector.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {

/// This class represents a _spatial acceleration_ class and has 6 elements with
/// an angular (rotational) acceleration α (3-element vector) on top of a
/// translational (linear) acceleration 𝐚 (3-element vector). Spatial
/// acceleration represents the rotational and translational acceleration of a
/// frame B with respect to a _measured-in_ frame M. This class assumes that
/// both the angular acceleration α and translational acceleration 𝐚 are
/// expressed in the same _expressed-in_ frame E. This class only stores 6
/// elements (namely α and 𝐚) and does not store the underlying frames B, M, E.
/// The user is responsible for explicitly tracking the underlying frames with
/// @ref multibody_quantities "monogram notation". For example, A_MB_E denotes
/// frame B's spatial acceleration measured in frame M, expressed in frame E and
/// contains alpha_MB_E (B's angular acceleration measured in M, expressed in E)
/// and a_MBo_E (Bo's translational acceleration measured in M, expressed in E),
/// where Bo is frame B's origin point.
/// For an @ref multibody_frames_and_bodies "offset frame" Bp, the monogram
/// notation A_MBp_E denotes frame Bp's spatial acceleration measured in M,
/// expressed in E.  Details on spatial vectors and monogram notation are
/// in section @ref multibody_spatial_vectors.
///
/// The typeset of A_MB_E is @f$\,{^MA^B}@f$ and its definition is
/// @f$^MA^B = \frac{^Md}{dt}\,{^MV^B}\,@f$, where @f${^MV^B}@f$ is frame B's
/// spatial velocity in frame M and @f$\frac{^Md}{dt}@f$ denotes the time
/// derivative taken in frame M. To differentiate a vector, we need to
/// specify in what frame the time derivative is taken, see [Mitiguy 2016, §6.1]
/// for an in depth discussion. Time derivatives can be taken in different
/// frames and they are related by the "Transport Theorem", which in Drake is
/// implemented in drake::math::ConvertTimeDerivativeToOtherFrame().
/// In source code (monogram) notation, we write `A_MB = DtM(V_MB)`, where
/// `DtM()` denotes the time derivative in frame M. Details on vector
/// differentiation is in section @ref Dt_multibody_quantities.
///
/// [Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
///
/// @tparam_default_scalar
template <typename T>
class SpatialAcceleration : public SpatialVector<SpatialAcceleration, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialAcceleration, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialAcceleration)

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial acceleration are uninitialized (for speed). In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial acceleration fail fast (fast bug detection).
  SpatialAcceleration() : Base() {}

  /// Constructs a spatial acceleration from an angular acceleration @p `alpha`
  /// and a translational acceleration @p `a`.
  SpatialAcceleration(const Eigen::Ref<const Vector3<T>>& alpha,
                      const Eigen::Ref<const Vector3<T>>& a) : Base(alpha, a) {}

  /// Constructs a spatial acceleration from an Eigen expression that represents
  /// a 6-element vector, i.e., two 3-element vectors, namely an angular
  /// acceleration α and a translational acceleration 𝐚. This constructor will
  /// assert the size of `A` is six (6) either at compile-time for fixed sized
  /// Eigen expressions or at run-time for dynamic sized Eigen expressions.
  template <typename Derived>
  explicit SpatialAcceleration(const Eigen::MatrixBase<Derived>& A) : Base(A) {}

  /// In-place shift of a %SpatialAcceleration from a frame B to a frame C,
  /// where both B and C are fixed to the same frame or rigid body. On entry,
  /// `this` is A_MB_E (frame B's spatial acceleration measured in a frame M and
  /// expressed in a frame E). On return `this` is modified to A_MC_E (frame C's
  /// spatial acceleration measured in frame M and expressed in frame E).
  /// @param[in] offset which is the position vector p_BoCo_E from frame B's
  /// origin to frame C's origin, expressed in frame E. p_BoCo_E must have
  /// the same expressed-in frame E as `this` spatial acceleration.
  /// @param[in] omega which is ω_MB_E, frame B's angular velocity measured in
  /// frame W and expressed in frame E.
  /// @retval A_MC_E reference to `this` spatial acceleration which has been
  /// modified to be frame C's spatial acceleration measured in frame M and
  /// expressed in frame E. The components of A_MC_E are calculated as: <pre>
  ///  α_MC_E = α_MB_E           (angular acceleration of `this` is unchanged).
  ///  a_MC_E = a_MB_E + α_MB_E x p_BoCo_E + ω_MB_E x (ω_MB_E x p_BoCo_E)
  /// </pre>
  /// @see Shift() to shift spatial acceleration without modifying `this`.
  ///
  /// <h3> Derivation </h3>
  ///
  /// <h4> Rotational acceleration component </h4>
  ///
  /// Frame B and frame C are fixed (e.g., welded) to the same rigid object.
  /// Hence frames B and C always rotate together at the same rate and: <pre>
  ///   α_MC_E = α_MB_E
  /// </pre>
  ///
  /// <h4> Translational acceleration component </h4>
  ///
  /// Since frame B and frame C are regarded as fixed to the same rigid object,
  /// the translational velocity of Co (frame C's origin) measured in frame M
  /// can be calculated as: <pre>
  ///   v_MCo = v_MBo + ω_MB x p_BoCo
  /// </pre>
  /// Point Co's translational acceleration in frame M is: <pre>
  ///   a_WQ = DtM(v_MCo)                       (definition of acceleration).
  ///         = DtM(v_MBo  + ω_MB x p_BoCo)     (substitution)
  ///         = DtM(v_MBo) + DtM(ω_MB) x p_BoCo + ω_MB x DtM(p_BoCo)
  ///         =     a_MBo  +     α_MB  x p_BoCo + ω_MB x DtM(p_BoCo)
  /// </pre>
  /// The "Transport Theorem" converts the time-derivative in the last term from
  /// `DtM()` to `DtB()` -- see math::ConvertTimeDerivativeToOtherFrame() <pre>
  ///   DtM(p_BoCo) = DtB(p_BoCo) + ω_MB x p_BoCo
  ///               =          0  + ω_MB x p_BoCo
  /// </pre>
  SpatialAcceleration<T>& ShiftInPlace(const Vector3<T>& offset,
                                       const Vector3<T>& omega) {
    const Vector3<T>& p_BoCo_E = offset;
    const Vector3<T>& w_MB_E = omega;
    // Frame B's angular acceleration measured in frame M, expressed in frame M.
    const Vector3<T>& alpha_MB_E = this->rotational();
    // Calculate point Co's translational acceleration measured in M.
    Vector3<T>& a_MCo_E = this->translational();
    a_MCo_E += (alpha_MB_E.cross(p_BoCo_E)
            +   w_MB_E.cross(w_MB_E.cross(p_BoCo_E)));
    return *this;
  }

  /// Shifts a %SpatialAcceleration from a frame B to a frame C, where both
  /// B and C are fixed to the same frame or rigid body.
  /// This method differs from ShiftInPlace() in that this method does not
  /// modify `this` whereas ShiftInPlace() does modify `this`.
  /// @param[in] offset which is the position vector p_BoCo_E from frame B's
  /// origin to frame C's origin, expressed in frame E. p_BoCo_E must have the
  /// same expressed-in frame E as `this` spatial acceleration
  /// (`this` = A_MB_E).
  /// @param[in] omega which is ω_MB_E, frame B's angular velocity measured in
  /// frame W and expressed in frame E.
  /// @retval A_MC_E which is frame C's spatial acceleration measured in
  /// frame M, expressed in frame E.
  /// @see ShiftInPlace() for more information and how A_MC_E is calculated.
  SpatialAcceleration<T> Shift(const Vector3<T>& offset,
                               const Vector3<T>& omega) const {
    return SpatialAcceleration<T>(*this).ShiftInPlace(offset, omega);
  }

  /// (Advanced) Given `this` spatial acceleration `A_MB` of a frame B measured
  /// in a frame M, shifts %SpatialAcceleration from frame B to a frame C,
  /// where both B and C are fixed to the same frame or rigid body and
  /// where ω_MB = 0 (frame B's angular velocity in frame M is zero).
  /// @param[in] offset which is the position vector p_BoCo_E from frame B's
  /// origin to frame C's origin, expressed in frame E. p_BoCo_E must have the
  /// same expressed-in frame E as `this` spatial acceleration
  /// (`this` = A_MB_E).
  /// @retval A_MC_E which is frame C's spatial acceleration measured in
  /// frame M, expressed in frame E.
  /// @see ShiftInPlace() for more information and how A_MC_E is calculated.
  /// This method speed the Shift() computation when ω_MB = 0.
  SpatialAcceleration<T> Shift(const Vector3<T>& offset) const {
    const Vector3<T>& p_BoCo_E = offset;
    const Vector3<T>& alpha_MB_E = this->rotational();
    const Vector3<T>& a_MBo_E = this->translational();
    return SpatialAcceleration<T>(alpha_MB_E,
                                  a_MBo_E + alpha_MB_E.cross(p_BoCo_E));
  }

  /// Given a frame C's acceleration relative to a frame B, and frame B's
  /// acceleration measured in a frame M, returns C's acceleration in M.
  /// @param[in] position_of_moving_frame which is the position vector p_BoCo_E
  /// (from frame B's origin Bo to frame C's origin Co), expressed in frame E.
  /// p_BoCo_E must have the same expressed-in frame E as `this` = A_MB_E.
  /// @param[in] omega which is ω_MB_E, frame B's angular velocity measured in
  /// frame W and expressed in frame E.
  /// @param[in] velocity_of_moving_frame which is V_BC_E, frame C's spatial
  /// velocity measured in frame B, expressed in the same frame E as
  /// `this` = A_MB_E.
  /// @param[in] acceleration_of_moving_frame which is A_BC_E, frame C's
  /// spatial acceleration measured in frame B, expressed in the same frame E
  /// as `this` = A_MB_E.
  /// @retval A_MC_E frame C's spatial acceleration measured in frame M,
  /// expressed in frame E.
  /// @note The returned spatial acceleration A_MC_E contains an angular
  /// acceleration α_MC_E and translational acceleration a_MCo_E that are
  /// calculated as: <pre>
  ///  α_MC_E  = α_MB_E + α_BC_E + ω_MB_E + ω_BC_E
  ///  a_MCo_E = a_MBo_E + α_MB_E x p_BoCo_E + a_BCo_E
  /// </pre>
  /// If frame C is rigidly fixed to frame B,  A_BC_E = 0 and V_BC_E = 0 and
  /// this method produces a Shift() operation (albeit inefficiently).
  /// @see %SpatialVelocity::ComposeWithMovingFrameVelocity() for the related
  /// %SpatialVelocity method and calculations.
  ///
  /// This operation can be written in a more compact form in terms of the
  /// rigid shift operator `Φᵀ(p_PB)` (see SpatialVelocity::Shift()) as: <pre>
  ///   A_WB = Φᵀ(p_PB) A_WP + Ac_WB(w_WP, V_PB) + A_PB_W
  /// </pre>
  /// where `Φᵀ(p_PB) A_WP` denotes the application of the rigid shift
  /// operation as in SpatialVelocity::Shift() and `Ac_WB(w_WP, V_PB)` contains
  /// the centrifugal and Coriolis terms: <pre>
  ///   Ac_WB(w_WP, V_PB) = | w_WP x w_PB_W                          |
  ///                       | w_WP x w_WP x p_PB_W + 2 w_WP x v_PB_W |
  ///                                   ^^^                ^^^
  ///                               centrifugal         Coriolis
  /// </pre>
  /// The equation above shows that composing spatial accelerations cannot be
  /// simply accomplished by adding `A_WP` with `A_PB`. Moreover, we see that,
  /// unlike with angular velocities, angular accelerations cannot be added in
  /// order to compose them. That is `w_AC = w_AB + w_BC` but `alpha_AC ≠
  /// alpha_AB + alpha_BC` due to the cross term `w_AC x w_BC`. See the
  /// derivation below for more details.
  ///
  /// <h3> Derivation </h3>
  /// The spatial velocity of frame B in W can be obtained by composing `V_WP`
  /// with `V_PB`: <pre>
  ///   V_WB = V_WPb + V_PB = Φᵀ(p_PB) V_WP + V_PB                        (1)
  /// </pre>
  /// This operation can be performed with the method
  /// SpatialVelocity::ComposeWithMovingFrameVelocity().
  ///
  /// <h4> Translational acceleration component </h4>
  ///
  /// The translational velocity `v_WB` of point B in W corresponds to the
  /// translational component in Eq. (1): <pre>
  ///   v_WB = v_WP + w_WP x p_PB + v_PB                                  (2)
  /// </pre>
  /// Therefore, for the translational acceleration we have: <pre>
  ///   a_WB = DtW(v_WB)
  ///         = DtW(v_WP + w_WP x p_PB + v_PB)
  ///         = DtW(v_WP) + DtW(w_WP x p_PB) + DtW(v_PB)
  ///         = a_WP + DtW(w_WP) x p_PB + w_WP x DtW(p_PB) + DtW(v_PB)
  ///         = a_WP + alpha_WP x p_PB + w_WP x DtW(p_PB) + DtW(v_PB)     (3)
  /// </pre>
  /// with `a_WP = DtW(v_WP)` and `alpha_WP = DtW(w_WP)` by definition.
  /// The term DtW(p_PB) in Eq. (3) is obtained by converting the vector time
  /// derivative from `DtW()` to `DtP()`,
  /// see drake::math::ConvertTimeDerivativeToOtherFrame(): <pre>
  ///   DtW(p_PB) = DtP(p_PB) + w_WP x p_PB
  ///             = v_PB + w_WP x p_PB                                    (4)
  /// </pre>
  /// since `v_PB = DtP(p_PB)` by definition.
  /// Similarly, the term `DtW(v_PB)` in Eq. (3) is also obtained by converting
  /// the time derivative from `DtW()` to `DtP()`: <pre>
  ///   DtW(v_PB) = DtP(v_PB) + w_WP x v_PB
  ///             = a_PB + w_WP x v_PB                                    (5)
  /// </pre>
  /// with `a_PB = DtP(v_PB)` by definition.
  /// Using Eqs. (4) and (5) in Eq. (3) yields for the translational
  /// acceleration: <pre>
  ///   a_WB = a_WP + alpha_WP x p_PB + a_PB + ac_WB
  ///   ac_WB = w_WP x (v_PB + w_WP x p_PB) + w_WP x v_PB                 (6)
  /// </pre>
  /// where finally the term `ac_WB` can be written as: <pre>
  ///   ac_WB = w_WP x w_WP x p_PB + 2 * w_WP x v_PB                      (7)
  /// </pre>
  /// which includes the effect of angular acceleration of P in W
  /// `alpha_WP x p_PB`, the centrifugal acceleration `w_WP x w_WP x p_PB`,
  /// the Coriolis acceleration `2 * w_WP x v_PB` due to the motion of B in
  /// P and, the additional acceleration of B in P `a_PB`.
  ///
  /// @note Alternatively, we can write an efficient version of the
  /// centrifugal term `ac_WB` in Eq. (6) for when the velocities of P and B are
  /// available (e.g. from velocity kinematics). This is accomplished by adding
  /// and subtracting v_WP within the parenthesized term in Eq. (6) and grouping
  /// together v_WB = v_WPb + v_PB = v_WP + w_WP x p_PB + v_PB: <pre>
  ///   ac_WB = w_WP x (v_WB - v_WP) + w_WP x v_PB
  ///         = w_WP x (v_WB - v_WP + v_PB)                               (6b)
  /// </pre>
  /// which simplifies the expression from three cross products to one.
  ///
  /// <h4> Rotational acceleration component </h4>
  ///
  /// The rotational velocity `w_WB` of frame B in W corresponds to the
  /// rotational component in Eq. (1): <pre>
  ///   w_WB = w_WP + w_PB                                                (8)
  /// </pre>
  /// Therefore, the rotational acceleration of B in W corresponds to: <pre>
  ///   alpha_WB = DtW(w_WB) = DtW(w_WP) + DtW(w_PB)
  ///            = alpha_WP + DtW(w_PB)                                   (9)
  /// </pre>
  /// where the last term in Eq. (9) can be converted to a time derivative in P
  /// as: <pre>
  ///   DtW(w_PB) = DtP(w_PB) + w_WP x w_PB = alpha_PB + w_WP x w_PB      (10)
  /// </pre>
  /// where `alpha_PB = DtP(w_PB)` by definition.
  /// Thus, the final expression for `alpha_WB` is obtained by using Eq. (10)
  /// into Eq. (9): <pre>
  ///   alpha_WB = alpha_WP + alpha_PB + w_WP x w_PB                      (11)
  /// </pre>
  /// Equation (11) shows that angular accelerations cannot be simply added as
  /// angular velocities can but there exists an additional term `w_WP x w_PB`.
  ///
  /// <h4> The spatial acceleration </h4>
  ///
  /// The rotational and translational components of the spatial acceleration
  /// are given by Eqs. (11) and (6) respectively: <pre>
  ///   A_WB.rotational() = alpha_WB
  ///                     = {alpha_WP} + alpha_PB + w_WP x w_PB           (12)
  ///   A_WB.translational() = a_WB
  ///                        = {a_WP + alpha_WP x p_PB + w_WP x w_WP x p_PB}
  ///                        + 2 * w_WP x v_PB + a_PB                     (13)
  /// </pre>
  /// where we have placed within curly brackets `{}` all the terms that also
  /// appear in the Shift() operation, which is equivalent to this method when
  /// `V_PB` and `A_PB` are both zero.
  /// In the equations above `alpha_WP = A_WP.rotational()` and
  /// `a_WP = A_WP.translational()`.
  /// The above expression can be written in a more compact form in terms of the
  /// rigid shift operator `Φᵀ(p_PB)` (see SpatialVelocity::Shift()) as
  /// presented in the main body of this documentation: <pre>
  ///   A_WB = Φᵀ(p_PB)A_WP + Ac_WB(w_WP, V_PB) + A_PB_W                  (14)
  /// </pre>
  /// where `Ac_WB(w_WP, V_PB)` contains the centrifugal and Coriolis terms:
  /// <pre>
  ///   Ac_WB(w_WP, V_PB) = | w_WP x w_PB_W                          |
  ///                       | w_WP x w_WP x p_PB_W + 2 w_WP x v_PB_W |
  ///                                   ^^^                ^^^
  ///                               centrifugal         Coriolis
  /// </pre>
  SpatialAcceleration<T> ComposeWithMovingFrameAcceleration(
      const Vector3<T>& p_PB_E,
      const Vector3<T>& w_WP_E,
      const SpatialVelocity<T>& V_PB_E,
      const SpatialAcceleration<T>& A_PB_E) const {
    const Vector3<T>& w_PB_E = V_PB_E.rotational();
    const Vector3<T>& v_PB_E = V_PB_E.translational();

    // Compute all the terms within curly brackets in the derivation above which
    // correspond to the Shift() operation:
    SpatialAcceleration<T> A_WB_E = this->Shift(p_PB_E, w_WP_E);
    // Adds non-linear coupling of angular velocities:
    A_WB_E.rotational() += (A_PB_E.rotational() + w_WP_E.cross(w_PB_E));

    // Adds Coriolis and translational acceleration of B in P.
    A_WB_E.translational() +=
        (A_PB_E.translational() + 2.0 * w_WP_E.cross(v_PB_E));
    return A_WB_E;
  }
};

/// (Advanced) Addition operator. Implements the addition of A1 and A2 as
/// elements in ℝ⁶.
/// @warning Composing spatial accelerations A1 and A2 cannot be simply
/// accomplished by adding them with this operator. The composition of
/// accelerations involves additional centrifugal and Coriolis terms, see
/// SpatialAcceleration::ComposeWithMovingFrameAcceleration() for details.
/// @relates SpatialAcceleration
template <typename T>
inline SpatialAcceleration<T> operator+(const SpatialAcceleration<T>& A1,
                                        const SpatialAcceleration<T>& A2) {
  // N.B. We use SpatialVector's implementation, though we provide the overload
  // for specific documentation purposes.
  return SpatialAcceleration<T>(A1) += A2;
}

/// (Advanced) Subtraction operator. Implements the subtraction of A1 and A2 as
/// elements in ℝ⁶.
/// @warning With `As = A1 - A2`, `A1 = As + A2` does not correspond to the
/// physical composition of spatial accelerations As and A2. Please refere to
/// operator+(const SpatialAcceleration<T>&, const SpatialAcceleration<T>&) for
/// details.
/// @relates SpatialAcceleration
template <typename T>
inline SpatialAcceleration<T> operator-(const SpatialAcceleration<T>& A1,
                                        const SpatialAcceleration<T>& A2) {
  // N.B. We use SpatialVector's implementation, though we provide the overload
  // for specific documentation purposes.
  return SpatialAcceleration<T>(A1) -= A2;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialAcceleration)
