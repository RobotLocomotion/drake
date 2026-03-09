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

/// This class represents a _spatial acceleration_ A and has 6 elements with
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
/// expressed in E. Details on spatial vectors and monogram notation are in
/// sections @ref multibody_spatial_vectors and @ref multibody_quantities.
///
/// The typeset for A_MB is @f$\,{^MA^B}@f$ and its definition is
/// @f$^MA^B = \frac{^Md}{dt}\,{^MV^B}\,@f$, where @f${^MV^B}@f$ is frame B's
/// spatial velocity in frame M and @f$\frac{^Md}{dt}@f$ denotes the time
/// derivative taken in frame M. To differentiate a vector, we need to
/// specify in what frame the time derivative is taken, see [Mitiguy 2022, §7.2]
/// for an in-depth discussion. Time derivatives in different frames are related
/// by the "Transport Theorem", which in Drake is implemented in
/// drake::math::ConvertTimeDerivativeToOtherFrame().
/// In source code (monogram) notation, we write A_MB = DtM(V_MB), where
/// DtM() denotes the time derivative in frame M. Details on vector
/// differentiation is in section @ref Dt_multibody_quantities.
///
/// [Mitiguy 2022] Mitiguy, P., 2022. Advanced Dynamics & Motion Simulation.
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialAcceleration);

  /// Default constructor. In Release builds, all 6 elements of a newly
  /// constructed spatial acceleration are uninitialized (for speed). In Debug
  /// builds, the 6 elements are set to NaN so that invalid operations on an
  /// uninitialized spatial acceleration fail fast (fast bug detection).
  SpatialAcceleration() : Base() {}

  /// Constructs a spatial acceleration A from an angular acceleration α (alpha)
  /// and a translational acceleration 𝐚.
  SpatialAcceleration(const Eigen::Ref<const Vector3<T>>& alpha,
                      const Eigen::Ref<const Vector3<T>>& a)
      : Base(alpha, a) {}

  /// Constructs a spatial acceleration A from an Eigen expression that
  /// represents a 6-element vector, i.e., a 3-element angular acceleration α
  /// and a 3-element translational acceleration 𝐚. This constructor will assert
  /// the size of A is six (6) either at compile-time for fixed sized Eigen
  /// expressions or at run-time for dynamic sized Eigen expressions.
  template <typename Derived>
  explicit SpatialAcceleration(const Eigen::MatrixBase<Derived>& A) : Base(A) {}

  /// In-place shift of a %SpatialAcceleration from a frame B to a frame C,
  /// where both B and C are fixed to the same frame or rigid body. On entry,
  /// `this` is A_MB_E (frame B's spatial acceleration measured in a frame M and
  /// expressed in a frame E). On return `this` is modified to A_MC_E (frame C's
  /// spatial acceleration measured in frame M and expressed in frame E).
  /// The components of A_MC_E are calculated as: <pre>
  ///   α_MC_E = α_MB_E           (angular acceleration of `this` is unchanged).
  ///  a_MCo_E = a_MBo_E + α_MB_E x p_BoCo_E + ω_MB_E x (ω_MB_E x p_BoCo_E)
  /// </pre>
  /// @param[in] offset which is the position vector p_BoCo_E from Bo (frame B's
  /// origin) to Co (frame C's origin), expressed in frame E. p_BoCo_E must have
  /// the same expressed-in frame E as `this` spatial acceleration.
  /// @param[in] angular_velocity_of_this_frame which is ω_MB_E, frame B's
  /// angular velocity measured in frame W and expressed in frame E.
  /// @see Shift() to shift spatial acceleration without modifying `this`.
  /// Use ComposeWithMovingFrameAcceleration() if frame C is moving on frame B.
  ///
  /// <h3> Derivation </h3>
  ///
  /// <h4> Rotational acceleration component </h4>
  ///
  /// Frame B and frame C are fixed (e.g., welded) to the same rigid object.
  /// Hence frames B and C always rotate together at the same rate and: <pre>
  ///   ω_MC_E = ω_MB_E
  ///   α_MC_E = α_MB_E
  /// </pre>
  ///
  /// <h4> Translational acceleration component </h4>
  ///
  /// Since frames B and C are fixed to the same rigid object, the translational
  /// velocity of Co (frame C's origin) measured in frame M can be calculated as
  /// <pre>
  ///   v_MCo = v_MBo + ω_MB x p_BoCo
  /// </pre>
  /// Point Co's translational acceleration in frame M is: <pre>
  ///   a_MCo = DtM(v_MCo)                      (definition of acceleration).
  ///         = DtM(v_MBo  + ω_MB x p_BoCo)     (substitution)
  ///         = DtM(v_MBo) + DtM(ω_MB) x p_BoCo + ω_MB x DtM(p_BoCo)
  ///         =     a_MBo  +     α_MB  x p_BoCo + ω_MB x DtM(p_BoCo)
  /// </pre>
  /// The "Transport Theorem" converts the time-derivative of the last term from
  /// DtM() to DtB() -- see math::ConvertTimeDerivativeToOtherFrame(), as <pre>
  ///   DtM(p_BoCo) = DtB(p_BoCo) + ω_MB x p_BoCo
  ///               =          0  + ω_MB x p_BoCo
  /// </pre>
  void ShiftInPlace(const Vector3<T>& offset,
                    const Vector3<T>& angular_velocity_of_this_frame) {
    const Vector3<T>& p_BoCo_E = offset;
    const Vector3<T>& w_MB_E = angular_velocity_of_this_frame;
    // Frame B's angular acceleration measured in frame M, expressed in frame M.
    const Vector3<T>& alpha_MB_E = this->rotational();
    // Calculate point Co's translational acceleration measured in M.
    Vector3<T>& a_MCo_E = this->translational();
    a_MCo_E += (alpha_MB_E.cross(p_BoCo_E) +
                w_MB_E.cross(w_MB_E.cross(p_BoCo_E)));  // 33 flops
  }

  /// Shifts a %SpatialAcceleration from a frame B to a frame C, where both
  /// B and C are fixed to the same frame or rigid body.
  /// @param[in] offset which is the position vector p_BoCo_E from Bo (frame B's
  /// origin) to Co (frame C's origin), expressed in frame E. p_BoCo_E must have
  /// the same expressed-in frame E as `this` spatial acceleration, where `this`
  /// is A_MB_E (frame B's spatial acceleration measured in M, expressed in E).
  /// @param[in] angular_velocity_of_this_frame which is ω_MB_E, frame B's
  /// angular velocity measured in frame M and expressed in frame E.
  /// @retval A_MC_E which is frame C's spatial acceleration measured in
  /// frame M, expressed in frame E.
  /// @note Shift() differs from ShiftInPlace() in that Shift() does not modify
  /// `this` whereas ShiftInPlace() does modify `this`.
  /// @see ShiftInPlace() for more information and how A_MC_E is calculated.
  /// Use ComposeWithMovingFrameAcceleration() if frame C is moving on frame B.
  SpatialAcceleration<T> Shift(
      const Vector3<T>& offset,
      const Vector3<T>& angular_velocity_of_this_frame) const {
    SpatialAcceleration<T> result(*this);
    result.ShiftInPlace(offset, angular_velocity_of_this_frame);
    return result;
  }

  /// (Advanced) Given `this` spatial acceleration A_MB of a frame B measured
  /// in a frame M, shifts %SpatialAcceleration from frame B to a frame C (i.e.,
  /// A_MB to A_MC), where both B and C are fixed to the same frame or rigid
  /// body and where ω_MB = 0 (frame B's angular velocity in frame M is zero).
  /// @param[in] offset which is the position vector p_BoCo_E from Bo (frame B's
  /// origin) to Co (frame C's origin), expressed in frame E. p_BoCo_E must have
  /// the same expressed-in frame E as `this` spatial acceleration, where `this`
  /// is A_MB_E (frame B's spatial acceleration measured in M, expressed in E).
  /// @retval A_MC_E which is frame C's spatial acceleration measured in
  /// frame M, expressed in frame E.
  /// @see ShiftInPlace() for more information and how A_MC_E is calculated.
  /// @note ShiftWithZeroAngularVelocity() speeds the Shift() computation when
  /// ω_MB = 0, even if α_MB ≠ 0 (α_MB is stored in `this`).
  SpatialAcceleration<T> ShiftWithZeroAngularVelocity(
      const Vector3<T>& offset) const {
    const Vector3<T>& p_BoCo_E = offset;
    const Vector3<T>& alpha_MB_E = this->rotational();
    const Vector3<T>& a_MBo_E = this->translational();
    return SpatialAcceleration<T>(alpha_MB_E,
                                  a_MBo_E + alpha_MB_E.cross(p_BoCo_E));
  }

  /// Compose `this` spatial acceleration (measured in some frame M) with the
  /// spatial acceleration of another frame to form the 𝐨𝐭𝐡𝐞𝐫 frame's spatial
  /// acceleration in frame M. Herein, `this` is the spatial acceleration of a
  /// frame (designated B) in frame M and the 𝐨𝐭𝐡𝐞𝐫 frame is designated C.
  /// @param[in] position_of_moving_frame which is the position vector p_BoCo_E
  /// (from frame B's origin Bo to frame C's origin Co), expressed in frame E.
  /// p_BoCo_E must have the same expressed-in frame E as `this`, where `this`
  /// is A_MB_E (frame B's spatial acceleration measured in M, expressed in E).
  /// @param[in] angular_velocity_of_this_frame which is ω_MB_E, frame B's
  /// angular velocity measured in frame W and expressed in frame E.
  /// @param[in] velocity_of_moving_frame which is V_BC_E, frame C's spatial
  /// velocity measured in frame B, expressed in frame E.
  /// @param[in] acceleration_of_moving_frame which is A_BC_E, frame C's
  /// spatial acceleration measured in frame B, expressed in frame E.
  /// @retval A_MC_E frame C's spatial acceleration measured in frame M,
  /// expressed in frame E.
  /// @see SpatialVelocity::ComposeWithMovingFrameVelocity().
  /// Use Shift() if frames B and C are both fixed to the same frame or body,
  /// i.e., velocity_of_moving_frame = 0 and acceleration_of_moving_frame = 0.
  /// @note The returned spatial acceleration A_MC_E contains an angular
  /// acceleration α_MC_E and translational acceleration a_MCo_E that are
  /// calculated as: <pre>
  ///  α_MC_E  = α_MB_E + α_BC_E + ω_MB_E x ω_BC_E
  ///  a_MCo_E = a_BCo_E + α_MB_E x p_BoCo_E + ω_MB_E x (ω_MB_E x p_BoCo_E)
  ///          + 2 ω_MB_E x v_BCo_E + a_BCo_E
  /// </pre>
  /// If frame C is rigidly fixed to frame B, A_BC_E = 0 and V_BC_E = 0 and
  /// this method produces a Shift() operation (albeit inefficiently).
  /// The previous equations show composing spatial acceleration is not simply
  /// adding A_MB + A_BC and these equations differ significantly from their
  /// spatial velocity counterparts. For example, angular velocities simply add
  /// as <pre>
  ///   ω_MC = ω_MB + ω_BC,   but 3D angular acceleration is more complicated as
  ///   α_MC = α_MB + α_BC + ω_MB x ω_BC
  /// </pre>
  ///
  /// <h3> Derivation </h3>
  ///
  /// <h4> Rotational acceleration component </h4>
  ///
  /// ω_MC (frame C's angular velocity in frame M) can be calculated with the
  /// angular velocity addition theorem as <pre>
  ///   ω_MC = ω_MB + ω_BC
  /// </pre>
  /// α_MC (frame C's angular acceleration measured in frame M) is defined as
  /// the time-derivative in frame M of ω_MC, and can be calculated using the
  /// "Transport Theorem" (Golden rule for vector differentation) which converts
  /// the time-derivative of a vector in frame M to frame B, e.g., as
  /// DtM(ω_BC) = DtB(ω_BC) + ω_MB x ω_BC, as <pre>
  ///   α_MC = DtM(ω_MC) = DtM(ω_MB) + DtM(ω_BC)
  ///                    =     α_MB  + DtB(ω_BC) + ω_MB x ω_BC
  ///                    =     α_MB  +     α_BC  + ω_MB x ω_BC   (End of proof).
  /// </pre>
  ///
  /// <h4> Translational acceleration component </h4>
  ///
  /// v_MCo (frame C's translational velocity in frame M) is calculated in
  /// SpatialVelocity::ComposeWithMovingFrameVelocity) as <pre>
  ///   v_MCo = v_MBo + ω_MB x p_BoCo + v_BCo
  /// </pre>
  /// a_MCo (frame C's translational acceleration measured in frame M) is
  /// defined as the time-derivative in frame M of v_MCo, calculated as <pre>
  ///  a_MCo = DtM(v_MCo)                             Definition.
  ///        = DtM(v_MBo + ω_MB x p_BoCo + v_BCo)     Substitution.
  ///        = DtM(v_MBo) + DtM(ω_MB) x p_BoCo + ω_MB x DtM(p_BoCo) + DtM(v_BCo)
  ///        =     a_MBo  +     α_MB  x p_BoCo + ω_MB x DtM(p_BoCo) + DtM(v_BCo)
  /// </pre>
  /// The last two terms are modified using the "Transport Theorem" (Golden rule
  /// for vector differentation) which converts time-derivatives of vectors in
  /// frame M to frame B via DtM(vec) = DtB(vec) + ω_MB x vec. <pre>
  ///  DtM(p_BoCo) = DtB(p_BoCo) + ω_MB x p_BoCo
  ///              =     v_BCo   + ω_MB x p_BoCo
  ///  DtM(v_BCo)  = DtB(v_BCo)  + ω_MB x v_BCo
  ///              =     a_BCo   + ω_MB x v_BCo
  /// </pre>
  /// Combining the last few equations proves the formula for a_MCo as: <pre>
  ///   a_MCo = a_MBo + α_MB x p_BoCo + ω_MB x (ω_MB x p_BoCo)
  ///         + 2 ω_MB x v_BCo + a_BCo                           (End of proof).
  /// </pre>
  /// Some terms in the previous equation have names, e.g., <pre>
  ///   centripetal acceleration   ω_MB x (ω_MB x p_BoCo)
  ///   Coriolis acceleration    2 ω_MB x v_BCo
  ///   Coincident point acceleration, i.e., acceleration of the point of frame
  ///   B coincident with Co      a_MBo + α_MB x p_BoCo + ω_MB x (ω_MB x p_BoCo)
  /// </pre>
  /// Note: The coincident point acceleration can be calculated with a Shift().
  ///
  /// Note: The three cross products appearing in the previous calculation of
  /// a_MCo can be reduced to one, possibly improving efficiency via <pre>
  ///   ω_MB x (ω_MB x p_BoCo) + 2 ω_MB x v_BCo = ω_MB x (v_MCo - v_MBo + v_BCo)
  /// </pre>
  /// To show this, we rearrange and substitute our expression for v_MCo. <pre>
  ///           v_MCo = v_MBo + ω_MB x p_BoCo + v_BCo        which rearranges to
  ///   ω_MB x p_BoCo = v_MCo - v_MBo - v_BCo.             Substitution produces
  ///   ω_MB x (ω_MB x p_BoCo) = ω_MB x (v_MCo - v_MBo - v_BCo)           Hence,
  ///   ω_MB x (ω_MB x p_BoCo) + 2 ω_MB x v_BCo = ω_MB x (v_MCo - v_MBo + v_BCo)
  /// </pre>
  SpatialAcceleration<T> ComposeWithMovingFrameAcceleration(
      const Vector3<T>& position_of_moving_frame,
      const Vector3<T>& angular_velocity_of_this_frame,
      const SpatialVelocity<T>& velocity_of_moving_frame,
      const SpatialAcceleration<T>& acceleration_of_moving_frame) const {
    // This operation can be written in a compact form using the rigid shift
    // operator Φᵀ(p_BoCo) (documented in SpatialVelocity::Shift()) and
    //   Ac_MC(ω_MB, V_BC) which contains the centrifugal and Coriolis terms:
    //   A_MC = Φᵀ(p_BoCo) A_MB + Ac_MC(ω_MB, V_BC) + A_BC
    //   Ac_MC(ω_MB, V_BC) = | ω_MB x ω_BC                             |
    //                       | ω_MB x (ω_MB x p_BoCo) + 2 ω_MB x v_BCo |
    //                               ^^^^^^^^^^^          ^^^^^^^^
    //                               centrifugal          Coriolis
    const Vector3<T>& p_PB_E = position_of_moving_frame;
    const Vector3<T>& w_WP_E = angular_velocity_of_this_frame;
    const SpatialVelocity<T>& V_PB_E = velocity_of_moving_frame;
    const SpatialAcceleration<T>& A_PB_E = acceleration_of_moving_frame;
    const Vector3<T>& w_PB_E = V_PB_E.rotational();
    const Vector3<T>& v_PB_E = V_PB_E.translational();

    // Use Shift() to calculate the coincident point acceleration, i.e.,
    // acceleration of the point of frame B coincident with Co as
    // a_MBo + α_MB x p_BoCo + ω_MB x (ω_MB x p_BoCo).
    SpatialAcceleration<T> A_WB_E = this->Shift(p_PB_E, w_WP_E);  // 33 flops
    // Adds additional term in angular acceleration calculation, i.e.,
    // α_MC = α_MB + α_BC + ω_MB x ω_BC.
    const Vector3<T>& alpha_PB_E = A_PB_E.rotational();
    A_WB_E.rotational() += (alpha_PB_E + w_WP_E.cross(w_PB_E));  // 15 flops

    // Adds Coriolis and translational acceleration of B in P.
    // a_MCo = ...  a_BCo + 2 ω_MB x v_BCo
    const Vector3<T>& a_PB_E = A_PB_E.translational();
    A_WB_E.translational() += (a_PB_E + 2.0 * w_WP_E.cross(v_PB_E));  // 18
    return A_WB_E;  // Total: 66 flops
  }
};

/// Adds two spatial accelerations by simply adding their 6 underlying elements.
/// @param[in] A1_E spatial acceleration expressed in the same frame E as A2_E.
/// @param[in] A2_E spatial acceleration expressed in the same frame E as A1_E.
/// @note The general utility of this operator+() function is questionable and
/// it should only be used if you are sure it makes sense.
/// @see Shift(), ShiftInPlace(), and ComposeWithMovingFrameAcceleration().
/// @relates SpatialAcceleration
template <typename T>
inline SpatialAcceleration<T> operator+(const SpatialAcceleration<T>& A1_E,
                                        const SpatialAcceleration<T>& A2_E) {
  // Although this implementation calls the base class operator, it is needed
  // for documentation.
  return SpatialVector<SpatialAcceleration, T>::operator+(A1_E, A2_E);
}

/// Subtracts spatial accelerations by simply subtracting their 6 underlying
/// elements.
/// @param[in] A1_E spatial acceleration expressed in the same frame E as A2_E.
/// @param[in] A2_E spatial acceleration expressed in the same frame E as A1_E.
/// @note The general utility of this operator-() function is questionable and
/// it should only be used if you are sure it makes sense.
/// @see Shift(), ShiftInPlace(), and ComposeWithMovingFrameAcceleration().
/// @relates SpatialAcceleration
template <typename T>
inline SpatialAcceleration<T> operator-(const SpatialAcceleration<T>& A1_E,
                                        const SpatialAcceleration<T>& A2_E) {
  // Although this implementation calls the base class operator, it is needed
  // for documentation.
  return SpatialVector<SpatialAcceleration, T>::operator-(A1_E, A2_E);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialAcceleration);
