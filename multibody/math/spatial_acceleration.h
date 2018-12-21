#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/convert_time_derivative.h"
#include "drake/multibody/math/spatial_vector.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {

/// This class is used to represent a _spatial acceleration_ that combines
/// rotational (angular acceleration) and translational (linear acceleration)
/// components.
/// While a SpatialVelocity `V_XY` represents the motion of a "moving frame"
/// Y measured with respect to a "measured-in" frame X, the %SpatialAcceleration
/// `A_XY` represents the rate of change of this spatial velocity `V_XY` in
/// frame X. That is @f$^XA^Y = \frac{^Xd}{dt}\,{^XV^Y} @f$ where
/// @f$\frac{^Xd}{dt} @f$ denotes the time derivative taken in frame X. That is,
/// to compute an acceleration we need to specify in what frame the time
/// derivative is taken, see [Mitiguy 2016, §6.1] for a more in depth discussion
/// on this. Time derivatives can be taken in different frames, and they
/// transform according to the "Transport Theorem", which is in Drake is
/// implemented in drake::math::ConvertTimeDerivativeToOtherFrame().
/// In source code comments we write `A_XY = DtX(V_XY)`, where `DtX()` is the
/// operator that takes the time derivative in the X frame.
/// By convention, and unless otherwise stated, we assume that the frame in
/// which the time derivative is taken is the "measured-in" frame, i.e. the time
/// derivative used in `A_XY` is in frame X by default (i.e. DtX()).
/// To perform numerical computations, we need to specify an "expressed-in"
/// frame E (which may be distinct from either X or Y), so that components can
/// be expressed as real numbers.
/// Only the vector values are stored in a %SpatialAcceleration object; the
/// frames must be understood from context and it is the responsibility of the
/// user to keep track of them. That is best accomplished through disciplined
/// notation.
/// In source code we use monogram notation where capital A is used to designate
/// a spatial acceleration quantity. The same monogram notation rules for
/// SpatialVelocity are also used for %SpatialAcceleration. That is, the spatial
/// acceleration of a frame Y measured in X and expressed in E is denoted with
/// `A_XY_E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
///
/// [Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialAcceleration : public SpatialVector<SpatialAcceleration, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialAcceleration, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialAcceleration)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial acceleration are left uninitialized resulting in a
  /// zero cost operation. However in Debug builds those entries are set to NaN
  /// so that operations using this uninitialized spatial acceleration fail
  /// fast, allowing fast bug detection.
  SpatialAcceleration() : Base() {}

  /// SpatialAcceleration constructor from an angular acceleration `alpha` and
  /// a linear acceleration `a`.
  SpatialAcceleration(const Eigen::Ref<const Vector3<T>>& alpha,
                      const Eigen::Ref<const Vector3<T>>& a) : Base(alpha, a) {}

  /// SpatialAcceleration constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// Under the hood, spatial accelerations are 6-element quantities that are
  /// pairs of ordinary 3-vectors. Elements 0-2 constitute the angular
  /// acceleration component while elements 3-5 constitute the translational
  /// acceleration. The argument `A` in this constructor is the concatenation
  /// of the rotational 3D component followed by the translational 3D
  /// component.
  /// This constructor will assert the size of `A` is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialAcceleration(const Eigen::MatrixBase<Derived>& A) : Base(A) {}

  /// In-place shift of `this` spatial acceleration `A_WP` of a frame P into the
  /// spatial acceleration `A_WPb` of a frame `Pb` which is an offset frame
  /// rigidly aligned with P, but with its origin shifted to a point `Bo` by an
  /// offset p_PoBo. Frame `Pb` is instantaneously moving together with frame P
  /// as if rigidly attached to it.
  /// As an example of application, this operation can be used to compute
  /// `A_WPb` where P is a frame on a rigid body and `Bo` is another point on
  /// that same body. Therefore P and `Pb` move together with the spatial
  /// velocity `V_PPb` being zero at all times.
  ///
  /// The shift operation modifies `this` spatial acceleration `A_WP_E` of a
  /// frame P measured in a frame W and expressed in a frame E, to become
  /// `A_WPb_E`, representing the acceleration of a frame `Pb` result of
  /// shifting frame P to point `Bo` which instantaneously moves together with
  /// frame P. This requires adjusting the linear acceleration component to
  /// account for:
  ///
  ///   1. the angular acceleration `alpha_WP` of frame P in W.
  ///   2. the centrifugal acceleration due to the angular velocity `w_WP` of
  ///      frame P in W.
  ///
  /// We are given the vector from the origin `Po` of frame P to point `Bo`,
  /// which becomes the origin of the shifted frame `Pb`, as the position vector
  /// `p_PoBo_E` expressed in the same frame E as `this` spatial acceleration.
  /// The operation performed, in coordinate-free form, is: <pre>
  ///   alpha_WPb  = alpha_WP,  i.e. the angular acceleration is unchanged.
  ///   a_WBo = a_WPo + alpha_WP x p_PoBo + w_WP x w_WP x p_PoBo
  /// </pre>
  /// where `alpha` and `a` represent the angular and linear acceleration
  /// components respectively. See notes at the end of this documentation for a
  /// detailed derivation.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_PoBo_E
  ///   Shift vector from the origin `Po` of frame P to point `Bo`, expressed in
  ///   frame E. The "from" frame P must be the frame whose acceleration is
  ///   currently represented in `this` spatial acceleration, and E must be the
  ///   same expressed-in frame as for this spatial acceleration.
  /// @param[in] w_WP_E
  ///   Angular velocity of frame P measured in frame W and expressed in frame
  ///   E.
  ///
  /// @returns A reference to `this` spatial acceleration which is now
  ///   `A_WPb_E`, that is, the spatial acceleration of frame `Pb`, still
  ///   measured in frame W and expressed in frame E.
  ///
  /// @see Shift() to compute the shifted spatial acceleration without modifying
  ///      this original object.
  ///
  /// <h3> Derivation </h3>
  ///
  /// <h4> Translational acceleration component </h4>
  ///
  /// Recall that frame `Pb` is an offset frame rigidly aligned with P, but with
  /// its origin shifted to a point `Bo` by an offset `p_PoBo`. Frame `Pb` is
  /// instantaneously moving together with frame P as if rigidly attached to it.
  /// The translational velocity `v_WPb` of frame `Pb`'s origin, point `Bo`, in
  /// W can be obtained by the shift operation as: <pre>
  ///   v_WPb = v_WPo + w_WP x p_PoBo                                       (1)
  /// </pre>
  /// Therefore, for the translational acceleration we have: <pre>
  ///   a_WBo = DtW(v_WPb)
  ///         = DtW(v_WPo + w_WP x p_PoBo)
  ///         = DtW(v_WPo) + DtW(w_WP x p_PoBo)
  ///         = a_WPo + DtW(w_WP) x p_PoBo + w_WP x DtW(p_PoBo)
  ///         = a_WPo + alpha_WP x p_PoBo + w_WP x DtW(p_PoBo)              (2)
  /// </pre>
  /// with `a_WPo = DtW(v_WPo)` and `alpha_WP = DtW(w_WP)` by definition.
  /// The last term in Eq. (2) is obtained by converting the vector time
  /// derivative from `DtW()` to `DtP()`,
  /// see drake::math::ConvertTimeDerivativeToOtherFrame(): <pre>
  ///   DtW(p_PoBo) = DtP(p_PoBo) + w_WP x p_PoBo
  ///               = w_WP x p_PoBo                                         (3)
  /// </pre>
  /// since `v_PBo = DtP(p_PoBo) = 0` because the position of point `Bo` is
  /// fixed in frame P. Using Eq. (3) in Eq. (2) finally yields for the
  /// translational acceleration: <pre>
  ///   a_WBo = a_WPo + alpha_WP x p_PoBo + w_WP x w_WP x p_PoBo            (4)
  /// </pre>
  ///
  /// <h4> Rotational acceleration component </h4>
  ///
  /// The rotational velocity of frame `Pb` simply equals that of frame P since
  /// they are moving together in rigid motion, therefore `w_WPb = w_WP`.
  /// From this, the rotational acceleration of frame `Pb` in W is obtained as:
  /// <pre>
  ///   alpha_WPb = DtW(w_WPb) = DtW(w_WP) = alpha_WP                       (5)
  /// </pre>
  /// which should be immediately obvious considering that frame `Pb` rotates
  /// together with frame P.
  ///
  /// With the rotational, Eq. (5), and translational, Eq. (4), components of
  /// acceleration derived above, we can write for `A_WPb`: <pre>
  ///   A_WPb.rotational() = alpha_WPb = alpha_WP
  ///   A_WPb.translational() = a_WBo
  ///                         = a_WPo + alpha_WP x p_PoBo + w_WP x w_WP x p_PoBo
  /// </pre>
  /// with `alpha_WP = A_WP.rotational()` and `a_WPo = A_WP.translational()`.
  /// As usual, for computation, all quantities above must be expressed in a
  /// common frame E; we add an `_E` suffix to each symbol to indicate that.
  SpatialAcceleration<T>& ShiftInPlace(const Vector3<T>& p_PoBo_E,
                                       const Vector3<T>& w_WP_E) {
    // Angular acceleration of this frame P measured in W.
    const Vector3<T>& alpha_WP_E = this->rotational();
    // Linear acceleration of point Bo measured in W.
    Vector3<T>& a_WBo_E = this->translational();
    a_WBo_E += (alpha_WP_E.cross(p_PoBo_E) +
        w_WP_E.cross(w_WP_E.cross(p_PoBo_E)));
    return *this;
  }

  /// Shifts `this` spatial acceleration `A_WP` of a frame P into the
  /// spatial acceleration `A_WPb` of a frame `Pb` which is an offset frame
  /// rigidly aligned with P, but with its origin shifted to a point `Bo` by an
  /// offset p_PoBo. Frame `Pb` is instantaneously moving together with frame P
  /// as if rigidly attached to it.
  /// As an example of application, this operation can be used to compute
  /// `A_WPb` where P is a frame on a rigid body and `Bo` is another point on
  /// that same body. Therefore P and `Pb` move together with the spatial
  /// velocity `V_PPb` being zero at all times.
  /// This is an alternate signature for shifting a spatial acceleration that
  /// does not change the original object.
  /// See ShiftInPlace() for more information and a description of the
  /// arguments.
  SpatialAcceleration<T> Shift(const Vector3<T>& p_PoBo_E,
                               const Vector3<T>& w_WP_E) const {
    return SpatialAcceleration<T>(*this).ShiftInPlace(p_PoBo_E, w_WP_E);
  }


  /// This method composes `this` spatial acceleration `A_WP` of a frame P
  /// measured in a frame W, with that of a third frame B moving in P with
  /// spatial acceleration `A_PB`. The result is the spatial acceleration
  /// `A_WB` of frame B measured in W. At the instant in which the accelerations
  /// are composed, frame B is located with its origin `Bo` at `p_PoBo` from P's
  /// origin Po.
  ///
  /// This operation can be written in a more compact fom in terms of the
  /// rigid shift operator `Φᵀ(p_PoBo)` (see SpatialVelocity::Shift()) as: <pre>
  ///   A_WB = Φᵀ(p_PoBo)A_WP + Ac_WB(w_WP, V_PB) + A_PB_W
  /// </pre>
  /// where `Φᵀ(p_PoBo)A_WP` denotes the application of the rigid shift
  /// operation as in SpatialVelocity::Shift() and `Ac_WB(w_WP, V_PB)` contains
  /// the centrifugal and Coriolis terms: <pre>
  ///   Ac_WB(w_WP, V_PB) = | w_WP x w_PB_W                            |
  ///                       | w_WP x w_WP x p_PoBo_W + 2 w_WP x v_PB_W |
  ///                                   ^^^                ^^^
  ///                               centrifugal         Coriolis
  /// </pre>
  /// The equation above shows that composing spatial accelerations cannot be
  /// simply accomplished by adding `A_WP` with `A_PB` (this is the reason why
  /// this class does not overload the `operator+()` and provides this method
  /// instead). Moreover, we see that, unlike with angular velocities, angular
  /// accelerations cannot be added in order to compose them. That is
  /// `w_AC = w_AB + w_BC` but `alpha_AC ≠ alpha_AB + alpha_BC` due to the cross
  /// term `w_AC x w_BC`. See the derivation below for more details.
  ///
  /// @see SpatialVelocity::ComposeWithMovingFrameVelocity() for the composition
  /// of SpatialVelocity quantities.
  ///
  /// @note This method is the extension to the Shift() operator, which computes
  /// the spatial acceleration frame P shifted to `Bo` as if frame B moved
  /// rigidly with P, that is, for when `V_PB` and `A_PB` are both zero.
  /// In other words the results from Shift() equal the results from
  /// this method when `V_PB` and `A_PB` are both zero.
  ///
  /// @param[in] p_PoBo_E
  ///   Shift vector from P's origin to B's origin, expressed in frame E.
  ///   The "from" point `Po` must be the point whose acceleration is currently
  ///   represented in `this` spatial acceleration, and E must be the same
  ///   expressed-in frame as for `this` spatial acceleration.
  /// @param[in] w_WP_E
  ///   Angular velocity of frame P measured in frame A and expressed in frame
  ///   E.
  /// @param[in] V_PB_E
  ///   The spatial velocity of a third frame B in motion with respect to P,
  ///   expressed in the same frame E as `this` spatial acceleration.
  /// @param[in] A_PB_E
  ///   The spatial acceleration of a third frame B in motion with respect to P,
  ///   expressed in the same frame E as `this` spatial acceleration.
  ///
  /// @retval A_WB_E The spatial acceleration of frame B in W, expressed in
  ///                frame E.
  ///
  /// <h3> Derivation </h3>
  /// The spatial velocity of frame B in W can be obtained by composing `V_WP`
  /// with `V_PB`: <pre>
  ///   V_WB = V_WPb + V_PB = V_WP.Shift(p_PoBo) + V_PB                     (1)
  /// </pre>
  /// This operation can be performed with the SpatialVelocity method
  /// ComposeWithMovingFrameVelocity().
  ///
  /// <h4> Translational acceleration component </h4>
  ///
  /// The translational velocity `v_WBo` of point Bo in W corresponds to the
  /// translational component in Eq. (1): <pre>
  ///   v_WBo = v_WPo + w_WP x p_PoBo + v_PBo                               (2)
  /// </pre>
  /// Therefore, for the translational acceleration we have: <pre>
  ///   a_WBo = DtW(v_WBo)
  ///         = DtW(v_WPo + w_WP x p_PoBo + v_PBo)
  ///         = DtW(v_WPo) + DtW(w_WP x p_PoBo) + DtW(v_PBo)
  ///         = a_WPo + DtW(w_WP) x p_PoBo + w_WP x DtW(p_PoBo) + DtW(v_PBo)
  ///         = a_WPo + alpha_WP x p_PoBo + w_WP x DtW(p_PoBo) + DtW(v_PBo) (3)
  /// </pre>
  /// with `a_WPo = DtW(v_WPo)` and `alpha_WP = DtW(w_WP)` by definition.
  /// The term DtW(p_PoBo) in Eq. (3) is obtained by converting the vector time
  /// derivative from `DtW()` to `DtP()`,
  /// see drake::math::ConvertTimeDerivativeToOtherFrame(): <pre>
  ///   DtW(p_PoBo) = DtP(p_PoBo) + w_WP x p_PoBo
  ///               = v_PBo + w_WP x p_PoBo                                 (4)
  /// </pre>
  /// since `v_PBo = DtP(p_PoBo)` by definition.
  /// Similarly, the term `DtW(v_PBo)` in Eq. (3) is also obtained by converting
  /// the time derivative from `DtW()` to `DtP()`: <pre>
  ///   DtW(v_PBo) = DtP(v_PBo) + w_WP x v_PBo
  ///              = a_PBo + w_WP x v_PBo                                   (5)
  /// </pre>
  /// with `a_PBo = DtP(v_PBo)` by definition.
  /// Using Eqs. (4) and (5) in Eq. (3) yields for the translational
  /// acceleration: <pre>
  ///   a_WBo = a_WPo + alpha_WP x p_PoBo
  ///         + w_WP x (v_PBo + w_WP x p_PoBo) + a_PBo + w_WP x v_PBo
  /// </pre>
  /// and finally, by grouping terms together: <pre>
  ///   a_WBo = a_WPo + alpha_WP x p_PoBo
  ///         + w_WP x w_WP x p_PoBo + 2 * w_WP x v_PBo + a_PBo             (6)
  /// </pre>
  /// which includes the effect of angular acceleration of P in W
  /// `alpha_WP x p_PoBo`, the centrifugual acceleration `w_WP x w_WP x p_PoBo`,
  /// the Coriolis acceleration `2 * w_WP x v_PBo` due to the motion of `Bo` in
  /// P and, the additional acceleration of `Bo` in P `a_PBo`.
  ///
  /// <h4> Rotational acceleration component </h4>
  ///
  /// The rotational velocity `w_WB` of frame B in W corresponds to the
  /// rotational component in Eq. (1): <pre>
  ///   w_WB = w_WP + w_PB                                                  (7)
  /// </pre>
  /// Therefore, the rotational acceleration of B in W corresponds to: <pre>
  ///   alpha_WB = DtW(w_WB) = DtW(w_WP) + DtW(w_PB)
  ///            = alpha_WP + DtW(w_PB)                                     (8)
  /// </pre>
  /// where the last term in Eq. (8) can be converted to a time derivative in P
  /// as: <pre>
  ///   DtW(w_PB) = DtP(w_PB) + w_WP x w_PB = alpha_PB + w_WP x w_PB        (9)
  /// </pre>
  /// where `alpha_PB = DtP(w_PB)` by definition.
  /// Thus, the final expression for `alpha_WB` is obtained by using Eq. (9)
  /// into Eq. (8): <pre>
  ///   alpha_WB = alpha_WP + alpha_PB + w_WP x w_PB                       (10)
  /// </pre>
  /// Equation (10) shows that angular accelerations cannot be simply added as
  /// angular velocities can but there exists an additional term `w_WP x w_PB`.
  ///
  /// <h4> The spatial acceleration </h4>
  ///
  /// The rotational and translational components of the spatial acceleration
  /// are given by Eqs. (10) and (6) respectively: <pre>
  ///   A_WB.rotational() = alpha_WB = {alpha_WP} + alpha_PB + w_WP x w_PB (11)
  ///   A_WB.translational() = a_WBo
  ///                      = {a_WPo + alpha_WP x p_PoBo + w_WP x w_WP x p_PoBo}
  ///                      + 2 * w_WP x v_PBo + a_PBo                      (12)
  /// </pre>
  /// where we have placed within curly brackets `{}` all the terms that also
  /// appear in the Shift() operation, which is equivalent to this method when
  /// `V_PB` and `A_PB` are both zero.
  /// In the equations above `alpha_WP = A_WP.rotational()` and
  /// `a_WPo = A_WP.translational()`.
  /// The above expression can be written in a more compact fom in terms of the
  /// rigid shift operator `Φᵀ(p_PoBo)` (see SpatialVelocity::Shift()) as
  /// presented in the main body of this documentation: <pre>
  ///   A_WB = Φᵀ(p_PoBo)A_WP + Ac_WB(w_WP, V_PB) + A_PB_W                 (13)
  /// </pre>
  /// where `Ac_WB(w_WP, V_PB)` contains the centrifugal and Coriolis terms:
  /// <pre>
  ///   Ac_WB(w_WP, V_PB) = | w_WP x w_PB_W                            |
  ///                       | w_WP x w_WP x p_PoBo_W + 2 w_WP x v_PB_W |
  ///                                   ^^^                ^^^
  ///                               centrifugal         Coriolis
  /// </pre>
  /// As usual, for computation, all quantities above must be expressed in a
  /// common frame E; we add an `_E` suffix to each symbol to indicate that.
  SpatialAcceleration<T> ComposeWithMovingFrameAcceleration(
      const Vector3<T>& p_PoBo_E, const Vector3<T>& w_WP_E,
      const SpatialVelocity<T>& V_PB_E,
      const SpatialAcceleration<T>& A_PB_E) const {
    const Vector3<T>& w_PB_E = V_PB_E.rotational();
    const Vector3<T>& v_PBo_E = V_PB_E.translational();

    // Compute all the terms within curly brackets in the derivation above which
    // correspond to the Shift() operation:
    SpatialAcceleration<T> A_WB_E = this->Shift(p_PoBo_E, w_WP_E);
    // Adds non-linear coupling of angular velocities:
    A_WB_E.rotational() += (A_PB_E.rotational() + w_WP_E.cross(w_PB_E));

    // Adds Coriolis and translational acceleration of Bo in P.
    A_WB_E.translational() +=
        (A_PB_E.translational() + 2.0 * w_WP_E.cross(v_PBo_E));
    return A_WB_E;
  }
};

}  // namespace multibody
}  // namespace drake
