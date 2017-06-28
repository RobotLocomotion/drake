#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/shift_time_derivative.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace multibody {

/// This class is used to represent a _spatial acceleration_ that combines
/// rotational (angular acceleration) and translational (linear acceleration)
/// components. Spatial accelerations are 6-element quantities that are pairs of
/// ordinary 3-vectors. Elements 0-2 constitute the angular acceleration
/// component while elements 3-5 constitute the translational acceleration.
/// While a SpatialVelocity `V_XY` represents the motion of a "moving frame"
/// Y measured with respect to a "measured-in" frame X, the %SpatialAcceleration
/// `A_XY` represents the rate of change of this spatial velocity `V_XY` in
/// frame X. That is @f$^XA^Y = \frac{^Xd ^XV^Y}{dt} @f$ where
/// @f$\frac{^Xd}{dt} @f$ denotes the time derivative taken in frame X. In
/// source code comments we write the previous expression as `A_XY = d/dt V_XY`.
/// It is important to note that the frame in which the time derivative is taken
/// matters, generally leading to different values in different frames. By
/// convention, and unless otherwise stated, we assume that the frame in which
/// the time derivative is taken is the "measured-in" frame.
/// As with SpatialVelocity, the two contained vectors must be expressed in the
/// same "expressed-in" frame E, which may be distinct from either X or Y.
/// Finally, while angular acceleration is identical for any frame fixed to a
/// rigid body, translational acceleration refers to a particular point. Only
/// the vector values are stored in a %SpatialAcceleration object; the frames
/// must be understood from context and it is the responsibility of the user to
/// keep track of them. That is best accomplished through disciplined notation.
/// In source code we use monogram notation where capital A is used to designate
/// a spatial acceleration quantity. The same monogram notation rules for
/// SpatialVelocity are also used for %SpatialAcceleration. That is, the spatial
/// acceleration of a frame Y measured in X and expressed in E is denoted with
/// `A_XY_E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
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
  /// This constructor will assert the size of A is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialAcceleration(const Eigen::MatrixBase<Derived>& A) : Base(A) {}

  /// In-place shift of this spatial acceleration `A_FP` of a frame P into the
  /// spatial acceleration `A_FPq` of a frame Pq which is an offset frame
  /// rigidly aligned with P, but with its origin shifted to a pint Q by an
  /// offset p_PoQo. Frame Pq is instantaneously moving together with frame P as
  /// if rigidly attached to it.
  /// As an example of application, this operation can be used to compute
  /// `A_FPq` where P is a frame on a rigid body and Q is another point on that
  /// same body. Therefore P and `Pq` move together with the spatial velocity
  /// V_PPq being zero at all times.
  ///
  /// The shift operation modifies `this` spatial acceleration `A_FP_E` of a
  /// frame P measured in a frame F and expressed in a frame E, to become
  /// `A_FPq_E`, representing the acceleration of a frame `Pq` shifted to Qo
  /// which instantaneously moves together with frame P. This requires adjusting
  /// the linear acceleration component to account for:
  ///   1. the angular acceleration `alpha_FP` of frame P in F.
  ///   2. the centrifugal acceleration due to the angular velocity `w_FP` of
  ///      frame P in F.
  ///
  /// We are given the vector from the origin `Po` of frame P to point Q, which
  /// is the origin of the shifted frame `Pq`, as the position vector `p_PoQo_E`
  /// expressed in the same frame E as `this` spatial acceleration. The
  /// operation performed, in coordinate-free form, is:
  /// <pre>
  ///   alpha_FPq  = alpha_FP,  i.e. the angular acceleration is unchanged.
  ///   a_FQo = a_FPo + alpha_FP x p_PoQo + w_FP x w_FP x p_PoQo
  /// </pre>
  /// where `alpha` and `a` represent the angular and linear acceleration
  /// components respectively.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_PoQo_E
  ///   Shift vector from the origin `Po` of frame P to point Q, expressed in
  ///   frame E. The "from" frame P must be the frame whose acceleration is
  ///   currently represented in `this` spatial acceleration, and E must be the
  ///   same expressed-in frame as for this spatial acceleration.
  /// @param[in] w_FP_E
  ///   Angular velocity of frame P measured in frame A and expressed in frame
  ///   E.
  ///
  /// @note For both input parameters, `p_PoQo_E` and `w_FP_E`, frame P must be
  ///   the frame whose acceleration is currently represented in `this` spatial
  ///   acceleration `A_FP_E`, and E must be the same expressed-in frame as for
  ///   this spatial acceleration.
  ///
  /// @returns A reference to `this` spatial acceleration which is now
  ///   `A_FPq_E`, that is, the spatial acceleration of frame `Pq`, still
  ///   measured in frame F and expressed in frame E.
  ///
  /// @see Shift() to compute the shifted spatial acceleration without modifying
  ///      this original object.
  SpatialAcceleration<T>& ShiftInPlace(const Vector3<T>& p_PoQo_E,
                                       const Vector3<T>& w_FP_E) {
    // Angular acceleration of this frame P measured in A.
    const Vector3<T>& alpha_FP_E = this->rotational();
    // Linear acceleration of point Qo measured in A.
    Vector3<T>& a_FQo_E = this->translational();
    a_FQo_E += (alpha_FP_E.cross(p_PoQo_E) +
        w_FP_E.cross(w_FP_E.cross(p_PoQo_E)));
    return *this;
  }

  /// In-place shift of this spatial acceleration `A_FP` of a frame P into the
  /// spatial acceleration `A_FPq` of a frame Pq which is an offset frame
  /// rigidly aligned with P, but with its origin shifted to a pint Q by an
  /// offset p_PoQo. Frame Pq is instantaneously moving together with frame P as
  /// if rigidly attached to it.
  /// This is an alternate signature for shifting a spatial acceleration that
  /// does not change the original object.
  /// See ShiftInPlace() for more information and a description of the
  /// arguments.
  SpatialAcceleration<T> Shift(const Vector3<T>& p_PoQo_E,
                               const Vector3<T>& w_AP_E) const {
    return SpatialAcceleration<T>(*this).ShiftInPlace(p_PoQo_E, w_AP_E);
  }

  /// Given the time derivative `ᴮd/dt(V)` of an arbitrary SpatialVelocity V in
  /// a frame B moving with angular velocity `w_AB` with respect to another
  /// frame A, this method computes (shifts) the time derivative `ᴬd/dt(V)` of
  /// the same SpatialVelocity V in frame A.
  /// Mathematically, this corresponds to performing the shift operation on the
  /// rotational and translational components (which are 3D vectors) of V
  /// separately. The operation on each 3D vector component is performed by
  /// drake::math::ShiftTimeDerivative() as: <pre>
  ///   ᴬd/dt(Vw) = ᴮd/dt(Vw) + w_AB x Vw
  ///   ᴬd/dt(Vv) = ᴮd/dt(Vv) + w_AB x Vv
  /// </pre>
  /// where `Vw` and `Vv` denote the rotational() and the translational()
  /// components of V respectively.
  ///
  /// The spatial velocity V can be arbitrary, between two arbitrary frames P
  /// and Q, i.e. `V_PQ`.
  ///
  /// In source code and comments we use the monogram notation
  /// `DtA_V_E = [ᴬd/dt(V)]_E` to denote the time derivative of the spatial
  /// velocity V in a frame A with the result returned as a %SpatialAcceleration
  /// expressed in a frame E.
  /// To perform this operation numerically all quantities must be expressed in
  /// the same frame E. Using the monogram notation: <pre>
  ///   DtA_Vw_E = DtB_Vw_E + w_AB_E x Vw_E
  ///   DtA_Vv_E = DtB_Vv_E + w_AB_E x Vv_E
  /// </pre>
  ///
  /// This operation is commonly known as the "Transport Theorem" while
  /// [Mitiguy 2016, §7.3] refers to it as to the "Golden Rule for Vector
  /// Differentiation".
  ///
  /// [Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
  static SpatialAcceleration<T> ShiftTimeDerivative(
      const SpatialVelocity<T>& V_E,
      const SpatialAcceleration<T>& DtB_V_E,
      const Vector3<T>& w_AB) {
    return SpatialAcceleration(
        drake::math::ShiftTimeDerivative(
            V_E.rotational(), DtB_V_E.rotational(), w_AB),
        drake::math::ShiftTimeDerivative(
            V_E.translational(), DtB_V_E.translational(), w_AB));
  }
};

/// Performs the addition of two spatial accelerations. This operator
/// returns the spatial acceleration that results from adding the operands as if
/// they were 6-dimensional vectors. In other words, the resulting spatial
/// acceleration contains a rotational component which is the 3-dimensional
/// addition of the operand's rotational components and a translational
/// component which is the 3-dimensional addition of the operand's translational
/// components.
///
/// The addition of two spatial accelerations can only be performed if the
/// operands meet strict conditions, in addition the the usual requirement of
/// common expressed-in frames.
///
/// Adding two spatial velocities has the effect of composing them like so (in
/// coordinate-free form, with no expressed-in frame implied):
/// <pre>
///   V_EB = V_EAb + V_AB = V_EA.Shift(p_AB) + V_AB
/// </pre>
/// where `p_AB` is the position vector from A's origin to B's origin. This
/// shift can also be thought of as yielding the spatial velocity of a new frame
/// Ab, which is an offset frame rigidly aligned with A, but with its origin
/// shifted to B's origin.
/// By definition, the spatial acceleration of frame B in E is
/// `A_EB = DtE(V_EB)`. Now, applying this derivative (the DtE() operator) to
/// the spatial velocity composition above yields: <pre>
///   A_EB = DtE(V_EB) = DtE(V_EAb + V_AB) = DtE(V_EAb) + DtE(V_AB)
/// </pre>
/// The first term is obtained by shifting the spatial acceleration `A_EA` to B
/// as: <pre>
///   DtE(V_EAb) = A_EAb = A_EA.Shift(p_AB, w_EA)
/// </pre>
/// while the second therm can be obtained by shifting the time derivative from
/// DtA() to DtE() with SpatialAcceleration::ShiftTimeDerivative() as: <pre>
///   DtE(V_AB) = ShiftTimeDerivative(V_AB, A_AB, w_EA)
/// </pre>
///
/// In summary, this operator carries out the addition: <pre>
///   A_EB_E = A_EAb_E + DtE_V_AB_E
/// </pre>
/// the caller must have already performed the necessary shifts (by calling
/// both SpatialAcceleration::Shift() and
/// SpatialAcceleration::ShiftTimeDerivative().) Notice that not only all
/// quantities are expressed in the same frame but also all time derivatives are
/// taken in the same frame.
template <typename T>
inline SpatialAcceleration<T> operator+(
    const SpatialAcceleration<T>& A_EAb,
    const SpatialAcceleration<T>& DtE_V_AB_E) {
  return SpatialAcceleration<T>(A_EAb.get_coeffs() + DtE_V_AB_E.get_coeffs());
}

}  // namespace multibody
}  // namespace drake
