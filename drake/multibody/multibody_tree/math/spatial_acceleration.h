#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

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
  /// spatial acceleration of another frame Q instantaneously moving together
  /// with frame P.
  /// As an example of application, this operation can be used when both frames
  /// P and Q are attached to a rigid body and therefore move together with the
  /// spatial velocity V_PQ being zero at all times.
  ///
  /// The shift operation modifies `this` spatial acceleration `A_FP_E` of a
  /// frame P measured in a frame F and expressed in a frame E, to become
  /// `A_FQ_E`, representing the acceleration of another frame Q which
  /// instantaneously moves together with frame P. This requires adjusting the
  /// linear acceleration component to account for:
  ///   1. the angular acceleration `alpha_FP` of frame P in F.
  ///   2. the centrifugal acceleration due to the angular velocity `w_FP` of
  ///      frame P in F.
  ///
  /// We are given the vector from the origin `Po` of frame P to the origin `Qo`
  /// of frame Q as the position vector `p_PoQo_E` expressed in the same frame E
  /// as `this` spatial acceleration. The operation performed, in
  /// coordinate-free form, is:
  /// <pre>
  ///   alpha_FQ  = alpha_FP,  i.e. the angular acceleration is unchanged.
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
  ///   Shift vector from the origin `Po` of frame P to the origin `Qo` of
  ///   frame Q expressed in frame E. The "from" frame P must be the frame
  ///   whose acceleration is currently represented in `this` spatial
  ///   acceleration, and E must be the same expressed-in frame as for this
  ///   spatial acceleration.
  /// @param[in] w_FP_E
  ///   Angular velocity of frame P measured in frame A and expressed in frame
  ///   E.
  ///
  /// @note For both input parameters, `p_PoQo_E` and `w_FP_E`, frame P must be
  ///   the frame whose acceleration is currently represented in `this` spatial
  ///   acceleration `A_FP_E`, and E must be the same expressed-in frame as for
  ///   this spatial acceleration.
  ///
  /// @returns A reference to `this` spatial acceleration which is now `A_FQ_E`,
  ///   that is, the spatial acceleration of frame Q, still measured in frame F
  ///   and expressed in frame E.
  ///
  /// @see Shift() to compute the shifted spatial acceleration without modifying
  ///      this original object.
  SpatialAcceleration<T>& ShiftInPlace(const Vector3<T>& p_PoQo_E,
                                       const Vector3<T>& w_AP_E) {
    // Angular acceleration of this frame P measured in A.
    const Vector3<T>& alpha_AP_E = this->rotational();
    // Linear acceleration of point Qo measured in A.
    Vector3<T>& a_AQo_E = this->translational();
    a_AQo_E += (alpha_AP_E.cross(p_PoQo_E) +
        w_AP_E.cross(w_AP_E.cross(p_PoQo_E)));
    return *this;
  }

  /// Shift of this spatial acceleration `A_FP` of a frame P into the
  /// spatial acceleration of another frame Q instantaneously moving together
  /// with frame P.
  /// This is an alternate signature for shifting a spatial acceleration that
  /// does not change the original object. See ShiftInPlace() for more
  /// information.
  ///
  /// @param[in] p_PoQo_E
  ///   Shift vector from the origin `Po` of frame P to the origin `Qo` of
  ///   frame Q expressed in frame E. The "from" frame P must be the frame
  ///   whose acceleration is currently represented in `this` spatial
  ///   acceleration, and E must be the same expressed-in frame as for this
  ///   spatial acceleration.
  /// @param[in] w_FP_E
  ///   Angular velocity of frame P measured in frame A and expressed in frame
  ///   E.
  ///
  /// @note For both input parameters, `p_PoQo_E` and `w_FP_E`, frame P must be
  ///   the frame whose acceleration is currently represented in `this` spatial
  ///   acceleration `A_FP_E`, and E must be the same expressed-in frame as for
  ///   this spatial acceleration.
  ///
  /// @retval A_FQ_E
  ///   The spatial acceleration of frame Q measured in frame F and expressed
  ///   in frame E.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial acceleration in-place
  ///      modifying the original object.
  SpatialAcceleration<T> Shift(const Vector3<T>& p_PoQo_E,
                               const Vector3<T>& w_AP_E) const {
    return SpatialAcceleration<T>(*this).ShiftInPlace(p_PoQo_E, w_AP_E);
  }
};

}  // namespace multibody
}  // namespace drake
