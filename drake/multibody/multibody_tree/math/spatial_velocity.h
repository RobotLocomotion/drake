#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial force.
template <typename T> class SpatialForce;

/// This class is used to represent a _spatial velocity_ (also called a
/// _twist_) that combines rotational (angular) and translational
/// (linear) velocity components. Spatial velocities are 6-element
/// quantities that are pairs of ordinary 3-vectors. Elements 0-2 are
/// the angular velocity component while elements 3-5 are the translational
/// velocity. Spatial velocities represent the motion of a "moving frame"
/// B measured with respect to a "measured-in" frame A. In addition,
/// the two contained vectors must be expressed in the same "expressed-in"
/// frame E, which may be distinct from either A or B. Finally,
/// while angular velocity is identical for any frame fixed to a rigid
/// body, translational velocity refers to a particular point. Only the
/// vector values are stored in a %SpatialVelocity object; the three
/// frames and the point must be understood from context. It is the
/// responsibility of the user to keep track of them. That is best
/// accomplished through disciplined notation. In source code we use
/// monogram notation where capital V is used to designate a spatial
/// velocity quantity. We write a point P fixed to body (or frame)
/// B as @f$B_P@f$ which appears in code and comments as `Bp`. Then
/// we write a particular spatial velocity as `V_ABp_E` where the `_E`
/// suffix indicates that the expressed-in frame is E. This symbol
/// represents the angular velocity of frame B in frame A, and the
/// translational velocity of point P in A, where P is fixed to
/// frame B, with both vectors expressed in E. Very often
/// the point of interest will be the body origin `Bo`; if no point is
/// shown the origin is understood, so `V_AB_E` means `V_ABo_E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialVelocity : public SpatialVector<SpatialVelocity, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialVelocity, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialVelocity)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial velocity are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial velocity fail fast,
  /// allowing fast bug detection.
  SpatialVelocity() : Base() {}

  /// SpatialVelocity constructor from an angular velocity @p w and a linear
  /// velocity @p v.
  SpatialVelocity(const Eigen::Ref<const Vector3<T>>& w,
                  const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of V is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialVelocity(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialVelocity from one point on a rigid body
  /// or frame to another point on the same body or frame.
  /// `this` spatial velocity `V_ABp_E` of a frame B at a point P fixed
  /// on B, measured in a frame A, and expressed in a frame E, is
  /// modified to become `V_ABq_E`, representing the velocity of another
  /// point Q on B instead (see class comment for more about this
  /// notation). This requires adjusting the translational (linear) velocity
  /// component to account for the velocity difference between P and Q
  /// due to the angular velocity of B in A.
  ///
  /// We are given the vector from point P to point Q, as a position
  /// vector `p_BpBq_E` (or `p_PQ_E`) expressed in the same frame E as the
  /// spatial velocity. The operation performed, in coordinate-free form, is:
  /// <pre>
  ///   w_AB  = w_AB,  i.e. the angular velocity is unchanged.
  ///   v_ABq = v_ABp + w_AB x p_BpBq
  /// </pre>
  /// where w and v represent the angular and linear velocity components
  /// respectively.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the point
  ///   whose velocity is currently represented in this spatial velocity,
  ///   and E must be the same expressed-in frame as for this spatial
  ///   velocity.
  ///
  /// @returns A reference to `this` spatial velocity which is now `V_ABq_E`,
  ///   that is, the spatial velocity of frame B at point Q, still
  ///   measured in frame A and expressed in frame E.
  ///
  /// @see Shift() to compute the shifted spatial velocity without modifying
  ///      this original object.
  SpatialVelocity<T>& ShiftInPlace(const Vector3<T>& p_BpBq_E) {
    this->translational() += this->rotational().cross(p_BpBq_E);
    return *this;
  }

  /// Shift of a %SpatialVelocity from one point on a rigid body
  /// or frame to another point on the same body or frame.
  /// This is an alternate signature for shifting a spatial velocity's
  /// point that does not change the original object. See
  /// ShiftInPlace() for more information.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the point
  ///   whose velocity is currently represented in this spatial velocity,
  ///   and E must be the same expressed-in frame as for this spatial
  ///   velocity.
  ///
  /// @retval V_ABq_E
  ///   The spatial velocity of frame B at point Q, measured in frame
  ///   A and expressed in frame E.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial velocity in-place
  ///      modifying the original obje  ct.
  SpatialVelocity<T> Shift(const Vector3<T>& p_BpBq_E) const {
    return SpatialVelocity<T>(*this).ShiftInPlace(p_BpBq_E);
  }

  /// Given `this` spatial velocity `V_IBp_E` of point P of body B,
  /// measured in an inertial frame I and expressed in a frame E,
  /// this method computes the 6-dimensional dot product with the spatial
  /// force `F_Bp_E` applied to point P, and expressed in the same
  /// frame E in which the spatial velocity is expressed.
  /// This dot-product represents the power generated by the spatial force
  /// when its body and application point have `this` spatial velocity.
  /// Although the two spatial vectors must be expressed in the same frame,
  /// the result is independent of that frame.
  ///
  /// @warning The result of this method cannot be interpreted as power unless
  ///          `this` spatial velocity is measured in an inertial frame I,
  ///          which cannot be enforced by this class.
  T dot(const SpatialForce<T>& F_Q_E) const;
};

/// Operator to perform the addition of two spatial velocities. This operator
/// returns the spatial velocity that results from adding the operands as if
/// they were 6-dimensional vectors. In other words, the resulting spatial
/// velocity contains a rotational component which is the 3-dimensional
/// addition of the operand's rotational components and a translational
/// component which is the 3-dimensional addition of the operand's translational
/// components.
///
/// The addition of two spatial velocities has a clear physical meaning.
/// Given the velocity V_EP of a frame P with respect to another frame E, and
/// the velocity V_PBq_E of a point Q on a frame B measured in frame P (both
/// expressed in frame E), the velocity of point Q on frame B measured and
/// expressed in frame E is obtained as:
/// <pre>
///   V_EBq = V_EP.Shift(p_PoBq_E) + V_PBq_E
/// </pre>
///
/// where `p_PoBq_E` is the position vector from P's origin to point Q. The
/// first term corresponds to the velocity of point Q as if instantaneously
/// (rigidly attached) moving with frame P. We deonte this spatial velocity with
/// `V_EPBq`. Therefore the above equation can be written more compactly as:
/// <pre>
///   V_EBq = V_EPBq + V_PBq_E
/// </pre>
///
/// The addition in the last expression is carried out by this operator.
template <typename T>
inline SpatialVelocity<T> operator+(
    const SpatialVelocity<T>& V_EPBq, const SpatialVelocity<T>& V_PBq_E) {
  return SpatialVelocity<T>(V_EPBq.get_coeffs() + V_PBq_E.get_coeffs());
}

}  // namespace multibody
}  // namespace drake
