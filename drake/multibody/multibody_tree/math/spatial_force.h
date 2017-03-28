#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

// Forward declaration to define dot product with a spatial velocity.
template <typename T> class SpatialVelocity;

/// This class is used to represent a _spatial force_ (also called a _wrench_)
/// that combines both rotational (torque) and translational force components.
/// Spatial forces are 6-element quantities that are pairs of ordinary
/// 3-vectors. Elements 0-2 are the torque component while elements 3-5 are the
/// force component.
/// Both vectors must be expressed in the same frame, and the translational
/// force is applied to a particular point of a body, but neither the frame nor
/// the point are stored with a %SpatialForce object; they must be understood
/// from context. It is the responsibility of the user to keep track of the
/// application point and the expressed-in frame. That is best accomplished
/// through disciplined notation. In source code we use monogram notation
/// where capital F is used to designate a spatial force quantity. We write
/// a point P fixed to body (or frame) B as @f$B_P@f$ which appears in
/// code and comments as `Bp`. Then we write a particular spatial force as
/// `F_Bp_E` where the `_E` suffix indicates that the expressed-in frame
/// is E. This symbol represents a torque applied to body B, and a force
/// applied to point P on B, with both vectors expressed in E. Very often
/// the application point will be the body origin `Bo`; if no point is
/// shown the origin is understood, so `F_B_E` means `F_Bo_E`.
/// For a more detailed introduction on spatial vectors and the monogram
/// notation please refer to section @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class SpatialForce : public SpatialVector<SpatialForce, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::SpatialForce, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialForce)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial force are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial force fail fast,
  /// allowing fast bug detection.
  SpatialForce() : Base() {}

  /// SpatialForce constructor from a torque `tau` and a force `f`.
  SpatialForce(const Eigen::Ref<const Vector3<T>>& tau,
               const Eigen::Ref<const Vector3<T>>& f) : Base(tau, f) {}

  /// SpatialForce constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of V is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit SpatialForce(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialForce from one application point to another.
  /// `this` spatial force `F_Bp_E`, which applies its translational force
  /// component to point P of body B, is modified to become the equivalent
  /// spatial force `F_Bq_E` that considers the force to be applied to point
  /// Q of body B instead (see class comment for more about this notation).
  /// This requires adjusting the torque component to account
  /// for the change in moment caused by the force shift.
  ///
  /// We are given the vector from point P to point Q, as a position vector
  /// `p_BpBq_E` (or `p_PQ_E`) expressed in the same frame E as the
  /// spatial force. The operation performed, in coordinate-free form, is:
  /// <pre>
  ///   τ_B  = τ_B -  p_BpBq x f_Bp
  ///   f_Bq = f_Bp,  i.e. the force as applied to body B at Q is the
  ///                 same as was applied to B at P.
  /// </pre>
  /// where τ and f represent the torque and force components respectively.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the
  ///   current application point of `this` spatial force, and E must be
  ///   the same expressed-in frame as for this spatial force.
  ///
  /// @returns A reference to `this` spatial force which is now `F_Bq_E`,
  ///          that is, the force is now applied at point Q rather than P.
  ///
  /// @see Shift() to compute the shifted spatial force without modifying
  ///              this original object.
  SpatialForce<T>& ShiftInPlace(const Vector3<T>& p_BpBq_E) {
    this->rotational() -= p_BpBq_E.cross(this->translational());
    return *this;
  }

  /// Shift of a %SpatialForce from one application point to another.
  /// This is an alternate signature for shifting a spatial force's
  /// application point that does not change the original object. See
  /// ShiftInPlace() for more information.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the
  ///   current application point of `this` spatial force, and E must be
  ///   the same expressed-in frame as for this spatial force.
  ///
  /// @retval F_Bq_E
  ///   The equivalent shifted spatial force, now applied at point Q
  ///   rather than P.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial force in-place
  ///                     modifying the original object.
  SpatialForce<T> Shift(const Vector3<T>& p_BpBq_E) const {
    return SpatialForce<T>(*this).ShiftInPlace(p_BpBq_E);
  }

  /// Given `this` spatial force `F_Bp_E` applied at point P of body B and
  /// expressed in a frame E, this method computes the 6-dimensional dot
  /// product with the spatial velocity `V_IBp_E` of body B at point P,
  /// measured in an inertial frame I and expressed in the same frame E
  /// in which the spatial force is expressed.
  /// This dot-product represents the power generated by `this` spatial force
  /// when its body and application point have the given spatial velocity.
  /// Although the two spatial vectors must be expressed in the same frame,
  /// the result is independent of that frame.
  ///
  /// @warning The result of this method cannot be interpreted as power unless
  ///          the spatial velocity is measured in an inertial frame I.
  T dot(const SpatialVelocity<T>& V_IBp_E) const;
};

}  // namespace multibody
}  // namespace drake
