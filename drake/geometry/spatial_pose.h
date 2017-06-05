#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/** This class is used to represent the pose of a frame, `X_PF`. It represents
 the position and orientation of a frame F, relative to another frame P. It is
 *related* to spatial vectors (e.g., SpatialVelocity), but it differs in several
 ways:
    - It serves as value storage and supports no mathematical operators.
    - It doesn't expose a monolithic array of coefficients but represents the
      pose as discrete orientation and position components.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class SpatialPose {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialPose)

  /** @name     Constructors  */
  // @{

  /** Default constructor. In Release builds the elements of the newly
   constructed spatial pose are left uninitialized resulting in a zero
   cost operation. However in Debug builds those entries are set to NaN so
   that operations using this uninitialized spatial vector fail fast,
   facilitating fast bug detection. */
  SpatialPose() {
    DRAKE_ASSERT_VOID(SetNaN());
  }

  /** Construction from an orientation `q` and a position `p`. */
  SpatialPose(const Quaternion<T>& q, const Eigen::Ref<const Vector3<T>>& p)
      : orientation_(q), position_(p) {}

  /** Construction from an isometry. */
  explicit SpatialPose(const Isometry3<T>& isometry)
      : orientation_(isometry.linear()), position_(isometry.translation()) {}

  //@}

  /** @name  Const access to pose values */
  //@{

  /** Returns the pose represented by a 4x4 homogeneous matrix (or isometry).
   The matrix is measured and expressed in the same frame as the underlying
   pose. */
  Isometry3<T> get_isometry() const {
    Isometry3<T> isometry;
    isometry.linear() = orientation_.toRotationMatrix();
    isometry.translation() = translational();
    return isometry;
  }

  /** Const access to the rotational component of this spatial vector. */
  const Quaternion<T>& rotational() const {
    return orientation_;
  }

  /** Const access to the translational component of this spatial vector. */
  const Vector3<T>& translational() const {
    return position_;
  }

  //@}

  /** @name  Updating pose values  */
  // @{

  /** Sets the rotational component of the pose from the given `rotational`. */
  void set_rotational(const Quaternion<T>& rotational) {
    orientation_ = rotational;
  }

  /** Sets the translational component of the pose from the given `position`. */
  void set_translational(const Vector3<T>& position) {
    position_ = position;
  }

  /** Sets the pose from the given isometry. */
  void set_pose(const Isometry3<T>& pose) {
    orientation_ = pose.linear();
    position_ = pose.translation();
  }

  //@}

  /** @name  Data access and manipulation  */
  // @{

  /** Compares `this` spatial pose to the provided spatial pose `other`
   with respect to a specified precision.
   @returns `true` if `other`'s values lie within a precision given by
   `tolerance`. The comparison is performed by comparing the translational
   components and rotational components of `this` and `other`
   using the fuzzy comparison provided by Eigen's method isApprox(). */
  bool IsApprox(const SpatialPose& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) const {
    return position_.isApprox(other.position_, tolerance) &&
        orientation_.isApprox(other.orientation_, tolerance);
  }

  /** Sets all entries in `this` %SpatialPose to NaN. Typically used to
   quickly detect uninitialized values since NaN will trigger a chain of
   invalid computations that can then be tracked back to the source. */
  void SetNaN() {
    orientation_.coeffs().setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
    position_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  //@}

 private:
  Quaternion<T> orientation_;
  Vector3<T> position_;
};
}  // namespace geometry
}  // namespace drake
