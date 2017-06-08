#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/** This class is used to represent the pose of a frame, `X_PF` --  the position
 and orientation of a frame F, relative to a parent frame P--used, for example,
 as the basis for communicating frame kinematics values to GeometryWorld and
 GeometrySystem via the FrameKinematicsSet.

 The %SpatialPose is _related_ to a SpatialVelocity. Whereas the SpatialVelocity
 `V_PF` represents the rate at which frame F moves with respect to frame P, the
 %SpatialPose `X_PF` represents the pose of frame F measured in frame P. It is
 worth emphasizing that `V_PF` ≠ `d(X_PF)/dt` on an element-wise basis. See
 @ref multibody_concepts for a full discussion.

 The %SpatialPose has no knowledge of its semantics (most particularly which
 frame it is associated with or which frame it is measured in). It assumes the
 context of its definition and use can maintain those associations consistently.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 @see FrameKinematicsSet */
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

  /** Construction from the orientation `q_PF_E` and a position `p_PF_E` of
   frame F relative to parent frame P. Both quantities must be expressed in the
   same frame E. */
  SpatialPose(const Quaternion<T>& q_PF_E,
              const Eigen::Ref<const Vector3<T>>& p_PF_E)
      : orientation_(q_PF_E), position_(p_PF_E) {}

  /** Construction from the pose `X_PF` represented by an Eigen::Isometry. */
  explicit SpatialPose(const Isometry3<T>& X_PF)
      : orientation_(X_PF.linear()), position_(X_PF.translation()) {}

  //@}

  /** @name  Const access to pose values

   The pose values returned by these methods are implicitly measured and
   expressed in the same frame as the user-provided values (either in the
   constructor or through the setter methods). %SpatialPose will take no action
   to re-express the values in any other frame.  Therefore, if the user-provided
   values are for frame F relative to frame P, these functions will return
   the quantity `q_PF` for the corresponding quantity. */
  //@{

  /** Returns the full pose represented by an Eigen::Isometry. */
  Isometry3<T> get_isometry() const {
    Isometry3<T> isometry;
    isometry.linear() = orientation_.toRotationMatrix();
    isometry.translation() = translational();
    return isometry;
  }

  /** Returns the rotational component of this pose. */
  const Quaternion<T>& rotational() const {
    return orientation_;
  }

  /** Returns the translational component of this pose. */
  const Vector3<T>& translational() const {
    return position_;
  }

  //@}

  /** @name  Setting pose values

   It is the responsibility of the caller to confirm that the values provided
   to these methods are meaningful. If provided separately, the rotational and
   translational components must both be measured and expressed relative to the
   same frame. There is no error checking in this regard.  */
  // @{

  /** Sets the rotational component of the pose from the given quaternion. */
  void set_rotational(const Quaternion<T>& rotational) {
    orientation_ = rotational;
  }

  /** Sets the translational component of the pose from the given vector. */
  void set_translational(const Vector3<T>& position) {
    position_ = position;
  }

  /** Sets the full pose from the given isometry. */
  void set_pose(const Isometry3<T>& pose) {
    orientation_ = pose.linear();
    position_ = pose.translation();
  }

  //@}

 private:
  // Sets all entries in `this` %SpatialPose to NaN. Typically used to quickly
  // detect uninitialized values since NaN will trigger a chain of invalid
  // computations that can then be tracked back to the source.
  void SetNaN() {
    orientation_.coeffs().setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
    position_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  Quaternion<T> orientation_;
  Vector3<T> position_;
};
}  // namespace geometry
}  // namespace drake
