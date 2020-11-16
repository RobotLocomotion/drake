#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace trajectories {

/**
 * A wrapper class that represents a pose trajectory, whose rotation part is a
 * PiecewiseQuaternionSlerp and the translation part is a PiecewisePolynomial.
 */
template <typename T>
class PiecewisePoseTrajectory {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewisePoseTrajectory)

  /**
   *  Constructs an empty piecewise pose trajectory.
   */
  PiecewisePoseTrajectory() {}

  /**
   * Constructs a PiecewisePoseTrajectory from given @p time and @p poses.
   * A cubic polynomial with zero end velocities is used to construct the
   * position part. There must be at least two elements in @p times and
   * @p poses.
   * @param times Breaks used to build the splines.
   * @param poses Knots used to build the splines.
   * @param vel0 Start linear velocity.
   * @param vel1 End linear velocity.
   */
  static PiecewisePoseTrajectory<T> MakeCubicLinearWithEndLinearVelocity(
      const std::vector<T>& times,
      const std::vector<math::RigidTransform<T>>& poses, const Vector3<T>& vel0,
      const Vector3<T>& vel1) {
    std::vector<MatrixX<T>> pos_knots(poses.size());
    std::vector<math::RotationMatrix<T>> rot_knots(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      pos_knots[i] = poses[i].translation();
      rot_knots[i] = poses[i].rotation();
    }

    return PiecewisePoseTrajectory(
        PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
            times, pos_knots, vel0, vel1),
        PiecewiseQuaternionSlerp<T>(times, rot_knots));
  }

  /**
   * Constructor.
   * @param pos_traj Position trajectory.
   * @param rot_traj Orientation trajectory.
   */
  PiecewisePoseTrajectory(const PiecewisePolynomial<T>& pos_traj,
                          const PiecewiseQuaternionSlerp<T>& rot_traj) {
    DRAKE_DEMAND(pos_traj.rows() == 3);
    DRAKE_DEMAND(pos_traj.cols() == 1);
    position_ = pos_traj;
    velocity_ = position_.derivative();
    acceleration_ = velocity_.derivative();
    orientation_ = rot_traj;
  }

  /**
   * Returns the interpolated pose at @p time.
   */
  math::RigidTransform<T> get_pose(double time) const {
    return math::RigidTransform<T>(orientation_.orientation(time),
                                   position_.value(time));
  }

  /**
   * Returns the interpolated velocity at @p time or zero if @p time is before
   * this trajectory's start time or after its end time.
   */
  Vector6<T> get_velocity(double time) const {
    Vector6<T> velocity;
    if (orientation_.is_time_in_range(time)) {
      velocity.template head<3>() = orientation_.angular_velocity(time);
    } else {
      velocity.template head<3>().setZero();
    }
    if (position_.is_time_in_range(time)) {
      velocity.template tail<3>() = velocity_.value(time);
    } else {
      velocity.template tail<3>().setZero();
    }
    return velocity;
  }

  /**
   * Returns the interpolated acceleration at @p time or zero if @p time is
   * before this trajectory's start time or after its end time.
   */
  Vector6<T> get_acceleration(double time) const {
    Vector6<T> acceleration;
    if (orientation_.is_time_in_range(time)) {
      acceleration.template head<3>() = orientation_.angular_acceleration(time);
    } else {
      acceleration.template head<3>().setZero();
    }
    if (position_.is_time_in_range(time)) {
      acceleration.template tail<3>() = acceleration_.value(time);
    } else {
      acceleration.template tail<3>().setZero();
    }
    return acceleration;
  }

  /**
   * Returns true if the position and orientation trajectories are both
   * within @p tol from the other's.
   */
  bool is_approx(const PiecewisePoseTrajectory<T>& other, const T& tol) const {
    bool ret = position_.isApprox(other.position_, tol);
    ret &= velocity_.isApprox(other.velocity_, tol);
    ret &= acceleration_.isApprox(other.acceleration_, tol);
    ret &= orientation_.is_approx(other.orientation_, tol);
    return ret;
  }

  /**
   * Returns the position trajectory.
   */
  const PiecewisePolynomial<T>& get_position_trajectory() const {
    return position_;
  }

  /**
   * Returns the orientation trajectory.
   */
  const PiecewiseQuaternionSlerp<T>& get_orientation_trajectory()
      const {
    return orientation_;
  }

 private:
  PiecewisePolynomial<T> position_;
  PiecewisePolynomial<T> velocity_;
  PiecewisePolynomial<T> acceleration_;
  PiecewiseQuaternionSlerp<T> orientation_;
};

}  // namespace trajectories
}  // namespace drake
