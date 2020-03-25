#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace manipulation {

/**
 * A wrapper class that stores a PiecewisePolynomial and its first and second
 * derivatives. This class is supposed to represent position, velocity and
 * acceleration. Thus, when the interpolating time is beyond the time bounds,
 * the interpolated velocity and acceleration will be set to zero, and the
 * interpolated position will peg at the terminal values. All dimensions are
 * assumed to be independent of each other.
 */
template <typename T>
class PiecewiseCubicTrajectory {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseCubicTrajectory)

  PiecewiseCubicTrajectory() {}

  /**
   * Constructor.
   * @param position_traj PiecewisePolynomial that represents the position
   * trajectory. Its first and second derivatives are computed and stored.
   */
  explicit PiecewiseCubicTrajectory(
      const trajectories::PiecewisePolynomial<T>& position_traj) {
    q_ = position_traj;
    qd_ = q_.derivative();
    qdd_ = qd_.derivative();
  }

  /**
   * Returns the interpolated position at @p time.
   */
  MatrixX<T> get_position(double time) const { return q_.value(time); }

  /**
   * Returns the interpolated velocity at @p time or zero if @p time is out of
   * the time bounds.
   */
  MatrixX<T> get_velocity(double time) const {
    MatrixX<T> ret = qd_.value(time);
    if (!q_.is_time_in_range(time)) ret.setZero();
    return ret;
  }

  /**
   * Returns the interpolated acceleration at @p time or zero if @p time is out
   * of the time bounds.
   */
  MatrixX<T> get_acceleration(double time) const {
    MatrixX<T> ret = qdd_.value(time);
    if (!q_.is_time_in_range(time)) ret.setZero();
    return ret;
  }

  /**
   * Returns the start time of this trajectory.
   */
  double get_start_time() const { return q_.start_time(); }

  /**
   * Returns the end time of this trajectory.
   */
  double get_end_time() const { return q_.end_time(); }

  /**
   * Returns true if the position trajectory and its first and second
   * derivatives are all within @p tol to @p other.
   */
  bool is_approx(const PiecewiseCubicTrajectory<T>& other, const T& tol) const {
    bool ret = q_.isApprox(other.q_, tol);
    ret &= qd_.isApprox(other.qd_, tol);
    ret &= qdd_.isApprox(other.qdd_, tol);
    return ret;
  }

  /**
   * Returns the position trajectory.
   */
  const trajectories::PiecewisePolynomial<T>& get_position_trajectory() const {
    return q_;
  }

  /**
   * Returns the velocity trajectory (first derivative).
   */
  const trajectories::PiecewisePolynomial<T>& get_velocity_trajectory() const {
    return qd_;
  }

  /**
   * Returns the acceleration trajectory (second derivative).
   */
  const trajectories::PiecewisePolynomial<T>& get_acceleration_trajectory()
      const {
    return qdd_;
  }

 private:
  trajectories::PiecewisePolynomial<T> q_;
  trajectories::PiecewisePolynomial<T> qd_;
  trajectories::PiecewisePolynomial<T> qdd_;
};

/**
 * A wrapper class that represents a Cartesian trajectory, whose position part
 * is a PiecewiseCubicTrajectory, and the rotation part is a
 * PiecewiseQuaternionSlerp.
 */
template <typename T>
class PiecewiseCartesianTrajectory {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseCartesianTrajectory)

  PiecewiseCartesianTrajectory() {}

  /**
   * Constructs a PiecewiseCartesianTrajectory from given @p time and @p poses.
   * A cubic polynomial with zero end velocities is used to construct the
   * position part. There must be at least two elements in @p times and
   * @p poses.
   * @param times Breaks used to build the splines.
   * @param poses Knots used to build the splines.
   * @param vel0 Start linear velocity.
   * @param vel1 End linear velocity.
   */
  static PiecewiseCartesianTrajectory<T> MakeCubicLinearWithEndLinearVelocity(
      const std::vector<T>& times, const std::vector<Isometry3<T>>& poses,
      const Vector3<T>& vel0, const Vector3<T>& vel1) {
    std::vector<MatrixX<T>> pos_knots(poses.size());
    std::vector<Matrix3<T>> rot_knots(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      pos_knots[i] = poses[i].translation();
      rot_knots[i] = poses[i].linear();
    }

    return PiecewiseCartesianTrajectory(
        trajectories::PiecewisePolynomial<
            T>::CubicWithContinuousSecondDerivatives(times, pos_knots, vel0,
                                                     vel1),
        trajectories::PiecewiseQuaternionSlerp<T>(times, rot_knots));
  }

  /**
   * Constructor.
   * @param pos_traj Position trajectory.
   * @param rot_traj Orientation trajectory.
   */
  PiecewiseCartesianTrajectory(
      const trajectories::PiecewisePolynomial<T>& pos_traj,
      const trajectories::PiecewiseQuaternionSlerp<T>& rot_traj)
      : PiecewiseCartesianTrajectory(PiecewiseCubicTrajectory<T>(pos_traj),
                                     rot_traj) {}

  /**
   * Constructor.
   * @param pos_traj Position trajectory.
   * @param rot_traj Orientation trajectory.
   */
  PiecewiseCartesianTrajectory(
      const PiecewiseCubicTrajectory<T>& pos_traj,
      const trajectories::PiecewiseQuaternionSlerp<T>& rot_traj) {
    DRAKE_DEMAND(pos_traj.get_position_trajectory().rows() == 3);
    DRAKE_DEMAND(pos_traj.get_position_trajectory().cols() == 1);
    position_ = pos_traj;
    orientation_ = rot_traj;
  }

  /**
   * Returns the interpolated pose at @p time.
   */
  Isometry3<T> get_pose(double time) const {
    Isometry3<T> pose;
    pose.fromPositionOrientationScale(
        position_.get_position(time),
        orientation_.orientation(time).toRotationMatrix(),
        Vector3<double>::Ones());
    return pose;
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
    velocity.template tail<3>() = position_.get_velocity(time);
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
    acceleration.template tail<3>() = position_.get_acceleration(time);
    return acceleration;
  }

  /**
   * Returns true if the position and orientation trajectories are both
   * within @p tol from the other's.
   */
  bool is_approx(const PiecewiseCartesianTrajectory<T>& other,
                 const T& tol) const {
    bool ret = position_.is_approx(other.position_, tol);
    ret &= orientation_.is_approx(other.orientation_, tol);
    return ret;
  }

  /**
   * Returns the position trajectory.
   */
  const PiecewiseCubicTrajectory<T>& get_position_trajectory() const {
    return position_;
  }

  /**
   * Returns the orientation trajectory.
   */
  const trajectories::PiecewiseQuaternionSlerp<T>& get_orientation_trajectory()
      const {
    return orientation_;
  }

 private:
  PiecewiseCubicTrajectory<T> position_;
  trajectories::PiecewiseQuaternionSlerp<T> orientation_;
};

}  // namespace manipulation
}  // namespace drake
