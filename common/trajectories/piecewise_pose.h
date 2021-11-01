#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace trajectories {

/**
 * A wrapper class that represents a pose trajectory, whose rotation part is a
 * PiecewiseQuaternionSlerp and the translation part is a PiecewisePolynomial.
 *
 * @tparam_default_scalars
 */
template <typename T>
class PiecewisePose final : public PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewisePose)

  /**
   *  Constructs an empty piecewise pose trajectory.
   */
  PiecewisePose() {}

  /**
   * Constructor.
   * @param pos_traj Position trajectory.
   * @param rot_traj Orientation trajectory.
   */
  PiecewisePose(const PiecewisePolynomial<T>& position_trajectory,
                const PiecewiseQuaternionSlerp<T>& orientation_trajectory);

  /**
   * Constructs a PiecewisePose from given @p times and @p poses. The positions
   * trajectory is constructed as a first-order hold. The orientation is
   * constructed using the quaternion slerp.  There must be at least two
   * elements in @p times and @p poses.
   * @param times     Breaks used to build the splines.
   * @param poses     Knots used to build the splines.
   */
  static PiecewisePose<T> MakeLinear(
      const std::vector<T>& times,
      const std::vector<math::RigidTransform<T>>& poses);

  /**
   * Constructs a PiecewisePose from given @p times and @p poses.
   * A cubic polynomial with given end velocities is used to construct the
   * position part. The rotational part is represented by a piecewise quaterion
   * trajectory.  There must be at least two elements in @p times and @p poses.
   * @param times     Breaks used to build the splines.
   * @param poses     Knots used to build the splines.
   * @param start_vel Start linear velocity.
   * @param end_vel   End linear velocity.
   */
  static PiecewisePose<T> MakeCubicLinearWithEndLinearVelocity(
      const std::vector<T>& times,
      const std::vector<math::RigidTransform<T>>& poses,
      const Vector3<T>& start_vel = Vector3<T>::Zero(),
      const Vector3<T>& end_vel = Vector3<T>::Zero());

  std::unique_ptr<Trajectory<T>> Clone() const override;

  Eigen::Index rows() const override { return 4; }

  Eigen::Index cols() const override { return 4; }

  DRAKE_DEPRECATED("2022-01-01", "get_pose() has been renamed to GetPose().")
  math::RigidTransform<T> get_pose(const T& time) const {
    return GetPose(time);
  }

  /**
   * Returns the interpolated pose at @p time.
   */
  math::RigidTransform<T> GetPose(const T& time) const;

  MatrixX<T> value(const T& t) const override {
    return GetPose(t).GetAsMatrix4();
  }

  DRAKE_DEPRECATED("2022-01-01",
                   "get_velocity() has been renamed to GetVelocity().")
  Vector6<T> get_velocity(const T& time) const {
    return GetVelocity(time);
  }

  /**
   * Returns the interpolated velocity at @p time or zero if @p time is before
   * this trajectory's start time or after its end time.
   */
  Vector6<T> GetVelocity(const T& time) const;

  DRAKE_DEPRECATED("2022-01-01",
                   "get_acceleration() has been renamed to GetAcceleration().")
  Vector6<T> get_acceleration(const T& time) const {
    return GetAcceleration(time);
  }

  /**
   * Returns the interpolated acceleration at @p time or zero if @p time is
   * before this trajectory's start time or after its end time.
   */
  Vector6<T> GetAcceleration(const T& time) const;

  DRAKE_DEPRECATED("2022-01-01",
                   "is_approx() has been renamed to IsApprox().")
  bool is_approx(const PiecewisePose<T>& other, double tol) const {
    return IsApprox(other, tol);
  }

  /**
   * Returns true if the position and orientation trajectories are both
   * within @p tol from the other's.
   */
  bool IsApprox(const PiecewisePose<T>& other, double tol) const;

  /**
   * Returns the position trajectory.
   */
  const PiecewisePolynomial<T>& get_position_trajectory() const {
    return position_;
  }

  /**
   * Returns the orientation trajectory.
   */
  const PiecewiseQuaternionSlerp<T>& get_orientation_trajectory() const {
    return orientation_;
  }

 private:
  bool do_has_derivative() const override;

  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const override;

  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const override;

  PiecewisePolynomial<T> position_;
  PiecewisePolynomial<T> velocity_;
  PiecewisePolynomial<T> acceleration_;
  PiecewiseQuaternionSlerp<T> orientation_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewisePose)
