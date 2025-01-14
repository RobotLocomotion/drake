#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
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
 * @tparam_default_scalar
 */
template <typename T>
class PiecewisePose final : public PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewisePose);

  /**
   *  Constructs an empty piecewise pose trajectory.
   */
  PiecewisePose() {}

  /**
   * Constructor.
   * @param position_trajectory Position trajectory.
   * @param orientation_trajectory Orientation trajectory.
   */
  PiecewisePose(const PiecewisePolynomial<T>& position_trajectory,
                const PiecewiseQuaternionSlerp<T>& orientation_trajectory);

  ~PiecewisePose() final;

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

  /**
   * Returns the interpolated pose at @p time.
   */
  math::RigidTransform<T> GetPose(const T& time) const;

  /**
   * Returns the interpolated velocity at @p time or zero if @p time is before
   * this trajectory's start time or after its end time.
   */
  Vector6<T> GetVelocity(const T& time) const;

  /**
   * Returns the interpolated acceleration at @p time or zero if @p time is
   * before this trajectory's start time or after its end time.
   */
  Vector6<T> GetAcceleration(const T& time) const;

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
  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final {
    return GetPose(t).GetAsMatrix4();
  }
  bool do_has_derivative() const final;
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;
  Eigen::Index do_rows() const final { return 4; }
  Eigen::Index do_cols() const final { return 4; }

  PiecewisePolynomial<T> position_;
  PiecewisePolynomial<T> velocity_;
  PiecewisePolynomial<T> acceleration_;
  PiecewiseQuaternionSlerp<T> orientation_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewisePose);
