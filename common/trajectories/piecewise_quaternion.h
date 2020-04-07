#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_trajectory.h"

namespace drake {
namespace trajectories {

// TODO(siyuan.feng): check if this works for AutoDiffScalar.
/**
 * A class representing a trajectory for quaternions that are interpolated
 * using piecewise slerp (spherical linear interpolation).
 * All the orientation samples are expected to be with respect to the same
 * parent reference frame, i.e. q_i represents the rotation R_PBi for the
 * orientation of frame B at the ith sample in a fixed parent frame P.
 * The world frame is a common choice for the parent frame.
 * The angular velocity and acceleration are also relative to the parent frame
 * and expressed in the parent frame.
 * Since there is a sign ambiguity when using quaternions to represent
 * orientation, namely q and -q represent the same orientation, the internal
 * quaternion representations ensure that q_n.dot(q_{n+1}) >= 0.
 * Another intuitive way to think about this is that consecutive quaternions
 * have the shortest geodesic distance on the unit sphere.
 *
 * @tparam_double_only
 */
template<typename T>
class PiecewiseQuaternionSlerp final : public PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseQuaternionSlerp)

  /**
   * Builds an empty PiecewiseQuaternionSlerp.
   */
  PiecewiseQuaternionSlerp() = default;

  /**
   * Builds a PiecewiseQuaternionSlerp.
   * @throws std::logic_error if breaks and quaternions have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const std::vector<Quaternion<T>>& quaternions);

  /**
   * Builds a PiecewiseQuaternionSlerp.
   * @throws std::logic_error if breaks and rot_matrices have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const std::vector<Matrix3<T>>& rot_matrices);

  /**
   * Builds a PiecewiseQuaternionSlerp.
   * @throws std::logic_error if breaks and ang_axes have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const std::vector<AngleAxis<T>>& ang_axes);

  ~PiecewiseQuaternionSlerp() override = default;

  std::unique_ptr<Trajectory<T>> Clone() const override;

  Eigen::Index rows() const override { return 4; }

  Eigen::Index cols() const override { return 1; }

  /**
   * Interpolates orientation.
   * @param t Time for interpolation.
   * @return The interpolated quaternion at `t`.
   */
  Quaternion<T> orientation(double t) const;

  MatrixX<T> value(const T& t) const override {
    return orientation(t).matrix();
  }

  /**
   * Interpolates angular velocity.
   * @param t Time for interpolation.
   * @return The interpolated angular velocity at `t`,
   * which is constant per segment.
   */
  Vector3<T> angular_velocity(double t) const;

  /**
   * @throws std::runtime_error (always) because it is not implemented yet.
   */
  // TODO(russt): Implement this if/when someone needs it!
  std::unique_ptr<Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override;

  /**
   * Interpolates angular acceleration.
   * @param t Time for interpolation.
   * @return The interpolated angular acceleration at `t`,
   * which is always zero for slerp.
   */
  Vector3<T> angular_acceleration(double t) const;

  /**
   * Getter for the internal quaternion samples.
   *
   * @note The returned quaternions might be different from the ones used for
   * construction because the internal representations are set to always be
   * the "closest" w.r.t to the previous one.
   *
   * @return the internal sample points.
   */
  const std::vector<Quaternion<T>>& get_quaternion_samples() const {
    return quaternions_;
  }

  DRAKE_DEPRECATED(
      "2020-07-01",
      "get_quaternion_knots() is renamed get_quaternion_samples().")
  const std::vector<Quaternion<T>>& get_quaternion_knots() const {
    return quaternions_;
  }

  /**
   * Returns true if all the corresponding segment times are within
   * @p tol seconds, and the angle difference between the corresponding
   * quaternion sample points are within @p tol.
   */
  bool is_approx(const PiecewiseQuaternionSlerp<T>& other,
                 const T& tol) const;

 private:
  // Initialize quaternions_ and computes angular velocity for each segment.
  void Initialize(
      const std::vector<double>& breaks,
      const std::vector<Quaternion<T>>& quaternions);

  // Computes angular velocity for each segment.
  void ComputeAngularVelocities();

  // Computes the interpolation time within each segment. Result is in [0, 1].
  double ComputeInterpTime(int segment_index, double time) const;

  std::vector<Quaternion<T>> quaternions_;
  std::vector<Vector3<T>> angular_velocities_;
};

}  // namespace trajectories
}  // namespace drake
