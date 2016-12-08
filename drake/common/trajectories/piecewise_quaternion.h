#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_function.h"

namespace drake {

/**
 * A class representing a trajectory for quaternions that can be interpolated
 * using piecewise slerp (spherical linear interpolation).
 * All the quaternions, angular velocity, angular acceleration are assumed to
 * be in the same reference frame.
 */
template <typename Scalar = double>
class PiecewiseQuaternionSlerp : public PiecewiseFunction {
 public:
  PiecewiseQuaternionSlerp() {}

  /*
   * Build a PiecewiseQuaternionSlerp.
   * `quaternions` are set to the closest w.r.t the previous one internally.
   * @throws if breaks and quaternions have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(const std::vector<double>& breaks,
                           const std::vector<Quaternion<Scalar>>& quaternions);

  /*
   * Build a PiecewiseQuaternionSlerp.
   * The internal quaternion representation of `rot_matrices` are set to the
   * closest one w.r.t the previous quaternion.
   * @throws if breaks and rot_matrices have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(const std::vector<double>& breaks,
                           const std::vector<Matrix3<Scalar>>& rot_matrices);

  /*
   * Build a PiecewiseQuaternionSlerp.
   * The internal quaternion representation of `ang_axises` are set to the
   * closest one w.r.t the previous quaternion.
   * @throws if breaks and ang_axises have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(const std::vector<double>& breaks,
                           const std::vector<AngleAxis<Scalar>>& ang_axises);

  Eigen::Index rows() const override { return 4; }
  Eigen::Index cols() const override { return 1; }

  /**
   * @param t Time for interpolation.
   * @returns The interpolated quaternion at `t`.
   */
  Quaternion<Scalar> orientation(double t) const;

  /**
   * @param t Time for interpolation.
   * @returns The interpolated angular velocity at `t`,
   * which is constant per segment.
   */
  Vector3<Scalar> angular_velocity(double t) const;

  /**
   * @param t Time for interpolation.
   * @returns The interpolated angular acceleration at `t`,
   * which is always zero for slerp.
   */
  Vector3<Scalar> angular_acceleration(double t) const;

  /**
   * Note: the returned quaternions might be different from the ones that are
   * passed in during construction. This is because the internal quaternions
   * are always set to the "closest" w.r.t to the previous one.
   *
   * @returns the internal knot points.
   */
  const std::vector<Quaternion<Scalar>>& get_quaternion_knots() const {
    return quaternions_;
  }

 private:
  // Initialize quaternions_ and computes angular velocity for each segment.
  void Initialize(const std::vector<double>& breaks,
                  const std::vector<Quaternion<Scalar>>& quaternions);

  // Computes angular velocity for each segment.
  void ComputeAngularVelocities();

  // Computes the interpolation time within each segment. Result is in [0, 1].
  double ComputeInterpTime(int segment_index, double time) const;

  std::vector<Quaternion<Scalar>> quaternions_;
  std::vector<Vector3<Scalar>> angular_velocities_;
};

}  // namespace drake
