#pragma once

#include <vector>

#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_function.h"

namespace drake {

/**
 * A class representing a trajectory for quaternions that are interpolated
 * using piecewise slerp (spherical linear interpolation).
 * All the orientation knots are expected to be with respect to the same
 * parent reference frame, i.e. q_i represents the rotation R_PBi for the
 * orientation of frame B at the ith knot in a fixed parent frame P.
 * The world frame is a common choice for the parent frame.
 * The angular velocity and acceleration are also relative to the parent frame
 * and expressed in the parent frame.
 * Since there is a sign ambiguity when using quaternions to represent
 * orientation, namely q and -q represent the same orientation, the internal
 * quaternion representations ensure that q_n.dot(q_{n+1}) >= 0.
 * Another intuitive way to think about this is that consecutive quaternions
 * have the shortest geodesic distance on the unit sphere.
 *
 * @tparam Scalar, double.
 *
 */
// TODO(siyuan.feng): check if this works for AutoDiffScalar.
template <typename Scalar = double>
class PiecewiseQuaternionSlerp : public PiecewiseFunction {
 public:
  PiecewiseQuaternionSlerp() {}

  /*
   * Builds a PiecewiseQuaternionSlerp.
   * @throw if breaks and quaternions have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<Quaternion<Scalar>>& quaternions);

  /*
   * Builds a PiecewiseQuaternionSlerp.
   * @throw if breaks and rot_matrices have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<Matrix3<Scalar>>& rot_matrices);

  /*
   * Builds a PiecewiseQuaternionSlerp.
   * @throw if breaks and ang_axes have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<AngleAxis<Scalar>>& ang_axes);

  Eigen::Index rows() const override { return 4; }
  Eigen::Index cols() const override { return 1; }

  /**
   * Interpolates orientation.
   * @param t Time for interpolation.
   * @return The interpolated quaternion at `t`.
   */
  Quaternion<Scalar> orientation(double t) const;

  /**
   * Interpolates angular velocity.
   * @param t Time for interpolation.
   * @return The interpolated angular velocity at `t`,
   * which is constant per segment.
   */
  Vector3<Scalar> angular_velocity(double t) const;

  /**
   * Interpolates angular acceleration.
   * @param t Time for interpolation.
   * @return The interpolated angular acceleration at `t`,
   * which is always zero for slerp.
   */
  Vector3<Scalar> angular_acceleration(double t) const;

  /**
   * Getter for the internal quaternion knots.
   * Note: the returned quaternions might be different from the ones used for
   * construction because the internal representations are set to always be
   * the "closest" w.r.t to the previous one.
   *
   * @return the internal knot points.
   */
  const eigen_aligned_std_vector<Quaternion<Scalar>>& get_quaternion_knots()
      const {
    return quaternions_;
  }

 private:
  // Initialize quaternions_ and computes angular velocity for each segment.
  void Initialize(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<Quaternion<Scalar>>& quaternions);

  // Computes angular velocity for each segment.
  void ComputeAngularVelocities();

  // Computes the interpolation time within each segment. Result is in [0, 1].
  double ComputeInterpTime(int segment_index, double time) const;

  eigen_aligned_std_vector<Quaternion<Scalar>> quaternions_;
  eigen_aligned_std_vector<Vector3<Scalar>> angular_velocities_;
};

}  // namespace drake
