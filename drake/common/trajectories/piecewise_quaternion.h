#pragma once

#include <vector>

#include "drake/common/eigen_stl_types.h"
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
   * @param enforce_closest If true, the internal representations of
   * `quaternions` are set to have the shortest geodesic distance on the unit
   * sphere relative to the previous ones.
   * @throws if breaks and quaternions have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<Quaternion<Scalar>>& quaternions,
      bool enforce_closest = true);

  /*
   * Build a PiecewiseQuaternionSlerp.
   * The internal quaternion representations of `rot_matrices` are set to have
   * the shortest geodesic distance on the unit sphere relative to the previous
   * ones.
   * @throws if breaks and rot_matrices have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<Matrix3<Scalar>>& rot_matrices);

  /*
   * Build a PiecewiseQuaternionSlerp.
   * @param enforce_closest If true, the internal representations of
   * `quaternions` are set to have the shortest geodesic distance on the unit
   * sphere relative to the previous ones.
   * @throws if breaks and ang_axises have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<AngleAxis<Scalar>>& ang_axises,
      bool enforce_closest = true);

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
   * passed in during construction because the internal representations can be
   * set to always be the "closest" w.r.t to the previous one.
   *
   * @returns the internal knot points.
   */
  const eigen_aligned_std_vector<Quaternion<Scalar>>& get_quaternion_knots()
      const {
    return quaternions_;
  }

  /**
   * @returns true if the internal representations always have the shortest
   * geodesic distance on the unit sphere w.r.t the previous one.
   */
  inline bool is_enforcing_closest() const { return is_enforcing_closest_; }

 private:
  // Initialize quaternions_ and computes angular velocity for each segment.
  void Initialize(
      const std::vector<double>& breaks,
      const eigen_aligned_std_vector<Quaternion<Scalar>>& quaternions,
      bool enforce_closest);

  // Computes angular velocity for each segment.
  void ComputeAngularVelocities();

  // Computes the interpolation time within each segment. Result is in [0, 1].
  double ComputeInterpTime(int segment_index, double time) const;

  eigen_aligned_std_vector<Quaternion<Scalar>> quaternions_;
  eigen_aligned_std_vector<Vector3<Scalar>> angular_velocities_;

  bool is_enforcing_closest_;
};

}  // namespace drake
