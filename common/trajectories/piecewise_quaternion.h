#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace trajectories {

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
 * @tparam_default_scalars
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
   * @throws std::exception if breaks and quaternions have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<T>& breaks,
      const std::vector<Quaternion<T>>& quaternions);

  /**
   * Builds a PiecewiseQuaternionSlerp.
   * @throws std::exception if breaks and rot_matrices have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<T>& breaks,
      const std::vector<Matrix3<T>>& rotation_matrices);

  /**
   * Builds a PiecewiseQuaternionSlerp.
   * @throws std::exception if breaks and rot_matrices have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<T>& breaks,
      const std::vector<math::RotationMatrix<T>>& rotation_matrices);

  /**
   * Builds a PiecewiseQuaternionSlerp.
   * @throws std::exception if breaks and ang_axes have different length,
   * or breaks have length < 2.
   */
  PiecewiseQuaternionSlerp(
      const std::vector<T>& breaks,
      const std::vector<AngleAxis<T>>& angle_axes);

  ~PiecewiseQuaternionSlerp() override = default;

  std::unique_ptr<Trajectory<T>> Clone() const override;

  Eigen::Index rows() const override { return 4; }

  Eigen::Index cols() const override { return 1; }

  /**
   * Interpolates orientation.
   * @param time Time for interpolation.
   * @return The interpolated quaternion at `time`.
   */
  Quaternion<T> orientation(const T& time) const;

  MatrixX<T> value(const T& time) const override {
    return orientation(time).matrix();
  }

  /**
   * Interpolates angular velocity.
   * @param time Time for interpolation.
   * @return The interpolated angular velocity at `time`,
   * which is constant per segment.
   */
  Vector3<T> angular_velocity(const T& time) const;

  /**
   * Interpolates angular acceleration.
   * @param time Time for interpolation.
   * @return The interpolated angular acceleration at `time`,
   * which is always zero for slerp.
   */
  Vector3<T> angular_acceleration(const T& time) const;

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

  /**
   * Returns true if all the corresponding segment times are within
   * @p tol seconds, and the angle difference between the corresponding
   * quaternion sample points are within @p tol (using `ExtractDoubleOrThrow`).
   */
  bool is_approx(const PiecewiseQuaternionSlerp<T>& other,
                 double tol) const;

  /**
   * Given a new Quaternion, this method adds one segment to the end of `this`.
   */
  void Append(const T& time, const Quaternion<T>& quaternion);

  /**
   * Given a new RotationMatrix, this method adds one segment to the end of
   * `this`.
   */
  void Append(const T& time, const math::RotationMatrix<T>& rotation_matrix);

  /**
   * Given a new AngleAxis, this method adds one segment to the end of `this`.
   */
  void Append(const T& time, const AngleAxis<T>& angle_axis);

 private:
  // Initialize quaternions_ and computes angular velocity for each segment.
  void Initialize(
      const std::vector<T>& breaks,
      const std::vector<Quaternion<T>>& quaternions);

  // Computes the interpolation time within each segment. Result is in [0, 1].
  T ComputeInterpTime(int segment_index, const T& time) const;

  bool do_has_derivative() const override;

  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const override;

  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const override;

  std::vector<Quaternion<T>> quaternions_;
  std::vector<Vector3<T>> angular_velocities_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseQuaternionSlerp)
