#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace trajectories {

/**
 * Represents a piecewise constant-curvature trajectory within a plane embdedded
 * in 3D space.
 *
 * This class defines a trajectory composed of segments with constant curvature:
 * circular arcs and line segments. It provides methods to calcualte poses,
 * velocities, and accelerations along the trajectory based on the distance
 * traveled.
 *
 * The curve is path-length paramaterized; as such the linear velocity is always
 * a unit vector.
 *
 * @tparam_default_scalar
 */
template <typename T>
class PiecewiseConstantCurvatureTrajectory final
    : public PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseConstantCurvatureTrajectory);

  /**
   * Constructs an empty piecewise constant curvature trajectory.
   */
  PiecewiseConstantCurvatureTrajectory() {}

  /**
   * Clones this trajectory to a trajectory with a different scalar type.
   *
   * @tparam ToScalar The scalar type to which to clone the trajectory.
   * @return The cloned trajectory.
   */
  template <typename ToScalar>
  PiecewiseConstantCurvatureTrajectory<ToScalar> CloneToScalar() const {
    std::vector<ToScalar> segment_curvatures;
    std::vector<ToScalar> segment_breaks;
    systems::scalar_conversion::ValueConverter<ToScalar, T> converter;
    for (const T& curvature : segment_curvatures_) {
      segment_curvatures.push_back(converter(curvature));
    }
    for (const T& segment_break : this->breaks()) {
      segment_breaks.push_back(converter(segment_break));
    }
    return PiecewiseConstantCurvatureTrajectory<ToScalar>(
        segment_breaks, segment_curvatures,
        base_frame_.col(kCurveTangentDimension).template cast<ToScalar>(),
        base_frame_.col(kPlaneNormalDimension).template cast<ToScalar>());
  }

  /**
   * Constructs a piecewise constant curvature trajectory.
   *
   * @param segment_breaks A vector of distance break points between segments.
   * @param segment_curvatures A vector of curvatures for each segment.
   * @param curve_tangent_axis The tangent of the curve at path length 0.
   * @param plane_normal_axis The normal axis of the 2D plane in which the curve
   * lies.
   */
  PiecewiseConstantCurvatureTrajectory(const std::vector<T>& segment_breaks,
                                       const std::vector<T>& segment_curvatures,
                                       const Vector3<T>& curve_tangent_axis,
                                       const Vector3<T>& plane_normal_axis);

  /**
   * @return The number of rows in the trajectory's output.
   */
  Eigen::Index rows() const override { return 4; }

  /**
   * @return The number of columns in the trajectory's output.
   */
  Eigen::Index cols() const override { return 4; }

  /**
   * Returns the interpolated pose at the given distance along the trajectory.
   *
   * @param distance The distance along the trajectory.
   * @return The interpolated pose as a RigidTransform.
   */
  math::RigidTransform<T> GetPose(const T& distance) const;

  /**
   * Returns the interpolated pose at the given distance along the trajectory as
   * an Eigen matrix.
   *
   * @param t The distance along the trajectory.
   * @return The 4x4 homogeneous transform matrix representing the pose.
   */
  MatrixX<T> value(const T& t) const override {
    return GetPose(t).GetAsMatrix4();
  }

  /**
   * Creates a deep copy of this trajectory.
   *
   * @return A unique pointer to the cloned trajectory.
   */
  std::unique_ptr<Trajectory<T>> Clone() const override;

  /**
   * Returns the interpolated spatial velocity at the given distance along the
   * trajectory.
   *
   * @param distance The distance along the trajectory.
   * @return The interpolated spatial velocity.
   */
  multibody::SpatialVelocity<T> GetVelocity(const T& distance) const;

  /**
   * Returns the interpolated spatial acceleration at the given distance along
   * the trajectory.
   *
   * @param distance The distance along the trajectory.
   * @return The interpolated spatial acceleration.
   */
  multibody::SpatialAcceleration<T> GetAcceleration(const T& distance) const;

  /**
   * Checks if the trajectory is nearly periodic within the given tolerance.
   *
   * Periodicity is defined as the beginning and end poses being equal up to the
   * same tolerance.
   *
   * @param tolerance The tolerance for periodicity check.
   * @return True if the trajectory is nearly periodic, false otherwise.
   */
  boolean<T> IsNearlyPeriodic(double tolerance) const;

  /**
   * Returns the frame of the trajectory at distance zero, which defines
   * the orientation of the plane in the embedded 3D space.
   *
   * @return The base frame as a RotationMatrix.
   */
  const math::RotationMatrix<T>& get_base_frame() const { return base_frame_; }

  /**
   * Returns the normal axis of the plane in which the curve lies
   *
   * @return The plane normal axis.
   */
  const Eigen::Ref<const Vector3<T>> get_plane_normal_axis() const {
    return base_frame_.col(kPlaneNormalDimension);
  }

 private:
  /**
   * Calculates the relative transform between the start of a constant-curavtue
   * segment and the pose at a given length of travel within the segment.
   *
   * @param curvature The curvature of the segment.
   * @param length The length of travel in the segment.
   * @return The pose relative to length 0 along the curve.
   */
  static math::RigidTransform<T> CalcPoseInLocalFrame(const T& curvature,
                                                      const T& length);

  /**
   * Builds the base frame from the given tangent and plane axes.
   *
   * @param curve_tangent_axis The tangent of the curve at distance 0.
   * @param plane_normal_axis The normal axis of the plane.
   *
   * @return The base frame as a RotationMatrix.
   */
  static math::RotationMatrix<T> MakeBaseFrame(
      const Vector3<T>& curve_tangent_axis,
      const Vector3<T>& plane_normal_axis);

  /**
   * Builds the curve from the given curvatures and segment breaks.
   *
   * @param base_frame The base frame of the trajectory.
   * @param segment_curvatures A vector of curvatures for each segment.
   * @param segment_breaks A vector of break points between segments.
   *
   * @return The local frames of each segment.
   */
  static std::vector<math::RigidTransform<T>> MakeSegmentLocalFrames(
      const math::RotationMatrix<T>& base_frame,
      const std::vector<T>& segment_curvatures,
      const std::vector<T>& segment_breaks);

  math::RotationMatrix<T> base_frame_;
  std::vector<T> segment_curvatures_;
  std::vector<math::RigidTransform<T>> segment_local_frames_;

  static inline constexpr size_t kCurveTangentDimension = 0;
  static inline constexpr size_t kCurveNormalDimension = 1;
  static inline constexpr size_t kPlaneNormalDimension = 2;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseConstantCurvatureTrajectory);
