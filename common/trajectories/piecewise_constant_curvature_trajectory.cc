#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"

#include <functional>
#include <limits>

namespace drake {
namespace trajectories {

template <typename T>
PiecewiseConstantCurvatureTrajectory<T>::PiecewiseConstantCurvatureTrajectory(
    const std::vector<T>& segment_breaks,
    const std::vector<T>& segment_curvatures,
    const Vector3<T>& curve_tangent_axis, const Vector3<T>& plane_normal_axis)
    : PiecewiseTrajectory<T>(segment_breaks),
      base_frame_(MakeBaseFrame(curve_tangent_axis, plane_normal_axis)),
      segment_curvatures_(segment_curvatures),
      segment_local_frames_(MakeSegmentLocalFrames(
          base_frame_, segment_curvatures, segment_breaks)) {}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewiseConstantCurvatureTrajectory<T>::Clone()
    const {
  return std::make_unique<PiecewiseConstantCurvatureTrajectory<T>>(
      this->breaks(), segment_curvatures_,
      base_frame_.col(kCurveTangentDimension),
      base_frame_.col(kPlaneNormalDimension));
}

template <typename T>
math::RigidTransform<T> PiecewiseConstantCurvatureTrajectory<T>::GetPose(
    const T& distance) const {
  int segment_index = this->get_segment_index(distance);
  math::RigidTransform<T> X_SiT =
      CalcPoseInLocalFrame(segment_curvatures_[segment_index],
                           distance - this->start_time(segment_index));
  return segment_local_frames_[segment_index] * X_SiT;
}

template <typename T>
multibody::SpatialVelocity<T>
PiecewiseConstantCurvatureTrajectory<T>::GetVelocity(const T& distance) const {
  math::RotationMatrix<T> R_BT = GetPose(distance).rotation();
  T curvature = segment_curvatures_[this->get_segment_index(distance)];

  multibody::SpatialVelocity<T> spatial_velocity;
  // Anuglar velocity is about plane normal axis. The curvature is defined
  // as the ratio of the angular velocity to the linear velocity, and the
  // latter is fixed to the unit vector along the curve tangent.
  spatial_velocity.rotational() = R_BT.col(kPlaneNormalDimension) * curvature;
  spatial_velocity.translational() = R_BT.col(kCurveTangentDimension);
  return spatial_velocity;
}

template <typename T>
multibody::SpatialAcceleration<T>
PiecewiseConstantCurvatureTrajectory<T>::GetAcceleration(
    const T& distance) const {
  math::RotationMatrix<T> R_BT = GetPose(distance).rotation();
  T curvature = segment_curvatures_[this->get_segment_index(distance)];

  multibody::SpatialAcceleration<T> spatial_acceleration;
  // As the trajectory has piecewise constant curvature, it has piecewise
  // constant angular velocity and thus zero angular acceleration almost
  // everywhere. The linear acceleration is simply the centripetal
  // accleration, which points along the curve normal axis.
  spatial_acceleration.rotational() = Vector3<T>::Zero();
  spatial_acceleration.translational() =
      R_BT.col(kCurveNormalDimension) * curvature;
  return spatial_acceleration;
}

template <typename T>
boolean<T> PiecewiseConstantCurvatureTrajectory<T>::IsNearlyPeriodic(
    double tolerance) const {
  return GetPose(this->start_time())
      .IsNearlyEqualTo(GetPose(this->end_time()), tolerance);
}

template <typename T>
math::RigidTransform<T>
PiecewiseConstantCurvatureTrajectory<T>::CalcPoseInLocalFrame(
    const T& curvature, const T& length) {
  Vector3<T> translation = Vector3<T>::Zero();
  // Calculate rotation angle
  T theta = length * curvature;
  math::RotationMatrix<T> rotation =
      math::RotationMatrix<T>::MakeZRotation(theta);
  if (curvature == T(0)) {
    // Casel 1: zero curvature (straight line)
    translation(kCurveTangentDimension) = length;
  } else {
    // Case 2: non-zero curvature (circular arc)
    // Calculate position on the circular arc, which
    // is about the centerpoint 1/curvature * normal_axis with
    // radius 1/curvature
    translation(kCurveNormalDimension) = (T(1) - cos(theta)) / curvature;
    translation(kCurveTangentDimension) = sin(theta) / curvature;
  }
  return math::RigidTransform<T>(rotation, translation);
}

template <typename T>
math::RotationMatrix<T> PiecewiseConstantCurvatureTrajectory<T>::MakeBaseFrame(
    const Vector3<T>& curve_tangent_axis, const Vector3<T>& plane_normal_axis) {
  double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  // NOTE: assumes right hand rule for tangent \times normal = plane
  const bool right_hand =
      (kPlaneNormalDimension + 1) % 3 == kCurveTangentDimension;
  DRAKE_DEMAND(right_hand);
  const Vector3<T> normal_axis = plane_normal_axis.cross(curve_tangent_axis);

  DRAKE_DEMAND(curve_tangent_axis.norm() >= kEpsilon);
  DRAKE_DEMAND(normal_axis.norm() >= kEpsilon);
  DRAKE_DEMAND(plane_normal_axis.norm() >= kEpsilon);
  Matrix3<T> base_frame_matrix = Matrix3<T>::Zero();

  // Fill in the rotation matrix
  base_frame_matrix.col(kCurveNormalDimension) = normal_axis.normalized();
  base_frame_matrix.col(kCurveTangentDimension) =
      curve_tangent_axis.normalized();
  base_frame_matrix.col(kPlaneNormalDimension) = plane_normal_axis.normalized();
  return math::RotationMatrix<T>(base_frame_matrix);
}

template <typename T>
std::vector<math::RigidTransform<T>>
PiecewiseConstantCurvatureTrajectory<T>::MakeSegmentLocalFrames(
    const math::RotationMatrix<T>& base_frame,
    const std::vector<T>& segment_curvatures,
    const std::vector<T>& segment_breaks) {
  size_t num_breaks = segment_breaks.size();
  size_t num_segments = num_breaks - 1;

  // calculate angular change and length of each segment.
  const VectorX<T> segment_breaks_eigen =
      Eigen::Map<const VectorX<T>>(segment_breaks.data(), num_breaks);
  const VectorX<T> curvatures_eigen =
      Eigen::Map<const VectorX<T>>(segment_curvatures.data(), num_segments);
  const VectorX<T> segment_lengths = segment_breaks_eigen.tail(num_segments) -
                                     segment_breaks_eigen.head(num_segments);
  const VectorX<T> segment_rotation_angles =
      segment_lengths.cwiseProduct(curvatures_eigen);
  VectorX<T> segment_cumulative_rotations = VectorX<T>::Zero(num_breaks);
  std::partial_sum(segment_rotation_angles.begin(),
                   segment_rotation_angles.end(),
                   segment_cumulative_rotations.begin() + 1);

  // build frames for the start of each segment.
  std::vector<math::RigidTransform<T>> segment_local_frames;
  segment_local_frames.reserve(num_segments);
  segment_local_frames.push_back(math::RigidTransform<T>(base_frame));

  for (size_t i = 0; i < num_segments; i++) {
    math::RigidTransform<T> X_SprevSi =
        CalcPoseInLocalFrame(segment_curvatures[i], segment_lengths[i]);
    segment_local_frames.push_back(segment_local_frames.back() * X_SprevSi);
  }

  return segment_local_frames;
}

// Explicit instantiation for the types specified in the header
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseConstantCurvatureTrajectory);

}  // namespace trajectories
}  // namespace drake
