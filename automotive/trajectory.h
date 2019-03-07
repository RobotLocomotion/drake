#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace automotive {

/// Wraps the raw data contained in a trajectory expressed in terms of pose and
/// velocity values.  Represents a translational and rotational transformation
/// of a reference frame A with respect to world frame W, expressed in x-y-z
/// coordinates, and translational and rotational velocities of frame A with
/// respect to W.
class PoseVelocity final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PoseVelocity)

  /// Default constructor.  Sets rotation to an identity transform and all
  /// translation and velocity components to zero.
  PoseVelocity();

  /// Fully-parameterized constructor.
  ///
  /// @param rotation the orientation R_WA of frame A with respect to the world
  /// frame W, and can be either normalized or unnormalized.
  /// @param translation the x, y, z position p_WA of frame A measured from W's
  /// origin.
  /// @param velocity the (rotational/translational) spatial velocity Xdot_WA of
  /// the frame A with respect to frame W.
  PoseVelocity(const Eigen::Quaternion<double>& rotation,
               const Eigen::Vector3d& translation,
               const multibody::SpatialVelocity<double>& velocity);

  /// Accesses p_WA, the translation component of the pose of A, expressed in
  /// world-frame coordinates W.
  const Eigen::Vector3d& translation() const { return translation_; }

  /// Accesses R_WA, the rotation component of the pose of A, expressed in
  /// world-frame coordinates W.
  const Eigen::Quaternion<double>& rotation() const { return rotation_; }

  /// Accesses the projection of the pose of frame A to the x-y components of
  /// translation and the z-component of rotation.
  Eigen::Vector3d pose3() const {
    const math::RollPitchYaw<double> rpy(rotation_);
    const double w_z = rpy.yaw_angle();
    return Eigen::Vector3d{translation_.x(), translation_.y(), w_z};
  }

  /// Accesses Xdot_WA, a SpatialVelocity of frame A, expressed in world-frame
  /// coordinates W.
  const multibody::SpatialVelocity<double>& velocity() const {
    return velocity_;
  }

  /// Gets the speed ‖pdot_WA‖₂, the 2-norm of the world-frame translational
  /// velocities.
  double speed() const { return velocity_.translational().norm(); }

 private:
  Eigen::Quaternion<double> rotation_;
  Eigen::Vector3d translation_;
  multibody::SpatialVelocity<double> velocity_;
};

/// A class that wraps a piecewise trajectory instantiated from pose data.  The
/// underlying pose data is represented internally as a configurable
/// PiecewisePolynomial for translation and a PiecewiseQuaternionSlerp for the
/// rotation.  The rotational/translational velocity of the pose is then
/// computed from time-differentiation of the PiecewisePolynomial and an angular
/// velocity conversion within PiecewiseQuaternionSlerp (see
/// piecewise_polynomial.h and piecewise_quaternion.h for details).  A
/// PoseVelocity instance is therefore well-defined when evaluated at a given
/// time and, additionally, the translaton and rotation components of PoseVector
/// match the input pose data exactly.
class Trajectory final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Trajectory)

  /// An identifier for the type of valid types of interpolation used in
  /// evaluating the translational component of a Trajectory.  These types
  /// mirror the associated constructors for PiecewisePolynomial (see
  /// common/trajectories/piecewise_polynomial.h for further details).
  enum class InterpolationType { kFirstOrderHold, kCubic, kPchip };

  /// Makes a Trajectory from a discrete set of time-indexed pose data
  /// under the specified interpolation scheme.
  ///
  /// @param times a vector of time indices representing the break points of the
  /// trajectory.
  /// @param knots_rotation a vector of time-indexed rotation data, whose length
  /// must match that of @p times.
  /// @param knots_translation a vector of time-indexed translation data, whose
  /// length must match that of @p times.
  /// @param interp_type an InterpolationType with the interpolation scheme used
  /// for building a piecewise polynomial trajectory for the translational
  /// component.
  /// @default InterpolationType::kFirstOrderHold.
  /// @throws std::logic_error if @p interp_type is not supported.
  /// @throws std::runtime_error if `times` and `knots` have different lengths,
  /// `times` is not strictly increasing, and the inputs are otherwise
  /// inconsistent with the given `interp_type` (see piecewise_polynomial.h).
  static Trajectory Make(
      const std::vector<double>& times,
      const std::vector<Eigen::Quaternion<double>>& knots_rotation,
      const std::vector<Eigen::Vector3d>& knots_translation,
      const InterpolationType& interp_type =
          InterpolationType::kFirstOrderHold);

  /// Makes a Trajectory from a discrete set of (time-independent)
  /// waypoints and a vector of speeds.  The resulting trajectory is assumed to
  /// start at time time t = 0 and follow a cubic-spline profile
  /// (InterpolationType::kCubic) for the translation elements, rendering speed
  /// as a piecewise-quadratic function.  For now, we determine the break points
  /// (time vector) from the time required to travel a Euclidean distance
  /// between consecutive waypoints at the average speed between each.  We apply
  /// this "average-speed" approach in order not to impose assumptions about the
  /// accelerations at the knot points.  Velocities at each break point are
  /// computed by taking into account both the translation and rotation of A
  /// with respect to W.
  ///
  /// @param waypoints_rotation a vector of rotations, whose length must match
  /// that of @p speeds.
  /// @param waypoints_translation a vector of translations, whose length must
  /// match that of @p speeds.
  /// @param speeds a vector of speeds to be applied at knot points and linearly
  /// interpolated between waypoints.  All entries of @p speeds must be
  /// non-negative, with some zero entries allowed, so long as they are not
  /// consecutive.
  /// @throws std::exception if `waypoints_*` and `speeds` have different
  /// lengths, any are empty, or if any element of `speeds` violates any of the
  /// conditions enumerated above.
  //
  // TODO(jadecastro) Compute the break points as the solution to an
  // optimization problem or from additional user inputs.
  static Trajectory MakeCubicFromWaypoints(
      const std::vector<Eigen::Quaternion<double>>& waypoints_rotation,
      const std::vector<Eigen::Vector3d>& waypoints_translation,
      const std::vector<double>& speeds);

  /// Makes a Trajectory from a discrete set of (time-independent)
  /// waypoints, based on a constant speed, using cubic-polynomial
  /// interpolation.
  ///
  /// @param waypoints_rotation a vector of rotations, whose length must match
  /// that of @p speeds.
  /// @param waypoints_translation a vector of translations, whose length must
  /// match that of @p speeds.
  /// @param speed a positive speed to be applied over the entirety of the
  /// trajectory.
  /// @throws std::exception if `speed` is non-positive or if either of the
  /// input vectors is empty.
  static Trajectory MakeCubicFromWaypoints(
      const std::vector<Eigen::Quaternion<double>>& waypoints_rotation,
      const std::vector<Eigen::Vector3d>& waypoints_translation, double speed);

  /// Evaluates the Trajectory at a given @p time, returning a packed
  /// PoseVelocity.
  PoseVelocity value(double time) const;

 private:
  // Constructs a Trajectory from a translation PiecewisePolynomial, @p
  // translation, and a rotation PiecewiseQuaternionSlerp, @p rotation.
  explicit Trajectory(
      const trajectories::PiecewisePolynomial<double>& translation,
      const trajectories::PiecewiseQuaternionSlerp<double>& rotation);

  trajectories::PiecewisePolynomial<double> translation_;
  trajectories::PiecewiseQuaternionSlerp<double> rotation_;
  trajectories::PiecewisePolynomial<double> translation_dot_;
};

}  // namespace automotive
}  // namespace drake
