#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

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

  /// Default constructor.  Sets all poses to identity transforms, and all
  /// velocities to zero.
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
               const Eigen::Translation<double, 3>& translation,
               const multibody::SpatialVelocity<double>& velocity);

  /// Accesses p_WA, the translation component of the pose of A, expressed in
  /// world-frame coordinates W.
  const Eigen::Translation<double, 3>& translation() const {
    return translation_;
  }

  /// Accesses R_WA, the rotation component of the pose of A, expressed in
  /// world-frame coordinates W.
  const Eigen::Quaternion<double>& rotation() const { return rotation_; }

  /// Accesses the projection of the pose of frame A to the x-y components of
  /// translation and the z-component of rotation.
  Eigen::Vector3d pose3() const {
    const double w_z = math::QuaternionToSpaceXYZ(rotation_).z();
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
  Eigen::Translation<double, 3> translation_;
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
class AgentTrajectory final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectory)

  /// An identifier for the type of interpolation to be used for the
  /// interpolating the translational component of an AgentTrajectory.  These
  /// types mirror the associated constructors for PiecewisePolynomial (see
  /// common/trajectories/piecewise_polynomial.h for further details).
  enum class InterpolationType {
    kZeroOrderHold,
    kFirstOrderHold,
    kCubic,
    kPchip
  };

  /// Makes an AgentTrajectory from a discrete set of time-indexed pose data
  /// under the specified interpolation scheme.
  ///
  /// @param times a vector of time indices representing the break points of the
  /// trajectory.
  /// @param knots a knot points containing a vector of Isometry3 pose data,
  /// whose length must match that of @p times.
  /// @param interp_type an InterpolationType with the interpolation scheme used
  /// for building a piecewise polynomial trajectory for the translational
  /// component.
  /// @default InterpolationType::kFirstOrderHold.
  /// @throws std::logic_error if @p interp_type is not supported.
  /// @throws std::runtime_error if `times` and `knots` have different lengths,
  /// `times` is not strictly increasing, and the inputs are otherwise
  /// inconsistent with the given `interp_type` (see piecewise_polynomial.h).
  static AgentTrajectory Make(const std::vector<double>& times,
                              const std::vector<Eigen::Isometry3d>& knots,
                              const InterpolationType& interp_type =
                                  InterpolationType::kFirstOrderHold);

  /// Evaluates the AgentTrajectory at a given @p time, returning a packed
  /// PoseVelocity.
  PoseVelocity value(double time) const;

 private:
  // Constructs an AgentTrajectory from a translation PiecewisePolynomial, @p
  // translation, and a rotation PiecewiseQuaternionSlerp, @p rotation.
  explicit AgentTrajectory(
      const trajectories::PiecewisePolynomial<double>& translation,
      const trajectories::PiecewiseQuaternionSlerp<double>& rotation);

  trajectories::PiecewisePolynomial<double> translation_;
  trajectories::PiecewiseQuaternionSlerp<double> rotation_;
  trajectories::PiecewisePolynomial<double> translation_dot_;
};

}  // namespace automotive
}  // namespace drake
