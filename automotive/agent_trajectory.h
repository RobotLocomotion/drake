#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace automotive {

/// Wraps the raw data contained in a trajectory expressed in terms of pose and
/// velocitiy values.  Represents a translational and rotational transformation
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

  /// Constructs Pose from a raw VectorXd of pose and velocity data in the order
  /// enumerated within PoseVelocity::AgentIndices.
  ///
  /// Note that this constructor is for use within AgentTrajectory and callers
  /// should NOT use it; instead use one of the above constructors.
  ///
  /// @param vector the vector of raw data values.
  explicit PoseVelocity(const Eigen::VectorXd& vector);

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

  /// Gets the speed ||pdot_WA||₂, the 2-norm of the world-frame translational
  /// velocities.
  double speed() const { return velocity_.translational().norm(); }

 private:
  Eigen::Quaternion<double> rotation_;
  Eigen::Translation<double, 3> translation_;
  multibody::SpatialVelocity<double> velocity_;
};

/// An identifier for the type of interpolation to be used when forming an
/// AgentTrajectory.  These types mirror the associated constructors for
/// PiecewisePolynomial (see common/trajectories/piecewise_polynomial.h for
/// details on each of the interpolation schemes).
enum class InterpolationType {
  kZeroOrderHold,
  kFirstOrderHold,
  kCubic,
  kPchip
};

/// A class that wraps a piecewise trajectory instantiated based on
/// PoseVelocity data.  The underlying piecewise polynomial may be
/// freely configured and, when evaluated at each knot point, the result will
/// match the input data exactly.
class AgentTrajectory final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectory)

  /// Makes an AgentTrajectory from a discrete set of time-indexed
  /// PoseVelocity data, under the specified interpolation scheme.
  ///
  /// @param times a vector of time indices representing the break points of the
  /// trajectory.
  /// @param knots a knot points containing a vector of PoseVelocity,
  /// whose length must match that of @p times.
  /// @param interp_type an InterpolationType with the interpolation scheme used
  /// for building a piecewise polynomial trajectory.
  /// @default InterpolationType::kFirstOrderHold.
  /// @throws std::logic_error if @p interp_type is not supported.
  /// @throws std::runtime_error if `times` and `knots` have different lengths,
  /// `times` is not strictly increasing, and the inputs are otherwise
  /// inconsistent with the given `interp_type` (see piecewise_polynomial.h).
  static AgentTrajectory Make(const std::vector<double>& times,
                              const std::vector<PoseVelocity>& knots,
                              const InterpolationType& interp_type =
                                  InterpolationType::kFirstOrderHold);

  /// Evaluates the AgentTrajectory at a given @p time, with the result returned
  /// as PoseVelocity.
  PoseVelocity value(double time) const {
    return from_vector(trajectory_.value(time));
  }

 private:
  // Constructs an AgentTrajectory from an appropriately-dimensioned
  // PiecewisePolynomial, given by @p trajectory.
  explicit AgentTrajectory(
      const trajectories::PiecewisePolynomial<double>& trajectory);

  // Makes a VectorXd of raw pose and velocity from the specified PoseVelocity
  // assuming the ordering enumerated within AgentTrajectory::Indices.
  static Eigen::VectorXd to_vector(const PoseVelocity& pose_velocity);

  // Makes a PoseVelocity from the specified VectorXd of raw pose and velocity
  // data assuming the ordering enumerated within AgentTrajectory::Indices.
  static PoseVelocity from_vector(const Eigen::VectorXd& vector);

  // Enumerates the indices for each of the necessary raw data elements to fully
  // specify a PoseVelocity.
  struct Indices {
    static const int kTx = 0;
    static const int kTy = 1;
    static const int kTz = 2;
    static const int kRw = 3;
    static const int kRx = 4;
    static const int kRy = 5;
    static const int kRz = 6;
    static const int kWx = 7;
    static const int kWy = 8;
    static const int kWz = 9;
    static const int kVx = 10;
    static const int kVy = 11;
    static const int kVz = 12;
  };

  trajectories::PiecewisePolynomial<double> trajectory_;
};

}  // namespace automotive
}  // namespace drake
