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

namespace internal {
static constexpr int kSize = 13;
}  // namespace internal

using multibody::SpatialVelocity;
using trajectories::PiecewisePolynomial;

class AgentTrajectory;

/// Wraps the raw data contained in a trajectory expressed in terms of pose and
/// velocitiy values.  Represents a translational and rotational transformation
/// of a reference frame A with respect to world frame W, expressed in x-y-z
/// coordinates, and translational and rotational velocities of frame A with
/// respect to W.
class AgentTrajectoryValues final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectoryValues)

  /// Default constructor.  Sets all poses to identity transforms, and all
  /// velocities to zero.
  AgentTrajectoryValues()
      : AgentTrajectoryValues(
            Eigen::Quaternion<double>::Identity(),
            Eigen::Translation<double, 3>::Identity(),
            SpatialVelocity<double>(Vector6<double>::Zero())) {}

  /// Fully-parameterized constructor.
  ///
  /// @param rotation the orientation R_WA of frame A with respect to the world
  /// frame W, and can be either normalized or unnormalized.
  /// @param translation the x, y, z position p_WA of frame A measured from W's
  /// origin.
  /// @param velocity the (rotational/translational) spatial velocity Xdot_WA of
  /// the frame A with respect to frame W.
  AgentTrajectoryValues(const Eigen::Quaternion<double>& rotation,
                        const Eigen::Translation<double, 3>& translation,
                        const SpatialVelocity<double>& velocity)
      : rotation_(rotation), translation_(translation), velocity_(velocity) {}

  /// Accesses X_WA, an Isometry3 representation of pose of A, expressed in
  /// world-frame coordinates W.
  Eigen::Isometry3d isometry3() const {
    Eigen::Isometry3d isometry(translation_);
    isometry.rotate(rotation_);
    return isometry;
  }

  /// Accesses the projection of the pose of frame A to the x-y components of
  /// translation and the z-component of rotation.
  Eigen::Vector3d pose3() const {
    const double w_z = math::QuaternionToSpaceXYZ(rotation_).z();
    return Eigen::Vector3d{translation_.x(), translation_.y(), w_z};
  }

  /// Accesses Xdot_WA, a SpatialVelocity of frame A, expressed in world-frame
  /// coordinates W.
  const SpatialVelocity<double>& velocity() const { return velocity_; }

  /// Gets the speed ||pdot_WA||₂, the 2-norm of the world-frame translational
  /// velocities.
  double speed() const { return velocity_.translational().norm(); }

 private:
  // Constructs AgentTrajectoryValues from a raw VectorXd of pose and velocity
  // data in the order enumerated within Indices.
  explicit AgentTrajectoryValues(const Eigen::VectorXd vector)
      : rotation_(Eigen::Quaternion<double>(
            vector(Indices::kRw), vector(Indices::kRx), vector(Indices::kRy),
            vector(Indices::kRz))),
        translation_(Eigen::Translation<double, 3>(
            vector(Indices::kTx), vector(Indices::kTy), vector(Indices::kTz))),
        velocity_(Eigen::Vector3d(vector(Indices::kWx), vector(Indices::kWy),
                                  vector(Indices::kWz)),
                  Eigen::Vector3d(vector(Indices::kVx), vector(Indices::kVy),
                                  vector(Indices::kVz))) {
    DRAKE_DEMAND(vector.size() == internal::kSize);
  }

  // Accesses the raw pose and velocity data in the form of a VectorXd in the
  // order enumerated within Indices.
  Eigen::VectorXd to_vector() const {
    Eigen::VectorXd vector(internal::kSize);
    vector(Indices::kTx) = translation_.x();
    vector(Indices::kTy) = translation_.y();
    vector(Indices::kTz) = translation_.z();
    vector(Indices::kRw) = rotation_.w();
    vector(Indices::kRx) = rotation_.x();
    vector(Indices::kRy) = rotation_.y();
    vector(Indices::kRz) = rotation_.z();
    vector(Indices::kWx) = velocity_.rotational().x();
    vector(Indices::kWy) = velocity_.rotational().y();
    vector(Indices::kWz) = velocity_.rotational().z();
    vector(Indices::kVx) = velocity_.translational().x();
    vector(Indices::kVy) = velocity_.translational().y();
    vector(Indices::kVz) = velocity_.translational().z();
    return vector;
  }

  // Enumerates the indices for each of the raw data elements passed privately
  // to/from this class.
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

  Eigen::Quaternion<double> rotation_;
  Eigen::Translation<double, 3> translation_;
  SpatialVelocity<double> velocity_;

  // We friend AgentTrajectory to allow it to populate via the raw data vector
  // constructor and raw data accessor.
  friend class AgentTrajectory;
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
/// AgentTrajectoryValues data.  The underlying piecewise polynomial may be
/// freely configured and, when evaluated at each knot point, the result will
/// match the input data exactly.
class AgentTrajectory final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectory)

  /// Makes an AgentTrajectory from a discrete set of time-indexed
  /// AgentTrajectoryValues data, under the specified interpolation scheme.
  ///
  /// @param interp_type an InterpolationType with the interpolation scheme used
  /// for building a piecewise polynomial trajectory.
  /// @param times a vector of time indices representing the break points of the
  /// trajectory.
  /// @param knots a knot points containing a vector of AgentTrajectoryValues,
  /// whose length must match that of @p times.
  /// @throws std::logic_error if @p interp_type is not supported.
  static AgentTrajectory Make(const InterpolationType& interp_type,
                              const std::vector<double>& times,
                              const std::vector<AgentTrajectoryValues>& knots) {
    DRAKE_THROW_UNLESS(times.size() == knots.size());
    std::vector<Eigen::MatrixXd> knots_vector(times.size());
    for (int i{0}; i < static_cast<int>(times.size()); ++i) {
      knots_vector[i] = knots[i].to_vector();
    }
    switch (interp_type) {
      case InterpolationType::kZeroOrderHold:
        return AgentTrajectory(
            PiecewisePolynomial<double>::ZeroOrderHold(times, knots_vector));
      case InterpolationType::kFirstOrderHold:
        return AgentTrajectory(
            PiecewisePolynomial<double>::FirstOrderHold(times, knots_vector));
      case InterpolationType::kCubic:
        return AgentTrajectory(
            PiecewisePolynomial<double>::Cubic(times, knots_vector));
      case InterpolationType::kPchip:
        return AgentTrajectory(
            PiecewisePolynomial<double>::Pchip(times, knots_vector));
      default:
        throw std::logic_error("The provided interp_type is not supported.");
    }
  }

  /// Evaluates the AgentTrajectory at a given @p time, with the result returned
  /// as AgentTrajectoryValues.
  AgentTrajectoryValues value(double time) const {
    return AgentTrajectoryValues{trajectory_.value(time)};
  }

 private:
  // Constructs an AgentTrajectory from an appropriately-dimensioned
  // PiecewisePolynomial, given by @p trajectory.
  explicit AgentTrajectory(const PiecewisePolynomial<double>& trajectory)
      : trajectory_(trajectory) {}

  PiecewisePolynomial<double> trajectory_;
};

}  // namespace automotive
}  // namespace drake
