#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

/** This class represents the base interface for performing configuration
distance and interpolation operations, used by CollisionChecker. See
LinearDistanceAndInterpolationProvider for an implementation covering common
"linear" distance and interpolation behavior.

Configuration distance and interpolation are necessary for a CollisionChecker to
perform edge collision checks, and an essential part of many motion planning
problems. The C-spaces for many planning problems combine joints with widely
differing effects (e.g. for a given angular change, the shoulder joint of a
robot arm results in much more significant motion than the same change on a
finger joint) or units (e.g. a mobile robot with translation in meters and yaw
in radians). As a result, it is often necessary to weight elements of the
configuration differently when computing configuration distance.

Likewise, in more complex C-spaces, it may be necessary to perform more complex
interpolation behavior (e.g. when planning for a mobile robot whose motion is
modelled via Dubbins or Reeds-Shepp paths).

Configuration distance takes two configurations of the robot, from and to, both
as Eigen::VectorXd, and returns (potentially weighted) C-space distance as a
double. The returned distance will be strictly non-negative.

To be valid, distance must satisfy the following condition:

 - ComputeConfigurationDistance(q, q) ≡ 0

for values of q that are valid for the C-space in use.

Configuration interpolation takes two configurations of the robot, from and to,
both as Eigen::VectorXd, plus a ratio in [0, 1] and returns the interpolated
configuration.

To be valid, interpolation must satisfy the following conditions:

 - InterpolateBetweenConfigurations(from, to, 0) ≡ from
 - InterpolateBetweenConfigurations(from, to, 1) ≡ to
 - InterpolateBetweenConfigurations(q, q, ratio) ≡ q, for all ratio in [0, 1]

for values of q, from, and to that are valid for the C-space in use.*/
class DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceAndInterpolationProvider);

  virtual ~DistanceAndInterpolationProvider();

  /** Computes the configuration distance from the provided configuration `from`
  to the provided configuration `to`. The returned distance will be strictly
  non-negative.
  @pre from.size() == to.size().
  @throws if `from` or `to` contain non-finite values. */
  double ComputeConfigurationDistance(const Eigen::VectorXd& from,
                                      const Eigen::VectorXd& to) const;

  /** Returns the interpolated configuration between `from` and `to` at `ratio`.
  @pre from.size() == to.size().
  @pre ratio in [0, 1].
  @throws if `from` or `to` contain non-finite values. */
  Eigen::VectorXd InterpolateBetweenConfigurations(const Eigen::VectorXd& from,
                                                   const Eigen::VectorXd& to,
                                                   double ratio) const;

 protected:
  DistanceAndInterpolationProvider();

  /** Derived distance and interpolation providers must implement distance
  computation. The returned distance must be non-negative.
  DistanceAndInterpolationProvider ensures that `from` and `to` are the same
  size.

  Base class guarantees that `from` and `to` contain only finite values. */
  virtual double DoComputeConfigurationDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const = 0;

  /** Derived distance and interpolation providers must implement interpolation.
  The returned configuration must be the same size as `from` and `to`.
  DistanceAndInterpolationProvider ensures that `from` and `to` are the same
  size and that `ratio` is in [0, 1].

  Base class guarantees that `from` and `to` contain only finite values. */
  virtual Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const = 0;
};

}  // namespace planning
}  // namespace drake
