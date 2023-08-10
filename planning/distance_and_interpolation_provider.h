#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

/** This class represents the base interface for performing configuration
distance and interpolation operations, used by CollisionChecker. */
class DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceAndInterpolationProvider);

  virtual ~DistanceAndInterpolationProvider();

  /** Computes the configuration distance from the provided configuration `from`
  to the provided configuration `to`. The returned distance will be strictly
  non-negative.
  @pre from.size() == to.size() */
  double ComputeConfigurationDistance(const Eigen::VectorXd& from,
                                      const Eigen::VectorXd& to) const;

  /** Returns the interpolated configuration between `from` and `to` at `ratio`.
  @pre from.size() == to.size()
  @pre ratio in [0, 1] */
  Eigen::VectorXd InterpolateBetweenConfigurations(const Eigen::VectorXd& from,
                                                   const Eigen::VectorXd& to,
                                                   double ratio) const;

 protected:
  DistanceAndInterpolationProvider();

  /** Derived distance and interpolation providers must implement distance
  computation. The returned distance must be non-negative.
  DistanceAndInterpolationProvider ensures that `from` and `to` are the same
  size. */
  virtual double DoComputeConfigurationDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const = 0;

  /** Derived distance and interpolation providers must implement interpolation.
  The returned configuration must be the same size as `from` and `to`.
  DistanceAndInterpolationProvider ensures that `from` and `to` are the same
  size and that `ratio` is in [0, 1]. */
  virtual Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const = 0;
};

}  // namespace planning
}  // namespace drake
