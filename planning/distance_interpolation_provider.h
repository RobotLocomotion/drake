#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

class DistanceAndInterpolationProvider {
 public:
  // The copy constructor is protected to allow certain derived classes to be
  // cloneable.
  // Does not allow copy, move, or assignment.
  DistanceAndInterpolationProvider(DistanceAndInterpolationProvider&&) = delete;
  DistanceAndInterpolationProvider& operator=(
      const DistanceAndInterpolationProvider&) = delete;
  DistanceAndInterpolationProvider& operator=(
      DistanceAndInterpolationProvider&&) = delete;

  virtual ~DistanceAndInterpolationProvider();

  double ComputeConfigurationDistance(const Eigen::VectorXd& from,
                                      const Eigen::VectorXd& to) const;

  Eigen::VectorXd InterpolateBetweenConfigurations(const Eigen::VectorXd& from,
                                                   const Eigen::VectorXd& to,
                                                   double ratio) const;

 protected:
  DistanceAndInterpolationProvider();

  DistanceAndInterpolationProvider(
      const DistanceAndInterpolationProvider& other);

  virtual double DoComputeConfigurationDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const = 0;

  virtual Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const = 0;
};

}  // namespace planning
}  // namespace drake
