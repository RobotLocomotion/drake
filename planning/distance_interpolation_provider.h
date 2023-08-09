#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

class DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceAndInterpolationProvider);

  virtual ~DistanceAndInterpolationProvider();

  double ComputeConfigurationDistance(const Eigen::VectorXd& from,
                                      const Eigen::VectorXd& to) const;

  Eigen::VectorXd InterpolateBetweenConfigurations(const Eigen::VectorXd& from,
                                                   const Eigen::VectorXd& to,
                                                   double ratio) const;

 protected:
  DistanceAndInterpolationProvider();

  virtual double DoComputeConfigurationDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const = 0;

  virtual Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const = 0;
};

}  // namespace planning
}  // namespace drake
