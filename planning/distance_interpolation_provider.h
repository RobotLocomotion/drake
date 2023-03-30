#pragma once

#include <memory>

#include <Eigen/Core>

namespace drake {
namespace planning {

class DistanceAndInterpolationProvider {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  DistanceAndInterpolationProvider(DistanceAndInterpolationProvider&&) = delete;
  DistanceAndInterpolationProvider& operator=(
      const DistanceAndInterpolationProvider&) = delete;
  DistanceAndInterpolationProvider& operator=(
      DistanceAndInterpolationProvider&&) = delete;

  virtual ~DistanceAndInterpolationProvider();

  std::unique_ptr<DistanceAndInterpolationProvider> Clone() const;

  double ComputeConfigurationDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;

  Eigen::VectorXd InterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const;

 protected:
  DistanceAndInterpolationProvider();

  /// Copy constructor for use in Clone().
  DistanceAndInterpolationProvider(
      const DistanceAndInterpolationProvider& other);

  virtual std::unique_ptr<DistanceAndInterpolationProvider> DoClone() const = 0;

  virtual double DoComputeConfigurationDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const = 0;

  virtual Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const = 0;
};

}  // namespace planning
}  // namespace drake
