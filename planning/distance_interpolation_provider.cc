#include "drake/planning/distance_interpolation_provider.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {

DistanceAndInterpolationProvider::~DistanceAndInterpolationProvider() = default;

double DistanceAndInterpolationProvider::ComputeConfigurationDistance(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
  return DoComputeConfigurationDistance(from, to);
}

Eigen::VectorXd
DistanceAndInterpolationProvider::InterpolateBetweenConfigurations(
    const Eigen::VectorXd& from, const Eigen::VectorXd& to,
    const double ratio) const {
  DRAKE_THROW_UNLESS(ratio >= 0.0);
  DRAKE_THROW_UNLESS(ratio <= 1.0);
  return DoInterpolateBetweenConfigurations(from, to, ratio);
}

std::unique_ptr<DistanceAndInterpolationProvider>
DistanceAndInterpolationProvider::Clone() const { return DoClone(); }

DistanceAndInterpolationProvider::DistanceAndInterpolationProvider(
    const DistanceAndInterpolationProvider& other) = default;

}  // namespace planning
}  // namespace drake
