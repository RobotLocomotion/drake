#include "drake/planning/point_sampler_base.h"
namespace drake {
namespace planning {

Eigen::MatrixXd PointSamplerBase::SamplePoints(int num_points) {
  return DoSamplePoints(num_points);
}

}  // namespace planning
}  // namespace drake
