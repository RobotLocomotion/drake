#pragma once

#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"

namespace drake {
namespace planning {

class UniformBoxSampler : public PointSamplerBase {
 public:
  UniformBoxSampler(const Hyperrectangle& bounding_box);
  UniformBoxSampler(const Hyperrectangle& bounding_box,
                    const RandomGenerator& generator);
  UniformBoxSampler(const Eigen::Ref<Eigen::VectorXd>& lb,
                    const Eigen::Ref<Eigen::VectorXd>& ub);
  UniformBoxSampler(const Eigen::Ref<Eigen::VectorXd>& lb,
                    const Eigen::Ref<Eigen::VectorXd>& ub,
                    const RandomGenerator& generator);

  Eigen::MatrixXd SamplePoints(int num_points);

 private:
  const geometry::optimization::Hyperrectangle bounding_box_;
  RandomGenerator generator_;
};
}  // namespace planning
}  // namespace drake