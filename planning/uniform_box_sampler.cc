#include "drake/planning/uniform_box_sampler.h"

namespace drake {
namespace planning {

UniformBoxSampler::UniformBoxSampler(const Hyperrectangle& bounding_box)
    : bounding_box_(bounding_box), generator_() {}

UniformBoxSampler::UniformBoxSampler(const Eigen::Ref<Eigen::VectorXd>& lb,
                                     const Eigen::Ref<Eigen::VectorXd>& ub,
                                     const RandomGenerator& generator)
    : bounding_box_(lb, ub), generator_(generator) {}

UniformBoxSampler::UniformBoxSampler(const Eigen::Ref<Eigen::VectorXd>& lb,
                                     const Eigen::Ref<Eigen::VectorXd>& ub)
    : bounding_box_(lb, ub), generator_() {}

UniformBoxSampler::UniformBoxSampler(const Eigen::Ref<Eigen::VectorXd>& lb,
                                     const Eigen::Ref<Eigen::VectorXd>& ub,
                                     const RandomGenerator& generator)
    : bounding_box_(lb, ub), generator_(generator) {}

Eigen::MatrixXd UniformBoxSampler::SamplePoints(int num_points) {
  Eigen::MatrixXd ret(bounding_box_.ambient_dimension(), num_points);
  for (int i = 0; i < num_points; ++i) {
    ret.col(i) = bounding_box_.UniformSample(&generator_);
  }
  return ret;
}
}  // namespace planning
}  // namespace drake