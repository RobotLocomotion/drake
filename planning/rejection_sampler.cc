#include "drake/planning/rejection_sampler.h"

#include <memory>

namespace drake {
namespace planning {

RejectionSampler::RejectionSampler(
    std::shared_ptr<PointSamplerBase> sampler,
    const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>&
        rejection_fun)
    : sampler_{sampler}, rejection_fun_{rejection_fun} {}

Eigen::MatrixXd RejectionSampler::DoSamplePoints(int num_points) {
  Eigen::VectorXd sample = sampler_->SamplePoints(1);
  Eigen::MatrixXd ret(sample.rows(), num_points);
  int points_added{0};
  int num_tries{0};
  while (points_added < num_points) {
    if (!rejection_fun_(sample)) {
      ret.col(points_added) = sample;
      ++points_added;
    }
    sample = sampler_->SamplePoints(1);
    ++num_tries;
  }
  return ret;
}

}  // namespace planning
}  // namespace drake
