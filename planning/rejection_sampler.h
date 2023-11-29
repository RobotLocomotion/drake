#pragma once

#include "drake/common/random.h"
#include "drake/planning/point_sampler_base.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

class RejectionSampler final : public PointSamplerBase {
 public:
//  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RejectionSampler);
  /**
   * This class provides a concrete implementation of the PointSamplerBase
   * interface. Instances of this class draw points according to the passed
   * sampler, but reject points which return true when passed to rejection_fun.
   */
  RejectionSampler(
      std::shared_ptr<PointSamplerBase> sampler,
      const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>& rejection_fun);

 private:
  Eigen::MatrixXd DoSamplePoints(int num_points) override;

  std::shared_ptr<PointSamplerBase> sampler_;

   const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>& rejection_fun_;
};

}  // namespace planning
}  // namespace drake