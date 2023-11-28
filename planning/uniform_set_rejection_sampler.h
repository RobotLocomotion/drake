#pragma once

#include "drake/common/random.h"
#include "drake/planning/point_sampler_base.h"

namespace drake {
namespace planning {
template <typename T>
class UniformSetRejectionSampler : public PointSamplerBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniformSetRejectionSampler);
  /**
   * This class provides a concrete implementation of the PointSamplerBase
   * interface. Instances of this class draw points uniformly at random from an
   * underlying set, usually one of Drake's ConvexSets.
   */
  UniformSetRejectionSampler(const T& set,
                             const std::shared_pointer<ConvexSets> obstacles);

  /**
   * This class provides a concrete implementation of the PointSamplerBase
   * interface. Instances of this class draw points uniformly at random from an
   * underlying set, usually one of Drake's ConvexSets. Passing a known
   * generator enables the user more control over the intialization of the
   * random nature of this class.
   */
  UniformSetRejectionSampler(const T& set,
                             const std::shared_pointer<ConvexSets> obstacles,
                             const RandomGenerator& generator);

 private:
  Eigen::MatrixXd DoSamplePoints(int num_points) override;

  T set_;
  std::shared_pointer<ConvexSets> obstacles_;
  RandomGenerator generator_;
};

}  // namespace planning
}  // namespace drake