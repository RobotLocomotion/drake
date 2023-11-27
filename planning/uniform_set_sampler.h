#pragma once

#include "drake/common/random.h"
#include "drake/planning/point_sampler_base.h"

namespace drake {
namespace planning {
template <typename T>
class UniformSetSampler : public PointSamplerBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniformSetSampler);
  /**
   * This class provides a concrete implementation of the PointSamplerBase
   * interface. Instances of this class draw points uniformly at random from an
   * underlying set, usually one of Drake's ConvexSets.
   */
  UniformSetSampler(const T& set);

  /**
   * This class provides a concrete implementation of the PointSamplerBase
   * interface. Instances of this class draw points uniformly at random from an
   * underlying set, usually one of Drake's ConvexSets. Passing a known
   * generator enables the user more control over the intialization of the
   * random nature of this class.
   */
  UniformSetSampler(const T& set, const RandomGenerator& generator);

 protected:
  Eigen::MatrixXd DoSamplePoints(int num_points);

 private:
  T set_;
  RandomGenerator generator_;
};

}  // namespace planning
}  // namespace drake