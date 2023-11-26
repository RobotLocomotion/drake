#pragma once

#include "drake/planning/point_sampler_base.h"
#include "drake/common/random.h"

namespace drake {
namespace planning {
template <typename T>
class UniformSetSampler: public PointSamplerBase {
 public:
//  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniformSetSampler);
  UniformSetSampler(const T& set);
  UniformSetSampler(const T& set, const RandomGenerator& generator);

 protected:
  Eigen::MatrixXd DoSamplePoints(int num_points);

 private:
  T set_;
  RandomGenerator generator_;
};

}  // namespace planning
}  // namespace drake