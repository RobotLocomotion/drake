#include "drake/planning/iris_from_clique_cover.h"

#include "drake/common/random.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"

namespace drake {
namespace planning {

namespace {
class UniformConfigurationSpaceSampler : public PointSamplerBase {
 public:
  UniformConfigurationSpaceSampler(const Eigen::Ref<Eigen::VectorXd> &lb,
                                   const Eigen::Ref<Eigen::VectorXd> &ub)
      :  bounding_box_(lb, ub), generator_() {};

  Eigen::MatrixXd SamplePoints(int num_points) {
    Eigen::MatrixXd ret(bounding_box_.ambient_dimension(), num_points);
    for (int i = 0; i < num_points; ++i) {
      ret.col(i) = bounding_box_.UniformSample(&generator_);
    }
    return ret;
  }

 private:
  const geometry::optimization::Hyperrectangle bounding_box_;
  RandomGenerator generator_;
};

}  // namespace

}  // namespace planning
}  // namespace drake