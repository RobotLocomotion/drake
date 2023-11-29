#include "drake/planning/rejection_sampler.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/planning/uniform_set_sampler.h"

namespace drake {
namespace planning {
namespace {
using Eigen::Vector2d;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperrectangle;

GTEST_TEST(RejectionSampler, UniformBoxMissingCorner) {
  /* A 1x1 box domain with a 0.45x0.45 obstacle in each corner. The free space
   * is a cross composed to the union of
   * Box1 with vertices (0.45,0), (0.45, 1), (0.55, 0), (0.55, 1)
   * and
   * Box2 with vertices (0, 0.45), (0, 0.55), (1, 0.45), (1, 0.55)
   */
  const Hyperrectangle domain{Vector2d(0, 0), Vector2d(1, 1)};
  RandomGenerator generator(0);
  std::shared_ptr<PointSamplerBase> uniform_sampler =
      std::make_shared<UniformSetSampler<Hyperrectangle>>(domain, generator);

  ConvexSets obstacles;
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(0.45, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0), Vector2d(1, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0.55), Vector2d(0.45, 1)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0.55), Vector2d(1, 1)));

  const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>&
      rejection_fun =
          [&obstacles](const Eigen::Ref<const Eigen::VectorXd>& sample) {
            for (const auto& obstacle : obstacles) {
              if (obstacle->PointInSet(sample)) {
                return true;
              }
            }
            return false;
          };

  RejectionSampler sampler{std::move(uniform_sampler), rejection_fun};

  const int num_samples = 10;
  const Eigen::MatrixXd samples = sampler.SamplePoints(num_samples);
  for (int i = 0; i < samples.cols(); ++i) {
    for (const auto& obstacle : obstacles) {
      EXPECT_FALSE(obstacle->PointInSet(samples.col(i)));
    }
  }
}

}  // namespace
}  // namespace planning
}  // namespace drake