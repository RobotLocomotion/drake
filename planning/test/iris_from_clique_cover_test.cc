#include "drake/planning/iris_from_clique_cover.h"

#include <chrono>

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/coverage_checker_via_bernoulli_test.h"
#include "drake/planning/rejection_sampler.h"
#include "drake/planning/uniform_set_sampler.h"

namespace drake {
namespace planning {
namespace {
using Eigen::Vector2d;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::IrisOptions;

GTEST_TEST(IrisFromCliqueCover, BoxWithCornerObstaclesTest) {
  /* A 1x1 box domain with a 0.45x0.45 obstacle in each corner. The free space
   * is a cross composed to the union of
   * Box1 with vertices (0.45,0), (0.45, 1), (0.55, 0), (0.55, 1)
   * and
   * Box2 with vertices (0, 0.45), (0, 0.55), (1, 0.45), (1, 0.55)
   */
  const HPolyhedron domain =
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(1, 1));
  ConvexSets obstacles;
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(0.45, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0), Vector2d(1, 0.45)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0, 0.55), Vector2d(0.45, 1)));
  obstacles.emplace_back(
      HPolyhedron::MakeBox(Vector2d(0.55, 0.55), Vector2d(1, 1)));

  IrisFromCliqueCoverOptions options;
  // TODO(Alexandre.Amice) bump this number of workers up to at least 2.
  options.num_builders = 2;
  options.num_points_per_coverage_check = 100;
  options.num_points_per_visibility_round = 100;
  std::vector<copyable_unique_ptr<HPolyhedron>> sets;
  std::cout << "entering iris from clique cover" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  IrisFromCliqueCover(obstacles, domain, options, &sets);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout
      << fmt::format(
             "Got {} sets in {}ms", sets.size(),
             std::chrono::duration<double, std::milli>(end - start).count())
      << std::endl;
  //  for(const auto& s: sets) {
  //    std::cout << fmt::format("A ={}", fmt_eigen(s->A())) << std::endl;
  //    std::cout << fmt::format("b ={}", fmt_eigen(s->b())) << std::endl;
  //    std::cout << std::endl;
  //  }

  EXPECT_EQ(ssize(sets), 2);

  // The sampling distribution for the domain of the configuration space.
  RandomGenerator generator{0};
  std::shared_ptr<PointSamplerBase> sampler =
      options.point_sampler.has_value()
          ? options.point_sampler.value()
          : std::make_shared<UniformSetSampler<HPolyhedron>>(domain, generator);

  const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>&
      reject_in_collision =
          [&obstacles](const Eigen::Ref<const Eigen::VectorXd>& sample) {
            for (const auto& obstacle : obstacles) {
              if (obstacle->PointInSet(sample)) {
                return true;
              }
            }
            return false;
          };
  // The rejection sampler for the C-Free.
  std::unique_ptr<PointSamplerBase> rejection_collision_sampler =
      std::make_unique<RejectionSampler>(sampler, reject_in_collision);

  CoverageCheckerViaBernoulliTest coverage_checker{
      0.9 /* Achieve 90% coverage */,
      static_cast<int>(1e3) /* number of sampled points for test*/,
      std::move(rejection_collision_sampler)};

  ConvexSets abstract_sets;
  double computed_volume{0};
  for (const auto& set : sets) {
    abstract_sets.emplace_back(set->Clone());
    computed_volume +=
        set->CalcVolumeViaSampling(&generator,
                                   1e-4 /* Ask for high relative accuracy */)
            .volume;
  }
  EXPECT_TRUE(coverage_checker.CheckCoverage(abstract_sets));
  // The free space is 0.2 units of area.
  EXPECT_GE(computed_volume, 0.189);

}

}  // namespace
}  // namespace planning
}  // namespace drake