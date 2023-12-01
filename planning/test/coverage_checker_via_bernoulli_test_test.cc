#include "drake/planning/coverage_checker_via_bernoulli_test.h"

#include <gtest/gtest.h>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/planning/uniform_set_sampler.h"

namespace drake {
namespace planning {
namespace {
using Eigen::Vector2d;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperrectangle;
using geometry::optimization::VPolytope;

// Check that the return of GetSampledCoverageFraction is within 1 standard
// deviation of the expected coverage.
void TestCoveredFractionWithinOneStandardDeviationOfExpected(
    const CoverageCheckerViaBernoulliTest& checker,
    const ConvexSets& current_sets, const double expected_coverage) {
  const double coverage{checker.GetSampledCoverageFraction(current_sets)};

  const double standard_deviation_of_check{checker.get_num_points_per_check() *
                                           expected_coverage *
                                           (1 - expected_coverage)};

  // Check that we achieve approximately expected_coverage within 1 standard
  // deviation.
  EXPECT_LE(abs(coverage - expected_coverage), standard_deviation_of_check);
}

GTEST_TEST(BernoulliCoverageCheck, TestCtorSettersAndGetters) {
  // Ask for 80% coverage, when we have 90% coverage.
  const double alpha{0.8};
  const int num_points_per_check{static_cast<int>(1E3)};
  const int num_threads = 2;
  const double point_in_set_tol = 1e-10;
  const Hyperrectangle domain{Vector2d{0, 0}, Vector2d{1, 1}};
  RandomGenerator generator(0);
  std::unique_ptr<PointSamplerBase> sampler =
      std::make_unique<UniformSetSampler<Hyperrectangle>>(domain, generator);
  CoverageCheckerViaBernoulliTest checker{alpha, num_points_per_check,
                                          std::move(sampler), num_threads,
                                          point_in_set_tol};

  EXPECT_EQ(checker.get_alpha(), alpha);
  EXPECT_EQ(checker.get_num_points_per_check(), num_points_per_check);
  EXPECT_EQ(checker.get_num_threads(), num_threads);
  EXPECT_EQ(checker.get_point_in_set_tol(), point_in_set_tol);

  const double new_alpha{0.2};
  checker.set_alpha(new_alpha);
  EXPECT_EQ(checker.get_alpha(), new_alpha);
  const int new_num_points_per_check{100};
  checker.set_num_points_per_check(new_num_points_per_check);
  EXPECT_EQ(checker.get_num_points_per_check(), new_num_points_per_check);
  const int new_num_threads{100};
  checker.set_num_threads(new_num_threads);
  EXPECT_EQ(checker.get_num_threads(), new_num_threads);
  const double new_point_in_set_tol{1e-6};
  checker.set_point_in_set_tol(new_point_in_set_tol);
  EXPECT_EQ(checker.get_point_in_set_tol(), new_point_in_set_tol);

  // Check that set_alpha clamps to the range [0,1]
  checker.set_alpha(-1);
  EXPECT_EQ(checker.get_alpha(), 0);
  // Check that set_alpha clamps to the range [0,1]
  checker.set_alpha(10);
  EXPECT_EQ(checker.get_alpha(), 1);

  std::unique_ptr<PointSamplerBase> sampler2 =
      std::make_unique<UniformSetSampler<Hyperrectangle>>(domain, generator);
  CoverageCheckerViaBernoulliTest checker_default{alpha, num_points_per_check,
                                                  std::move(sampler2)};
  // Check that the default number of threads is -1
  EXPECT_EQ(checker_default.get_num_threads(), -1);
  // Check that the default point_in_set_tol_ is 1e-8
  EXPECT_EQ(checker_default.get_point_in_set_tol(), 1e-8);
}

/*
 * A 1x1 side length box, covered by the union of 0.45 x 0.5 length boxes in
 * each corner. This covers 90% of the 1x1 box.
 */
GTEST_TEST(BernoulliCoverageCheck, BoxDomainCoveredByBoxesSuccess) {
  const double alpha{0.8};
  const int num_points_per_check{static_cast<int>(1E3)};
  const int num_threads = 3;
  const double point_in_set_tol = 1e-10;
  const Hyperrectangle domain{Vector2d{0, 0}, Vector2d{1, 1}};
  RandomGenerator generator(0);
  std::unique_ptr<PointSamplerBase> sampler =
      std::make_unique<UniformSetSampler<Hyperrectangle>>(domain, generator);
  CoverageCheckerViaBernoulliTest checker{alpha, num_points_per_check,
                                          std::move(sampler), num_threads,
                                          point_in_set_tol};

  ConvexSets current_sets;
  current_sets.emplace_back(
      std::make_unique<Hyperrectangle>(Vector2d{0, 0}, Vector2d{0.45, 0.5}));
  current_sets.emplace_back(
      std::make_unique<Hyperrectangle>(Vector2d{0.55, 0}, Vector2d{1, 0.5}));
  current_sets.emplace_back(
      std::make_unique<Hyperrectangle>(Vector2d{0, 0.5}, Vector2d{0.45, 1}));
  current_sets.emplace_back(
      std::make_unique<Hyperrectangle>(Vector2d{0.55, 0.5}, Vector2d{1, 1}));

  TestCoveredFractionWithinOneStandardDeviationOfExpected(
      checker, current_sets,
      0.90 /* Check that we achieve approximately 90% coverage.*/);

  /* Asking for 80% coverage when we have 90% */
  checker.set_alpha(0.8);
  EXPECT_TRUE(checker.CheckCoverage(current_sets));
}

/*
 * A 1x1 side length box, covered by a single of 0.5 x 0.5 length boxes in
 * the bottom left corner. This covers 25% of the 1x1 box.
 */
GTEST_TEST(BernoulliCoverageCheck, BoxDomainCoveredByBoxesFails) {
  // Ask for 80% coverage, when we have 25% coverage.
  const double alpha{0.8};
  const int num_points_per_check{static_cast<int>(1E5)};
  const int num_threads = 3;
  const Hyperrectangle domain{Vector2d{0, 0}, Vector2d{1, 1}};
  const double point_in_set_tol = 1e-10;
  RandomGenerator generator(0);
  std::unique_ptr<PointSamplerBase> sampler =
      std::make_unique<UniformSetSampler<Hyperrectangle>>(domain, generator);
  CoverageCheckerViaBernoulliTest checker{alpha, num_points_per_check,
                                          std::move(sampler), num_threads,
                                          point_in_set_tol};

  ConvexSets current_sets;
  current_sets.emplace_back(
      std::make_unique<Hyperrectangle>(Vector2d{0, 0}, Vector2d{0.5, 0.5}));

  TestCoveredFractionWithinOneStandardDeviationOfExpected(
      checker, current_sets,
      0.25 /* Check that we achieve approximately 25% coverage.*/);
  /* Asking for 80% coverage when we only have 25% */
  checker.set_alpha(0.8);
  EXPECT_FALSE(checker.CheckCoverage(current_sets));
}

/*
 * A 1x1 side length box, covered by a single of HPolyedron consisting of the
 * bottom left diagonal. This covers 50% of the box. This test checks that the
 * underlying domain need not be the same type as the covering sets.
 */
GTEST_TEST(BernoulliCoverageCheck, BoxDomainCoveredByHPolyhedron) {
  const double alpha{1};
  const int num_points_per_check{static_cast<int>(1E3)};
  const int num_threads = 3;
  const Hyperrectangle domain{Vector2d{0, 0}, Vector2d{1, 1}};
  const double point_in_set_tol = 1e-10;
  RandomGenerator generator(0);
  std::unique_ptr<PointSamplerBase> sampler =
      std::make_unique<UniformSetSampler<Hyperrectangle>>(domain, generator);

  CoverageCheckerViaBernoulliTest checker{alpha, num_points_per_check,
                                          std::move(sampler), num_threads,
                                          point_in_set_tol};

  ConvexSets current_sets;
  // An HPolyhedron comprising the lower left diagonal of the 1x1 box.
  const Eigen::Matrix<double, 3, 2> A{{-1, 0}, {0, -1}, {1, 1}};
  const Eigen::Vector3d b{0, 0, 1};
  current_sets.emplace_back(std::make_unique<HPolyhedron>(A, b));

  TestCoveredFractionWithinOneStandardDeviationOfExpected(
      checker, current_sets,
      0.5 /* Check that we achieve approximately 50% coverage.*/);

  /* Asking for 80% coverage when we only have 50% failse */
  checker.set_alpha(0.8);
  EXPECT_FALSE(checker.CheckCoverage(current_sets));

  // Asking for 25% coverage, when we have 50% coverage succeeds
  checker.set_alpha(0.25);
  EXPECT_TRUE(checker.CheckCoverage(current_sets));
}

/*
 * A 1x1 side length box, covered by a single of HPolyedron consisting of the
 * bottom left diagonal and a VPolytope which covers the entire domain.
 * This covers 100% of the box. This test checks that we can use multiple types
 * of sets to cover our domain and that the domains can overlap
 */
GTEST_TEST(BernoulliCoverageCheck, BoxDomainCoveredByMix) {
  const double alpha{0.9};
  const int num_points_per_check{static_cast<int>(10)};
  const int num_threads = 3;
  const Hyperrectangle domain{Vector2d{0, 0}, Vector2d{1, 1}};
  const double point_in_set_tol = 1e-10;
  RandomGenerator generator(0);
  std::unique_ptr<PointSamplerBase> sampler =
      std::make_unique<UniformSetSampler<Hyperrectangle>>(domain, generator);

  CoverageCheckerViaBernoulliTest checker{alpha, num_points_per_check,
                                          std::move(sampler), num_threads,
                                          point_in_set_tol};

  ConvexSets current_sets;
  // An HPolyhedron comprising the lower left diagonal of the 1x1 box.
  //  const Eigen::Matrix<double, 3, 2> A{{-1, 0}, {0, -1}, {1, 1}};
  //  const Eigen::Vector3d b{0, 0, 1};
  //  current_sets.emplace_back(std::make_unique<HPolyhedron>(A, b));
  // A VPolytope covering the entire 1x1 box.
  const Eigen::Matrix<double, 2, 4> vertices{{0, 0, 1, 1}, {0, 1, 0, 1}};
  current_sets.emplace_back(std::make_unique<VPolytope>(vertices));

  TestCoveredFractionWithinOneStandardDeviationOfExpected(
      checker, current_sets, 1 /* Check that we achieve 100% coverage.*/);
  // Asking for 90% coverage, when we have 100% coverage succeeds
  checker.set_alpha(0.9);
  EXPECT_TRUE(checker.CheckCoverage(current_sets));
  // Asking for 100% coverage, when we have 100% coverage succeeds
  checker.set_alpha(1.0);
  EXPECT_TRUE(checker.CheckCoverage(current_sets));
}

}  // namespace
}  // namespace planning
}  // namespace drake
