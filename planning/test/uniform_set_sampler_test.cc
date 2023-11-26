#include "drake/planning/uniform_set_sampler.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"

namespace drake {
namespace planning {
namespace {
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperrectangle;

GTEST_TEST(UniformSetSampler, UniformHPolyhedronSamplerDefaultCtor) {
  const Eigen::Matrix<double, 3, 2> A{{-1, 0}, {0, -1}, {1, 1}};
  const Eigen::Vector3d b{0, 0, 1};
  const HPolyhedron polyhedron{A, b};
  RandomGenerator generator;

  UniformSetSampler<HPolyhedron> sampler{polyhedron};

  const int num_samples = 10;

  const Eigen::MatrixXd samples_from_sampler =
      sampler.SamplePoints(num_samples);
  Eigen::MatrixXd samples_from_polyhedron(2, num_samples);
  for (int i = 0; i < num_samples; ++i) {
    samples_from_polyhedron.col(i) = polyhedron.UniformSample(&generator);
  }
  EXPECT_TRUE(
      CompareMatrices(samples_from_sampler, samples_from_polyhedron, 1e-12));
}

GTEST_TEST(UniformSetSampler, UniformHPolyhedronSamplerManualSeed) {
  const Eigen::Matrix<double, 3, 2> A{{-1, 0}, {0, -1}, {1, 1}};
  const Eigen::Vector3d b{0, 0, 1};
  const HPolyhedron polyhedron{A, b};
  RandomGenerator generator(0);

  UniformSetSampler<HPolyhedron> sampler{polyhedron, generator};

  const int num_samples = 10;

  const Eigen::MatrixXd samples_from_sampler =
      sampler.SamplePoints(num_samples);
  Eigen::MatrixXd samples_from_polyhedron(2, num_samples);
  for (int i = 0; i < num_samples; ++i) {
    samples_from_polyhedron.col(i) = polyhedron.UniformSample(&generator);
  }
  EXPECT_TRUE(
      CompareMatrices(samples_from_sampler, samples_from_polyhedron, 1e-12));
}

GTEST_TEST(UniformSetSampler, UniformHyperrectangleSamplerDefaultCtor) {
  const Eigen::Vector2d lb{-2,-1};
  const Eigen::Vector2d ub{-2,-1};
  const Hyperrectangle rectangle{lb, ub};
  RandomGenerator generator(0);

  UniformSetSampler<Hyperrectangle> sampler{rectangle, generator};

  const int num_samples = 10;

  const Eigen::MatrixXd samples_from_sampler =
      sampler.SamplePoints(num_samples);
  Eigen::MatrixXd samples_from_polyhedron(2, num_samples);
  for (int i = 0; i < num_samples; ++i) {
    samples_from_polyhedron.col(i) = rectangle.UniformSample(&generator);
  }
  EXPECT_TRUE(
      CompareMatrices(samples_from_sampler, samples_from_polyhedron, 1e-12));
}

GTEST_TEST(UniformSetSampler, UniformHyperrectangleSamplerManualSeed) {
  const Eigen::Vector2d lb{-2,-1};
  const Eigen::Vector2d ub{-2,-1};
  const Hyperrectangle rectangle{lb, ub};
  RandomGenerator generator(0);

  UniformSetSampler<Hyperrectangle> sampler{rectangle, generator};

  const int num_samples = 10;

  const Eigen::MatrixXd samples_from_sampler =
      sampler.SamplePoints(num_samples);
  Eigen::MatrixXd samples_from_polyhedron(2, num_samples);
  for (int i = 0; i < num_samples; ++i) {
    samples_from_polyhedron.col(i) = rectangle.UniformSample(&generator);
  }
  EXPECT_TRUE(
      CompareMatrices(samples_from_sampler, samples_from_polyhedron, 1e-12));
}

}  // namespace
}  // namespace planning
}  // namespace drake