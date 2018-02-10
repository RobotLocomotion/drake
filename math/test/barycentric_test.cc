#include "drake/math/barycentric.h"

#include <memory>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector3i;

GTEST_TEST(BarycentricTest, GetMeshPoints) {
  // Create a mesh in 3 (input) dimensions, with the second dimension being a
  // singleton.
  const int kNumInputs = 3;
  BarycentricMesh<double> bary{{{0.0, 1.0},  // BR
                                {2.0},       // BR
                                {3.0, 4.0}}};

  EXPECT_EQ(bary.get_input_size(), kNumInputs);
  EXPECT_EQ(bary.get_num_mesh_points(), 4);

  Vector3d point;
  bary.get_mesh_point(0, &point);
  EXPECT_TRUE(CompareMatrices(point, Vector3d{0, 2, 3}));
  bary.get_mesh_point(1, &point);
  EXPECT_TRUE(CompareMatrices(point, Vector3d{1, 2, 3}));
  bary.get_mesh_point(2, &point);
  EXPECT_TRUE(CompareMatrices(point, Vector3d{0, 2, 4}));
  bary.get_mesh_point(3, &point);
  EXPECT_TRUE(CompareMatrices(point, Vector3d{1, 2, 4}));
}

GTEST_TEST(BarycentricTest, EvalWeights) {
  // Create a mesh in 3 (input) dimensions, with the second dimension being a
  // singleton.
  BarycentricMesh<double> bary{{{0.0, 1.0},  // BR
                                {2.0},       // BR
                                {3.0, 4.0}}};

  VectorXi indices(3);
  VectorXd weights(3);

  Vector3d sample{1, 2, 3.1};
  bary.EvalBarycentricWeights(sample, &indices, &weights);
  EXPECT_TRUE(CompareMatrices(indices, Vector3i{3, 1, 0}));
  EXPECT_TRUE(CompareMatrices(weights, Vector3d{.1, .9, 0}, 1e-8));

  // Off the grid (in singleton dimension) should not change things.
  sample[1] = 3.0;
  bary.EvalBarycentricWeights(sample, &indices, &weights);
  EXPECT_TRUE(CompareMatrices(indices, Vector3i{3, 1, 0}));
  EXPECT_TRUE(CompareMatrices(weights, Vector3d{.1, .9, 0}, 1e-8));

  // Off the grid to the right.
  sample[0] = 1.5;
  bary.EvalBarycentricWeights(sample, &indices, &weights);
  EXPECT_TRUE(CompareMatrices(indices, Vector3i{3, 1, 1}));
  EXPECT_TRUE(CompareMatrices(weights, Vector3d{.1, .9, 0}, 1e-8));

  // Test a different face.
  sample = Vector3d{0., 2.0, 3.4};
  bary.EvalBarycentricWeights(sample, &indices, &weights);
  EXPECT_TRUE(CompareMatrices(indices, Vector3i{2, 0, 0}));
  EXPECT_TRUE(CompareMatrices(weights, Vector3d{.4, .6, 0.}, 1e-8));

  // Off the grid to the left should not change things.
  sample[0] = -1.5;
  bary.EvalBarycentricWeights(sample, &indices, &weights);
  EXPECT_TRUE(CompareMatrices(indices, Vector3i{2, 0, 0}));
  EXPECT_TRUE(CompareMatrices(weights, Vector3d{.4, .6, 0}, 1e-8));

  // Smack in the middle.
  sample = Vector3d{.5, 2., 3.5};
  bary.EvalBarycentricWeights(sample, &indices, &weights);
  EXPECT_TRUE(CompareMatrices(indices, Vector3i{3, 2, 0}));
  EXPECT_TRUE(CompareMatrices(weights, Vector3d{.5, 0, .5}, 1e-8));
}

GTEST_TEST(BarycentricTest, EvalTest) {
  BarycentricMesh<double> bary{{{0.0, 1.0},  // BR
                                {0.0, 1.0}}};

  MatrixXd mesh = Eigen::RowVector4d{1., 2., 3., 4.};

  Vector1d value;
  double tol = 1e-8;
  // Check grid points.
  bary.Eval(mesh, Vector2d{0., 0.}, &value);
  EXPECT_NEAR(value[0], 1., tol);
  bary.Eval(mesh, Vector2d{1., 0.}, &value);
  EXPECT_NEAR(value[0], 2., tol);
  bary.Eval(mesh, Vector2d{0., 1.}, &value);
  EXPECT_NEAR(value[0], 3., tol);
  bary.Eval(mesh, Vector2d{1., 1.}, &value);
  EXPECT_NEAR(value[0], 4., tol);

  // Check the middle.
  bary.Eval(mesh, Vector2d{.5, .5}, &value);
  EXPECT_NEAR(value[0], 2.5, 1e-8);

  // Check the two faces.
  bary.Eval(mesh, Vector2d{.75, .25}, &value);
  EXPECT_NEAR(value[0], 2.25, 1e-8);
  bary.Eval(mesh, Vector2d{.25, .75}, &value);
  EXPECT_NEAR(value[0], 2.75, 1e-8);

  // Lift a corner and check again.
  mesh(0, 2) = 10.;
  bary.Eval(mesh, Vector2d{.25, .75}, &value);
  EXPECT_NEAR(value[0], 6.25, 1e-8);
}

GTEST_TEST(BarycentricTest, MultidimensionalOutput) {
  BarycentricMesh<double> bary{{{0.0, 1.0},  // BR
                                {0.0, 1.0}}};

  const int kNumOutputs = 2;
  MatrixXd mesh(kNumOutputs, bary.get_num_mesh_points());
  mesh << 1., 2., 3., 4.,  // BR
      5., 6., 7., 8.;

  Vector2d value;
  // Check the two faces.
  bary.Eval(mesh, Vector2d{.75, .25}, &value);
  EXPECT_TRUE(CompareMatrices(value, Vector2d{2.25, 6.25}, 1e-8));
  bary.Eval(mesh, Vector2d{.25, .75}, &value);
  EXPECT_TRUE(CompareMatrices(value, Vector2d{2.75, 6.75}, 1e-8));
}

}  // namespace
}  // namespace math
}  // namespace drake
