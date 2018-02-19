#include "drake/math/barycentric.h"

#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
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
  EXPECT_EQ(*(bary.get_input_grid()[2].rbegin()), 4.0);
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

  // Test the alternative call signature.
  EXPECT_TRUE(CompareMatrices(bary.get_mesh_point(0), Vector3d{0, 2, 3}));

  // Test the batch retrieval.
  const MatrixXd points = bary.get_all_mesh_points();
  EXPECT_EQ(points.cols(), 4);
  EXPECT_TRUE(CompareMatrices(points.col(3), Vector3d{1, 2, 4}));
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

  // Test the alternative call signature.
  EXPECT_NEAR(bary.Eval(mesh, Vector2d{.25, .75})[0], 6.25, 1e-8);
}

GTEST_TEST(BarycentricTest, EvalSymbolicTest) {
  BarycentricMesh<double> bary{{{0.0, 1.0},  // BR
                                {0.0, 1.0}}};

  using symbolic::Variable;
  using symbolic::Expression;
  Variable a{"a"}, b{"b"}, c{"c"}, d{"d"};
  RowVector4<Expression> mesh;
  mesh << a, b, c, d;

  Vector1<Expression> value;
  // Check grid points.
  bary.EvalWithMixedScalars<Expression>(mesh, Vector2d{0., 0.}, &value);
  EXPECT_TRUE(value[0].EqualTo(a));
  bary.EvalWithMixedScalars<Expression>(mesh, Vector2d{1., 0.}, &value);
  EXPECT_TRUE(value[0].EqualTo(b));
  bary.EvalWithMixedScalars<Expression>(mesh, Vector2d{0., 1.}, &value);
  EXPECT_TRUE(value[0].EqualTo(c));
  bary.EvalWithMixedScalars<Expression>(mesh, Vector2d{1., 1.}, &value);
  EXPECT_TRUE(value[0].EqualTo(d));

  // Check the middle.
  bary.EvalWithMixedScalars<Expression>(mesh, Vector2d{.5, .5}, &value);
  EXPECT_TRUE(value[0].EqualTo(.5 * a + .5 * d));

  // Test the alternative call signature.
  EXPECT_TRUE(
      bary.EvalWithMixedScalars<Expression>(mesh, Vector2d{0., 0.})[0].EqualTo(
          a));
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

Vector1d my_sine(const Eigen::Ref<const Vector1d>& x) {
  return Vector1d(std::sin(x[0]));
}

// Build a BarycentricMesh from a function pointer.
GTEST_TEST(BarycentricTest, FromVectorFunc) {
  BarycentricMesh<double>::Coordinates x_values{0, .1, .2, 5, 204};

  BarycentricMesh<double> bary({x_values});

  MatrixXd mesh_values = bary.MeshValuesFrom(&my_sine);

  // Check that it evaluates correctly on the grid (the interpolation is
  // verified with the other tests).
  Vector1d y_value;
  for (const auto& x : x_values) {
    bary.Eval(mesh_values, Vector1d(x), &y_value);
    EXPECT_EQ(y_value[0], std::sin(x));
  }
}

// Build a BarycentricMesh from a lambda expression.
GTEST_TEST(BarycentricTest, FromLambda) {
  BarycentricMesh<double>::Coordinates x_values{0, .1, .2, 5, 204};

  BarycentricMesh<double> bary({x_values});

  MatrixXd mesh_values = bary.MeshValuesFrom(
      [](const auto& x) { return Vector1d(std::sin(x[0])); });

  // Check that it evaluates correctly on the grid (the interpolation is
  // verified with the other tests).
  Vector1d y_value;
  for (const auto& x : x_values) {
    bary.Eval(mesh_values, Vector1d(x), &y_value);
    EXPECT_EQ(y_value[0], std::sin(x));
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
