#include "drake/util/convexHull.h"

#include <iostream>

#include <gtest/gtest.h>

using namespace Eigen;

GTEST_TEST(TestConvexHull, testConvexHull) {
  Matrix<double, 2, Dynamic> pts(2, 4);
  pts << 1.0, 2.0, 3.0, 2.0, 2.0, 1.0, 2.0, 3.0;

  EXPECT_TRUE(inConvexHull(pts, Vector2d(2.0, 2.0)));
  EXPECT_TRUE(inConvexHull(pts, Vector2d(1.0001, 2.0)));
  EXPECT_FALSE(inConvexHull(pts, Vector2d(0.9999, 2.0)));
  EXPECT_TRUE(inConvexHull(pts, Vector2d(2.49, 2.49)));
  EXPECT_FALSE(inConvexHull(pts, Vector2d(2.51, 2.51)));
}

GTEST_TEST(TestConvexHull, testDistanceFromHull) {
  Matrix<double, 2, Dynamic> pts(2, 5);
  pts << 0, 1, 1, 0, 0.5, 0, 1, 0, 1, 0.5;

  double d = signedDistanceInsideConvexHull(pts, Vector2d(0, 0));
  EXPECT_NEAR(d, 0.0, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(0.5, 0.5));
  EXPECT_NEAR(d, 0.5, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(-0.5, 0));
  EXPECT_NEAR(d, -0.5, 1e-8);

  pts << 0, 2, 2, 0, 0.5, 0, 2, 0, 2, 0.5;

  d = signedDistanceInsideConvexHull(pts, Vector2d(0, 0));
  EXPECT_NEAR(d, 0.0, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(0.5, 0.5));
  EXPECT_NEAR(d, 0.5, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(-0.5, 0));
  EXPECT_NEAR(d, -0.5, 1e-8);
}

GTEST_TEST(TestConvexHull, testRealData) {
  // A real example that we once got wrong (due to an indexing bug in
  // signedDistanceInsideConvexHull)
  Matrix<double, 2, Dynamic> pts(2, 16);
  pts << 0.237506, 0.330077, 0.297687, 0.390258, 0.177325, 0.269896, 0.357868,
      0.450439, 0.00257912, 0.116144, 0.0466612, 0.160226, -0.03475, 0.0788149,
      0.0873669, 0.200932, -0.00459488, -0.093748, 0.0579462, -0.0312069,
      -0.067136, -0.156289, 0.120487, 0.0313342, 0.102483, 0.0421703, 0.185504,
      0.125191, 0.0321808, -0.0281323, 0.262166, 0.201853;
  Vector2d q(0.196956, 0.0487772);

  double d = signedDistanceInsideConvexHull(pts, q);
  EXPECT_NEAR(d, 0.136017, 1e-6);
}

GTEST_TEST(TestConvexHull, testDuplicates) {
  Matrix<double, 2, Dynamic> pts(2, 8);
  pts << 0.0, 1.0, 1.0, 0.0, 0.5, 0.0, 0.0, 1.0 - 1e-16, 0.0, 1.0, 0.0, 1.0,
      0.5, 0.0, 0.0, 1.0 - 1e-16;
  double d = signedDistanceInsideConvexHull(pts, Vector2d(0, 0));
  EXPECT_NEAR(d, 0.0, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(0.5, 0.5));
  EXPECT_NEAR(d, 0.5, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(-0.5, 0));
  EXPECT_NEAR(d, -0.5, 1e-8);
}

GTEST_TEST(TestConvexHull, testRandomConvexCombinations) {
  // Generate a set of points, then find a random convex combination of those
  // points, and verify that it's correctly reported as being inside the
  // convex hull
  for (int i = 2; i < 50; ++i) {
    for (int j = 0; j < 500; ++j) {
      MatrixXd pts = MatrixXd::Random(2, i);
      VectorXd weights = VectorXd::Random(i);
      if (weights.minCoeff() < 0) {
        weights = weights.array() -
                  weights.minCoeff();  // make sure they're all nonnegative
      }
      weights = weights.array() / weights.sum();
      Vector2d q = pts * weights;
      EXPECT_TRUE(inConvexHull(pts, q, 1e-8));
    }
  }
}

GTEST_TEST(TestConvexHull, testRandomTriangles) {
  // signed distance to convex hull should always be 0 for any vertex of a
  // triangle
  for (int j = 0; j < 500; ++j) {
    MatrixXd pts = MatrixXd::Random(2, 3);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(signedDistanceInsideConvexHull(pts, pts.col(i)), 0, 1e-8);
    }
  }
}
