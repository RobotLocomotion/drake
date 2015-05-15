#include <iostream>
#include "convexHull.h"
#include "testUtil.h"

using namespace Eigen;

int testConvexHull() {
  Matrix<double, 2, Dynamic> pts(2, 4);
  pts << 1, 2, 3, 2,
         2, 1, 2, 3;

  if (!inConvexHull(pts, Vector2d(2, 2))) {
    fprintf(stderr, "2,2 should be in hull\n");
    return 1;
  }

  if (!inConvexHull(pts, Vector2d(1.0001, 2))) {
    fprintf(stderr, "1.0001, 2 should be in hull\n");
    return 1;
  }
  if (inConvexHull(pts, Vector2d(0.9999, 2))) {
    fprintf(stderr, "0.9999, 2 should not be in hull\n");
    return 1;
  }
  if (!inConvexHull(pts, Vector2d(2.49, 2.49))) {
    fprintf(stderr, "2.49,2.49 should be in hull\n");
    return 1;
  }
  if (inConvexHull(pts, Vector2d(2.51, 2.51))) {
    fprintf(stderr, "2.51,2.51 should not be in hull\n");
    return 1;
  }

  return 0;
}

void testDistanceFromHull() {
  Matrix<double, 2, Dynamic> pts(2, 5);
  pts << 0, 1, 1, 0, 0.5,
         0, 1, 0, 1, 0.5;

  double d = signedDistanceInsideConvexHull(pts, Vector2d(0, 0));
  valuecheck(d, 0.0, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(0.5, 0.5));
  valuecheck(d, 0.5, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(-0.5, 0));
  valuecheck(d, -0.5, 1e-8);

  pts << 0, 2, 2, 0, 0.5,
         0, 2, 0, 2, 0.5;

  d = signedDistanceInsideConvexHull(pts, Vector2d(0, 0));
  valuecheck(d, 0.0, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(0.5, 0.5));
  valuecheck(d, 0.5, 1e-8);

  d = signedDistanceInsideConvexHull(pts, Vector2d(-0.5, 0));
  valuecheck(d, -0.5, 1e-8);
}

void testRealData() {
  Matrix<double, 2, Dynamic> pts(2, 16);
  pts << 0.237506,    0.330077,    0.297687,    0.390258,    0.177325,    0.269896,    0.357868,    0.450439,  0.00257912,    0.116144,   0.0466612,    0.160226,    -0.03475,   0.0788149,   0.0873669,    0.200932,
        -0.00459488,   -0.093748,   0.0579462,  -0.0312069,   -0.067136,   -0.156289,    0.120487,   0.0313342,    0.102483,   0.0421703,    0.185504,    0.125191,   0.0321808,  -0.0281323,    0.262166,    0.201853;
  Vector2d q(0.196956, 0.0487772);

  double d = signedDistanceInsideConvexHull(pts, q);
  valuecheck(d, 0.136017, 1e-6);
}


int main() {
  bool failed = false;
  int error;

  error = testConvexHull();
  if (error) {
    std::cout << "testConvexHull FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testConvexHull passed" << std::endl;
  }

  testDistanceFromHull();
  std::cout << "testDistanceFromHull passed" << std::endl;

  testRealData();
  std::cout << "testRealData passed" << std::endl;

  if (!failed) {
    std::cout << "convexHull tests passed" << std::endl;
  } else {
    std::cout << "convexHull tests FAILED" << std::endl;
    exit(1);
  }
}
