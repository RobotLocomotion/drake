// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
// Adapted (okay, stolen) from http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain

#include <algorithm>
#include <vector>
#include <iostream>
#include <limits>
#include "convexHull.h"
using namespace std;
using namespace Eigen;
 
// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
coord2_t cross(const Point &O, const Point &A, const Point &B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}
 
// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
vector<Point> convexHull(vector<Point> P)
{

  int n = P.size(), k = 0;
  vector<Point> H(2*n);

  if (n == 2) {
    H.reserve(3);
    H.push_back(P[0]); 
    H.push_back(P[1]); 
    H.push_back(P[0]); 
    return H;
  }
 
  // Sort points lexicographically
  sort(P.begin(), P.end());

  // Build lower hull
  for (int i = 0; i < n; ++i) {
    while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
    H[k++] = P[i];
  }
 
  // Build upper hull
  for (int i = n-2, t = k+1; i >= 0; i--) {
    while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
    H[k++] = P[i];
  }
 
  H.resize(k);

  return H;
}

vector<Point> eigenToPoints(const Ref<const Matrix<double, 2, Dynamic>> &P) {
  vector<Point> points;
  points.reserve(P.cols());
  for (int i=0; i < P.cols(); ++i) {
    Point p;
    p.x = P(0, i);
    p.y = P(1, i);
    points.push_back(p);
  }
  return points;
}

bool inConvexHull(const Ref<const Matrix<double, 2, Dynamic>> &P, const Ref<const Vector2d> &q) {
  return signedDistanceInsideConvexHull(P, q) >= 0;
}

double signedDistanceInsideConvexHull(const Ref<const Matrix<double, 2, Dynamic>> &pts, const Ref<const Vector2d> &q) {
  vector<Point> hull_pts = convexHull(eigenToPoints(pts));
  double d_star = numeric_limits<double>::infinity();

  Matrix2d R;
  R << 0, 1, 
       -1, 0;

  for (int i=0; i < hull_pts.size(); ++i) {
    Vector2d ai = R * Vector2d(hull_pts[i+1].x - hull_pts[i].x, hull_pts[i+1].y - hull_pts[i].y);
    double b = ai.transpose() * Vector2d(hull_pts[i].x, hull_pts[i].y);
    double n = ai.norm();
    ai = ai.array() / n;
    b = b / n;
    double d = b - ai.transpose() * q;
    if (d < d_star) {
      d_star = d;
    }
  }
  return d_star;
}

