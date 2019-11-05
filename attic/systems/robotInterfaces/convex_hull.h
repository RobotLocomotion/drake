#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_deprecated.h"

typedef double coord_t;   // coordinate type
typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2

struct DRAKE_DEPRECATED("2020-02-01",
    "The robotInterfaces package is being removed.")
Point {
  coord_t x, y;

  bool operator<(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }
};

DRAKE_DEPRECATED("2020-02-01", "The robotInterfaces package is being removed.")
std::vector<Point> convexHull(std::vector<Point> P);

DRAKE_DEPRECATED("2020-02-01", "The robotInterfaces package is being removed.")
bool inConvexHull(
    const Eigen::Ref<const Eigen::Matrix<double, 2, Eigen::Dynamic>> &P,
    const Eigen::Ref<const Eigen::Vector2d> &q, double tolerance = 1e-16);

// Returns the perpendicular distance from point q to the convex hull of pts.
// Specifically, if pts form a polytope defined by a_i'x <= b_i where each a_i
// is a unit vector, then this returns:
//
// d* = min [b_i - a_i'q]
//       i
//
// If q is inside the convex hull of pts, then d* will be positive, else it will
// be negative.
DRAKE_DEPRECATED("2020-02-01", "The robotInterfaces package is being removed.")
double signedDistanceInsideConvexHull(
    const Eigen::Ref<const Eigen::Matrix<double, 2, Eigen::Dynamic>> &pts,
    const Eigen::Ref<const Eigen::Vector2d> &q);
