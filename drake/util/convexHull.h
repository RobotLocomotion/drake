#include <vector>
#include <Eigen/Dense>
#include "drake/drakeConvexHull_export.h"

typedef double coord_t;         // coordinate type
typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
 
struct DRAKECONVEXHULL_EXPORT Point {
  coord_t x, y;
 
  bool operator <(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }
};

DRAKECONVEXHULL_EXPORT std::vector<Point> convexHull(std::vector<Point> P);
DRAKECONVEXHULL_EXPORT bool inConvexHull(const Eigen::Ref<const Eigen::Matrix<double, 2, Eigen::Dynamic>> &P, const Eigen::Ref<const Eigen::Vector2d> &q, double tolerance=1e-16);

// Returns the perpendicular distance from point q to the convex hull of pts. 
// Specifically, if pts form a polytope defined by a_i'x <= b_i where each a_i is a unit vector, then this returns:
// 
// d* = min [b_i - a_i'q]
//       i 
// 
// If q is inside the convex hull of pts, then d* will be positive, else it will be negative. 
DRAKECONVEXHULL_EXPORT double signedDistanceInsideConvexHull(const Eigen::Ref<const Eigen::Matrix<double, 2, Eigen::Dynamic>> &pts, const Eigen::Ref<const Eigen::Vector2d> &q);