#include "drake/solvers/test/rotation_constraint_visualization.h"

#include "drake/common/proto/call_matlab.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/rotation_constraint_internal.h"

namespace drake {
namespace solvers {
namespace {
// Draw an arc between two end points on the unit sphere. The two end points
// are the vertices of the intersection region, between the box and the surface
// of the sphere.
// This requires that
// arc_end0(fixed_axis) = arc_end1(fixed_axis) = x(fixed_axis),
// where `x` is a point on the arc.
// Draws the shorter arc between the two points.
void DrawArcBoundaryOfBoxSphereIntersection(const Eigen::Vector3d& arc_end0,
                                            const Eigen::Vector3d& arc_end1,
                                            int fixed_axis,
                                            const Eigen::RowVector3d& color) {
  DRAKE_DEMAND(std::abs(arc_end0(fixed_axis) - arc_end1(fixed_axis)) < 1E-3);
  DRAKE_DEMAND(std::abs(arc_end0.norm() - 1) < 1E-3);
  DRAKE_DEMAND(std::abs(arc_end1.norm() - 1) < 1E-3);
  int free_axis0 = (fixed_axis + 1) % 3;
  int free_axis1 = (fixed_axis + 2) % 3;
  if (arc_end0(free_axis0) * arc_end1(free_axis0) < 0 ||
      arc_end0(free_axis1) * arc_end1(free_axis1) < 0) {
    // The two end points have to be in the same orthant.
    throw std::runtime_error(
        "The end points of the boundary arc are not in the same orthant.");
  }

  const int kNumViaPoints = 20;
  Eigen::Matrix<double, 3, kNumViaPoints> via_pts;
  via_pts.row(fixed_axis) =
      Eigen::Matrix<double, 1, kNumViaPoints>::Constant(arc_end0(fixed_axis));
  Eigen::Vector3d start_via_pts, end_via_pts;
  // Eigen::LinSpaced requires the smaller number being the first argument. So
  // find out whether arc_end0(free_axis0) or arc_end1(free_axis0) is smaller.
  if (arc_end0(free_axis0) < arc_end1(free_axis0)) {
    start_via_pts = arc_end0;
    end_via_pts = arc_end1;
  } else {
    start_via_pts = arc_end1;
    end_via_pts = arc_end0;
  }
  via_pts.row(free_axis0) = Eigen::Matrix<double, 1, kNumViaPoints>::LinSpaced(
      kNumViaPoints, start_via_pts(free_axis0), end_via_pts(free_axis0));
  via_pts(free_axis1, 0) = start_via_pts(free_axis1);
  via_pts(free_axis1, kNumViaPoints - 1) = end_via_pts(free_axis1);
  bool positive_free_axis1 =
      arc_end0(free_axis1) > 0 || arc_end1(free_axis1) > 0;
  for (int i = 1; i < kNumViaPoints - 1; ++i) {
    // A point `x` on the arc satisfies
    // x(free_axis0)^2 + x(free_axis1)^2 = 1 - x(fixed_axis)^2
    via_pts(free_axis1, i) = std::sqrt(1 - std::pow(via_pts(fixed_axis, i), 2) -
                                       std::pow(via_pts(free_axis0, i), 2));
    if (!positive_free_axis1) {
      via_pts(free_axis1, i) *= -1;
    }
  }
  auto h = common::CallMatlab(1, "plot3", via_pts.row(0), via_pts.row(1),
                              via_pts.row(2));
  common::CallMatlab("set", h[0], "Color", color);
}
}  // namespace

void DrawSphere(const Eigen::RowVector3d& color) {
  using common::CallMatlab;
  auto xyz_sphere = common::CallMatlab(3, "sphere", 40);
  auto h_sphere =
      CallMatlab(1, "surf", xyz_sphere[0], xyz_sphere[1], xyz_sphere[2]);
  CallMatlab("set", h_sphere[0], "FaceColor", color);
  CallMatlab("set", h_sphere[0], "FaceAlpha", 0.2);
  CallMatlab("set", h_sphere[0], "EdgeColor", color);
  CallMatlab("set", h_sphere[0], "LineStyle", "none");
}

void DrawBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax,
             const Eigen::RowVector3d& color) {
  using common::CallMatlab;
  if ((bmin.array() < 0).any()) {
    throw std::runtime_error("bmin should be in the first orthant in DrawBox.");
  }
  if ((bmax.array() < 0).any()) {
    throw std::runtime_error("bmax should be in the first orthant in DrawBox.");
  }
  if (bmin.norm() <= 1 && bmax.norm() >= 1) {
    // The box and the sphere has intersections.

    // Draw 6 planes
    for (int fixed_axis = 0; fixed_axis < 3; ++fixed_axis) {
      int free_axis0 = (fixed_axis + 1) % 3;
      int free_axis1 = (fixed_axis + 2) % 3;
      std::array<Eigen::Matrix2d, 3> plane;
      // For the free axes, one axis takes the mesh points
      // bmin bmin
      // bmax bmax
      // The other axis takes the mesh points
      // bmin bmax
      // bmin bmax
      // Please refer to MATLAB meshgrid function for more details
      // https://www.mathworks.com/help/matlab/ref/meshgrid.html
      plane[free_axis0] << Eigen::RowVector2d::Constant(bmin(free_axis0)),
          Eigen::RowVector2d::Constant(bmax(free_axis0));
      plane[free_axis1].col(0) = Eigen::Vector2d::Constant(bmin(free_axis1));
      plane[free_axis1].col(1) = Eigen::Vector2d::Constant(bmax(free_axis1));
      plane[fixed_axis] = Eigen::Matrix2d::Constant(bmin(fixed_axis));
      auto h0 = CallMatlab(1, "surf", plane[0], plane[1], plane[2]);
      plane[fixed_axis] = Eigen::Matrix2d::Constant(bmax(fixed_axis));
      auto h1 = CallMatlab(1, "surf", plane[0], plane[1], plane[2]);

      std::array<common::MatlabRemoteVariable, 2> h = {{h0[0], h1[0]}};
      for (int i = 0; i < 2; ++i) {
        CallMatlab("set", h[i], "FaceColor", color);
        CallMatlab("set", h[i], "FaceAlpha", 0.2);
        CallMatlab("set", h[i], "EdgeColor", color);
        CallMatlab("set", h[i], "LineStyle", "none");
      }
    }
  }
}

void DrawBoxSphereIntersection(const Eigen::Vector3d& bmin,
                               const Eigen::Vector3d& bmax,
                               const Eigen::RowVector3d& color) {
  DRAKE_DEMAND((bmax.array() > bmin.array()).all());
  // First convert bmin and bmax to the first orthant.
  Eigen::Vector3d orthant_bmin;
  Eigen::Vector3d orthant_bmax;
  // orthant_sign(i) = 1 if the i'th axis of the box is positive, otherwise it
  // is -1.
  Eigen::Vector3i orthant_sign;
  for (int i = 0; i < 3; ++i) {
    if (bmin(i) >= 0) {
      orthant_bmin(i) = bmin(i);
      orthant_bmax(i) = bmax(i);
      orthant_sign(i) = 1;
    } else if (bmax(i) <= 0) {
      orthant_bmin(i) = -bmax(i);
      orthant_bmax(i) = -bmin(i);
      orthant_sign(i) = -1;
    } else {
      throw std::runtime_error(
          "The box bmin <= x <= bmax should satisfy either bmin(i) >=0 or "
          "bmax(i) <= 0");
    }
  }
  // Compute the intersection points between the sphere in the first orthant,
  // with the box orthant_bmin <= x <= orthant_bmax
  auto intersection_pts = internal::ComputeBoxEdgesAndSphereIntersection(
      orthant_bmin, orthant_bmax);
  // Now convert the intersection point back to the right orthant.
  for (int i = 0; i < static_cast<int>(intersection_pts.size()); ++i) {
    for (int j = 0; j < 3; ++j) {
      intersection_pts[i](j) *= orthant_sign(j);
    }
  }
  // Draw the line that connects adjacent intersection points.
  // For each intersection point, find out the neighbouring points, and then
  // draw the arc between these two points. The neighbouring points should have
  // one axis same as the queried intersection point. Also, the neighbouring
  // points should be on the facet of the box as the queried intersection point.
  for (int i = 0; i < static_cast<int>(intersection_pts.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(intersection_pts.size()); ++j) {
      for (int dim = 0; dim < 3; ++dim) {
        if (std::abs(intersection_pts[i](dim) - intersection_pts[j](dim)) < 1E-3
            && (std::abs(intersection_pts[i](dim) - bmin(dim)) < 1E-3
                || std::abs(intersection_pts[i](dim) - bmax(dim)) < 1E-3)) {
          // Determine if two intersection points are the neighbouring boundary
          // points of the intersection region.
          DrawArcBoundaryOfBoxSphereIntersection(
              intersection_pts[i], intersection_pts[j], dim, color);
        }
      }
    }
  }
}
}  // namespace solvers
}  // namespace drake
