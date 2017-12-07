#pragma once

#include <Eigen/Core>
namespace drake {
namespace solvers {
/**
 * Draw a unit sphere
 * @param color The rgb color of the surface to be plotted. @default is [0, 0.5,
 * 0.5]
 */
void DrawSphere(const Eigen::RowVector3d& color = Eigen::RowVector3d(0, 0.5,
                                                                     0.5));

/**
 * Draw the box bmin <= x <= bmax, where the inequality is elementwise.
 * @param bmin The innermost corner of the box
 * @param bmax The outermost corner of the box
 * @param color The rgb color of the surface to be plotted.
 * @default is [0.5, 0.2, 0.3]
 * @pre bmin >= 0 and bmax >= 0. The inequality is elementwise.
 */
void DrawBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax,
             const Eigen::RowVector3d& color = Eigen::RowVector3d(0.5, 0.2,
                                                                  0.3));

/**
 * Draw the boundary of the intersection region, between the box
 * bmin <= x <= bmax, and the unit sphere.
 * Currently we only accept the box in the first orthant.
 * @param bmin The innermost corner of the box.
 * @param bmax The outermost corner of the box.
 * @param color The rgb color of the boundary to be plotted. @default is red.
 */
void DrawBoxSphereIntersection(
    const Eigen::Vector3d &bmin,
    const Eigen::Vector3d &bmax,
    const Eigen::RowVector3d& color = Eigen::RowVector3d(1, 0, 0));
}  // namespace solvers
}  // namespace drake
