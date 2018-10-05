#pragma once

#include <vector>

#include <Eigen/Core>

// This file only exists to expose some internal methods for unit testing.  It
// should NOT be included in user code.
// The API documentation for these functions lives in
// mixed_integer_rotation_constraint_internal.cc, where they are implemented.
namespace drake {
namespace solvers {
namespace internal {
/**
 * Given an axis-aligned box in the first orthant, computes and returns all the
 * intersecting points between the edges of the box and the unit sphere.
 * @param bmin  The vertex of the box closest to the origin.
 * @param bmax  The vertex of the box farthest from the origin.
 */
std::vector<Eigen::Vector3d> ComputeBoxEdgesAndSphereIntersection(
    const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax);

/*
 * For the intersection region between the surface of the unit sphere, and the
 * interior of a box aligned with the axes, use a half space relaxation for
 * the intersection region as nᵀ * v >= d
 * @param[in] pts. The vertices containing the intersecting points between edges
 * of the box and the surface of the unit sphere.
 * @param[out] n. The unit length normal vector of the halfspace, pointing
 * outward.
 * @param[out] d. The intercept of the halfspace.
 */
void ComputeHalfSpaceRelaxationForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d* n, double* d);

/**
 * For the vertices in `pts`, determine if these vertices are co-planar. If they
 * are, then compute that plane nᵀ * x = d.
 * @param pts The vertices to be checked.
 * @param n The unit length normal vector of the plane, points outward from the
 * origin. If the vertices are not co-planar, leave `n` to 0.
 * @param d The intersecpt of the plane. If the vertices are not co-planar, set
 * this to 0.
 * @return If the vertices are co-planar, set this to true. Otherwise set to
 * false.
 */
bool AreAllVerticesCoPlanar(const std::vector<Eigen::Vector3d>& pts,
                            Eigen::Vector3d* n, double* d);

/**
 * For the intersection region between the surface of the unit sphere, and the
 * interior of a box aligned with the axes, relax this nonconvex intersection
 * region to its convex hull. This convex hull has some planar facets (formed
 * by the triangles connecting the vertices of the intersection region). This
 * function computes these planar facets. It is guaranteed that any point x on
 * the intersection region, satisfies A * x <= b.
 * @param[in] pts The vertices of the intersection region. Same as the `pts` in
 * ComputeHalfSpaceRelaxationForBoxSphereIntersection()
 * @param[out] A The rows of A are the normal vector of facets. Each row of A is
 * a unit length vector.
 * @param b b(i) is the interscept of the i'th facet.
 * @pre pts[i] are all in the first orthant, namely (pts[i].array() >=0).all()
 * should be true.
 */
void ComputeInnerFacetsForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts,
    Eigen::Matrix<double, Eigen::Dynamic, 3>* A, Eigen::VectorXd* b);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
