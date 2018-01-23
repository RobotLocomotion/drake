#pragma once

#include <vector>

// This file only exists to expose some internal methods for unit testing.  It
// should NOT be included in user code.
// The API documentation for these functions lives in rotation_constraint.cc,
// where they are implemented.
namespace drake {
namespace solvers {
namespace internal {

std::vector<Eigen::Vector3d> ComputeBoxEdgesAndSphereIntersection(
    const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax);

void ComputeHalfSpaceRelaxationForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d* n, double* d);

bool AreAllVerticesCoPlanar(const std::vector<Eigen::Vector3d>& pts,
                            Eigen::Vector3d* n, double* d);

void ComputeInnerFacetsForBoxSphereIntersection(
    const std::vector<Eigen::Vector3d>& pts,
    Eigen::Matrix<double, Eigen::Dynamic, 3>* A, Eigen::VectorXd* b);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
