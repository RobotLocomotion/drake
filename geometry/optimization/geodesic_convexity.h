/** @file
A robot that has revolute joints without any limits has an inherently
non-Euclidean configuration space, but one can still consider
"geodesically-convex" sets, akin to convex sets in Euclidean space. In practice,
this only requires that the width of the set along each dimension corresponding
to an unbounded revolute joint be strictly less than π. These functions are
primarily used by GcsTrajectoryOptimization to make motion plans for these types
of robots.
*/

#pragma once

#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

/* Computes the minimum and maximum values that can be attained along a certain
dimension for a point constrained to lie within a convex set.
@throws std::exception if dimension is outside the interval
[0, region.ambient_dimension()). */
std::pair<double, double> GetMinimumAndMaximumValueAlongDimension(
    const ConvexSet& region, int dimension);

/* Helper function to assert that a given list of continuous revolute joint
indices satisfies the requirements for the constructor to
GcsTrajectoryOptimization, as well as any static functions that may take in
such a list. */
void ThrowsForInvalidContinuousJointsList(
    int num_positions, const std::vector<int>& continuous_revolute_joints);
}  // namespace internal

/** Given a convex set, and a list of indices corresponding to continuous
revolute joints, checks whether or not the set satisfies the convexity radius.
See §6.5.3 of "A Panoramic View of Riemannian Geometry", Marcel Berger for a
general definition of convexity radius. When dealing with continuous revolute
joints, respecting the convexity radius entails that each convex set has a width
of stricty less than π along each dimension corresponding to a continuous
revolute joint.
@throws std::exception if continuous_revolute_joints has repeated entries, or if
any entry is outside the interval [0, convex_set.ambient_dimension()). */
bool CheckIfSatisfiesConvexityRadius(
    const geometry::optimization::ConvexSet& convex_set,
    const std::vector<int>& continuous_revolute_joints);

/** Partitions a convex set into (smaller) convex sets whose union is the
original set and that each respect the convexity radius as in
CheckIfSatisfiesConvexityRadius. In practice, this is implemented as
partitioning sets into pieces whose width is less than or equal to π-ϵ. Each
entry in continuous_revolute_joints must be non-negative, less than
num_positions, and unique.
@param epsilon is the ϵ value used for the convexity radius inequality. The
partitioned sets are made by intersecting convex_set with axis-aligned
bounding boxes that respect the convexity radius. These boxes are made to
overlap by ϵ radians along each dimension, for numerical purposes.
@return the vector of convex sets that each respect convexity radius.
@throws std::exception if ϵ <= 0 or ϵ >= π.
@throws std::exception if the input convex set is unbounded along dimensions
corresponding to continuous revolute joints.
@throws std::exception if continuous_revolute_joints has repeated entries, or if
any entry is outside the interval [0, convex_set.ambient_dimension()). */
geometry::optimization::ConvexSets PartitionConvexSet(
    const geometry::optimization::ConvexSet& convex_set,
    const std::vector<int>& continuous_revolute_joints,
    const double epsilon = 1e-5);

/* Function overload to take in a list of convex sets, and partition all so as
to respect the convexity radius. Every set must be bounded and have the same
ambient dimension. Each entry in continuous_revolute_joints must be
non-negative, less than num_positions, and unique.
@throws std::exception unless every ConvexSet in convex_sets has the same
ambient_dimension.
@throws std::exception if ϵ <= 0 or ϵ >= π.
@throws std::exception if any input convex set is unbounded along dimensions
corresponding to continuous revolute joints.
@throws std::exception if continuous_revolute_joints has repeated entries, or if
any entry is outside the interval [0, ambient_dimension). */
geometry::optimization::ConvexSets PartitionConvexSet(
    const geometry::optimization::ConvexSets& convex_sets,
    const std::vector<int>& continuous_revolute_joints,
    const double epsilon = 1e-5);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
