/** \defgroup geometry_optimization_geodesic_convexity Geodesic Convexity
@ingroup geometry_optimization

@brief A robot that has revolute joints without any limits has an inherently
non-Euclidean configuration space, but one can still consider
"geodesically-convex" sets, akin to convex sets in Euclidean space. In practice,
this only requires that the width of the set along each dimension corresponding
to an unbounded revolute joint be strictly less than π. These functions are
primarily used by GcsTrajectoryOptimization to make motion plans for these types
of robots. */

#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

/* Computes the minimum and maximum values that can be attained along a certain
dimension for a point constrained to lie within a convex set region.
@param region is the convex set we're examining.
@param dimension specifies for which dimension we compute the minimum and
maximum values.
@throws std::exception if dimension is outside the interval
[0, region.ambient_dimension()). */
std::pair<double, double> GetMinimumAndMaximumValueAlongDimension(
    const ConvexSet& region, int dimension);

/* Convenience overload to find the minimum and maximum values that can be
attained along a list of dimensions for a point constrained to lie within a
convex set.
@param region is the convex set we're examining.
@param dimensions specifies a list of dimensions for which we compute the
minimum and maximum values.
@returns a vector of pairs, where the entry in index i is the pair (min, max)
describing the minimum and maximum value along the dimension given by index i
of the input dimensions vector.
@throws std::exception if any entry of dimensions is outside the interval
[0, region.ambient_dimension()). */
std::vector<std::pair<double, double>> GetMinimumAndMaximumValueAlongDimension(
    const ConvexSet& region, std::vector<int> dimensions);

/* Helper function to assert that a given list of continuous revolute joint
indices satisfies the requirements for the constructor to
GcsTrajectoryOptimization, as well as any static functions that may take in
such a list. */
void ThrowsForInvalidContinuousJointsList(
    int num_positions, const std::vector<int>& continuous_revolute_joints);

/* Given two convex sets A and B with ambient dimension num_positions, we denote
a subset of those positions as continuous_revolute_joints, and the bounding box
along these positions is obtained via GetMinimumAndMaximumValueAlongDimension().
This function takes in the bounding boxes, and computes the translation that
should be applied to the first convex set to align it with the second, such that
intersections can directly be checked without considering the 2π wraparound that
may occur with continuous revolute joints.
@throws std::exception if continuous_bbox_A.size() != continuous_bbox_B.size().
@throws std::exception if num_positions and continuous_revolute_joints do not
satisfy the conditions checked by
internal::ThrowsForInvalidContinuousJointsList. */
Eigen::VectorXd ComputeOffsetContinuousRevoluteJoints(
    const int num_positions, const std::vector<int>& continuous_revolute_joints,
    const std::vector<std::pair<double, double>>& continuous_bbox_A,
    const std::vector<std::pair<double, double>>& continuous_bbox_B);
}  // namespace internal

/** \addtogroup geometry_optimization_geodesic_convexity
@{ */

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

// TODO(@cohnt): Call the following function "PartitionConvexSets" instead.
// Currently // the name is misleading and causes problem in the documentation.
// see https://github.com/RobotLocomotion/drake/issues/21046

/** Function overload to take in a list of convex sets, and partition all so as
to respect the convexity radius. Every set must be bounded and have the same
ambient dimension. Each entry in continuous_revolute_joints must be
non-negative, less than num_positions, and unique.
@throws std::exception unless every ConvexSet in convex_sets has the same
ambient_dimension.
@throws std::exception if ϵ <= 0 or ϵ >= π.
@throws std::exception if any input convex set is unbounded along dimensions
corresponding to continuous revolute joints.
@throws std::exception if continuous_revolute_joints has repeated entries, or if
any entry is outside the interval [0, ambient_dimension).
@throws if any entry of `convex_sets` is a nullptr.
*/
geometry::optimization::ConvexSets PartitionConvexSet(
    const geometry::optimization::ConvexSets& convex_sets,
    const std::vector<int>& continuous_revolute_joints,
    const double epsilon = 1e-5);

/** Computes the pairwise intersections between two lists of convex sets,
returning a list of edges, and a list of their corresponding offsets. Each edge
is a tuple in the form [index_A, index_B], where index_A is the index of the set
in `convex_sets_A` and index_B is the index of the set in `convex_sets_B`. The
corresponding entry in the list of offsets (i.e., the entry at the same index)
is the translation that is applied to all the points in the index_A'th set in
`convex_sets_A` to align them with the index_B'th set in `convex_sets_B`. This
translation may only have non-zero entries along the dimensions corresponding to
`continuous_revolute_joints`. All non-zero entries are integer multiples of 2π
as the translation of the sets still represents the same configurations for the
indices in `continuous_revolute_joints`.

@param convex_sets_A is a vector of convex sets. Pairwise intersections will be
computed between `convex_sets_A` and `convex_sets_B`.
@param convex_sets_B is the other vector of convex sets.
@param continuous_revolute_joints is a list of joint indices corresponding to
continuous revolute joints.
@param preprocess_bbox is a flag for whether the function should precompute
axis-aligned bounding boxes (AABBs) for every set. This can speed up the
pairwise intersection checks, by determining some sets to be disjoint without
needing to solve an optimization problem. However, it does require some overhead
to compute those bounding boxes.
@param parallelism specifies the number of threads to use.

@throws if `continuous_revolute_joints` has repeated entries, or if any entry
is outside the interval [0, ambient_dimension), where ambient_dimension is the
ambient dimension of the convex sets in `convex_sets_A` and `convex_sets_B`.
@throws if `convex_sets_A` or `convex_sets_B` are empty.
@throws if any entry of `convex_sets_A` or `convex_sets_B` is a nullptr.
*/
std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(const ConvexSets& convex_sets_A,
                             const ConvexSets& convex_sets_B,
                             const std::vector<int>& continuous_revolute_joints,
                             bool preprocess_bbox = true,
                             Parallelism parallelism = Parallelism::Max());

/** Overload of `ComputePairwiseIntersections` allowing the user to supply axis-
aligned bounding boxes if they're known a priori, to save on computation time.

@param bboxes_A is a vector of Hyperrectangles, allowing the user to manually
pass in the AABBs of each set in `convex_sets_A` to avoid recomputation.
@param bboxes_B serves the same role to `convex_sets_B` as `bboxes_A` does to
`convex_sets_A`.

@warning The function does not check that the entries of bboxes_A are indeed the
AABBs corresponding to the sets in `convex_sets_A` (and likewise for bboxes_B).

@throws if `convex_sets_A.size() != bboxes_A.size()`
@throws if `convex_sets_B.size() != bboxes_B.size()`
@throws if not all entries of `convex_sets_A`, `convex_sets_B`, `bboxes_A`, and
`bboxes_B` have the same ambient dimension.
@throws if any entry of `convex_sets_A` or `convex_sets_B` is a nullptr.
*/
std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(
    const ConvexSets& convex_sets_A, const ConvexSets& convex_sets_B,
    const std::vector<int>& continuous_revolute_joints,
    const std::vector<geometry::optimization::Hyperrectangle>& bboxes_A,
    const std::vector<geometry::optimization::Hyperrectangle>& bboxes_B,
    Parallelism parallelism = Parallelism::Max());

/** Convenience overload to compute pairwise intersections within a list of
convex sets. Equivalent to calling ComputePairwiseIntersections(convex_sets,
convex_sets, continuous_revolute_joints).

@param convex_sets is a vector of convex sets. Pairwise intersections will be
computed within `convex_sets`.
@param continuous_revolute_joints is a list of joint indices corresponding to
continuous revolute joints.
@param preprocess_bbox is a flag for whether the function should precompute
axis-aligned bounding boxes for every set. This can speed up the pairwise
intersection checks, by determining some sets to be disjoint without needing
to solve an optimization problem.
@param parallelism specifies the number of threads to use.

@throws if `continuous_revolute_joints` has repeated entries, or if any entry
is outside the interval [0, ambient_dimension), where ambient_dimension is the
ambient dimension of the convex sets in `convex_sets`.
@throws if `convex_sets` is empty.
@throws if any entry of `convex_sets` is a nullptr.
*/
std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(const ConvexSets& convex_sets,
                             const std::vector<int>& continuous_revolute_joints,
                             bool preprocess_bbox = true,
                             Parallelism parallelism = Parallelism::Max());

/** Overload of `ComputePairwiseIntersections` allowing the user to supply axis-
aligned bounding boxes if they're known a priori, to save on computation time.

@param bboxes is a vector of Hyperrectangles, allowing the user to manually pass
in the AABBs of each set in `convex_sets` to avoid recomputation.

@warning The function does not check that the entries are indeed the AABBs
corresponding to the sets in `convex_sets`.

@throws if `convex_sets.size() != bboxes.size()`
@throws if not all entries of `convex_sets` and `bboxes` have the same
ambient dimension.
@throws if any entry of `convex_sets` is a nullptr.
*/
std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(
    const ConvexSets& convex_sets,
    const std::vector<int>& continuous_revolute_joints,
    const std::vector<geometry::optimization::Hyperrectangle>& bboxes,
    Parallelism parallelism = Parallelism::Max());

/** @} */

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
