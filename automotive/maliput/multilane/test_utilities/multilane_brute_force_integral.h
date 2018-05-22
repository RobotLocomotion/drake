#pragma once

#include <tuple>

#include "drake/automotive/maliput/multilane/road_curve.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

// Approximates a path length lower bound for the given @p road_curve
// from @p p_0 to @p p_1, for constant @p r and @p h offsets, by
// computing the same integral for a 2^@p k_order linear approximation.
//
// @param road_curve The RoadCurve to compute path length for.
// @param p_0 The lower integration bound for the p coordinate.
// @param p_1 The upper integration bound for the p coordinate.
// @param r The r coordinate offset.
// @param h The h coordinate offset.
// @param k_order Order k of the linear approximation, i.e. 2^k segments
//                are used in the approximation.
// @param maximum_step A mutable reference that, if given (can be nullptr), is
//                     set to the maximum step length used in the computation.
// @pre Given upper integration bound @p p_1 is less than or equal to 1.
// @pre Given upper integration bound @p p_1 is greater than or equal to
//      the given lower integration bound @p p_0.
// @pre Given lower integration bound @p p_0 is greater than or equal to 0.
// @pre Given @p k_order for the linear approximation is a non-negative number.
// @throw std::runtime_error if preconditions are not met.
double BruteForcePathLengthIntegral(const RoadCurve& road_curve,
                                    double p_0, double p_1, double r,
                                    double h, int k_order,
                                    double* maximum_step);

// Approximates the path length of a given @p road_curve from @p p_0 to @p p_1,
// for constant @p r and @p h offsets, to within specified @p tolerance.
//
// To ensure the error falls within @p tolerance, a path length lower bound
// is used (see BruteForcePathLengthIntegral()). If the curve is split in pieces
// no longer than the scale length of the curve, assuming the curve is well
// behaved, each one of the pieces can roughly be approximated as a constant
// curvature arc of radius R, subtending an angle θ. Then, let E(k) be the kth
// order approximation computed as the length of the chord that connects the arc
// endpoints, E(k+1) be the (k+1)th order approximation computed as the sum
// of the lengths of the resulting chords after an arc bisection and E(∞) be the
// true path length. It can be shown that: E(∞) - E(k+1) <= E(k+1) - E(k).
//
// TODO(hidmic): Compute a path length upper bound to ensure the approximation
// is within tolerance.
//
// @param road_curve The RoadCurve to compute path length for.
// @param p_0 The lower integration bound for the p coordinate.
// @param p_1 The upper integration bound for the p coordinate.
// @param r The r coordinate offset.
// @param h The h coordinate offset.
// @param tolerance The tolerance for the approximation, in the absolute error
//                  sense.
// @param k_order_hint A mutable reference to the order k of the linear
//                     approximation that, if given (can be nullptr) it's used
//                     as hint on call and it's updated to the actually required
//                     order necessary to achieve the specified tolerance on
//                     return.
// @pre Given upper integration bound @p p_1 is less than or equal to 1.
// @pre Given upper integration bound @p p_1 is greater than or equal to
//      the given lower integration bound @p p_0.
// @pre Given lower integration bound @p p_0 is greater than or equal to 0.
// @pre Given tolerance is a positive real number.
// @pre If given, the order suggested by @p k_order_hint is a non-negative
//      number.
// @throw std::runtime_error if preconditions are not met.
double AdaptiveBruteForcePathLengthIntegral(
    const RoadCurve& rc, double p_0, double p_1,
    double r, double h, double tolerance,
    int* k_order_hint);

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
