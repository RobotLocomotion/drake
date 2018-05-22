#include "drake/automotive/maliput/multilane/test_utilities/multilane_brute_force_integral.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

double BruteForcePathLengthIntegral(const RoadCurve& rc, double p_0, double p_1,
                                    double r, double h, int k_order,
                                    double* maximum_step) {
  DRAKE_THROW_UNLESS(0. <= p_0);
  DRAKE_THROW_UNLESS(p_0 <= p_1);
  DRAKE_THROW_UNLESS(p_1 <= 1.);
  DRAKE_THROW_UNLESS(k_order >= 0);
  double length = 0.0;
  const double d_p = (p_1 - p_0);
  const int iterations = std::pow(2, k_order);
  if (maximum_step != nullptr) {
    *maximum_step = -std::numeric_limits<double>::infinity();
  }
  // Splits the [p_0, p_1] interval in 2^k intervals, computes the positions
  // in the global frame for each interval boundary and sums up the path lengths
  // of the segments in the global frame that correspond to each one of those
  // intervals.
  Vector3<double> geo_position_at_prev_p = rc.W_of_prh(p_0, r, h);
  for (int i = 1; i <= iterations; ++i) {
    const double p = p_0 + d_p * static_cast<double>(i) / iterations;
    const Vector3<double> geo_position_at_p = rc.W_of_prh(p, r, h);
    const double ith_step_length = (
        geo_position_at_p - geo_position_at_prev_p).norm();
    if (maximum_step != nullptr) {
      // Keep track of the maximum step taken.
      *maximum_step = std::max(*maximum_step, ith_step_length);
    }
    length += ith_step_length;
    geo_position_at_prev_p = geo_position_at_p;
  }
  return length;
}

double AdaptiveBruteForcePathLengthIntegral(
    const RoadCurve& rc, double p_0, double p_1, double r,
    double h, double tolerance, int* k_order_hint) {
  DRAKE_THROW_UNLESS(tolerance > 0.);
  const double kInfinity = std::numeric_limits<double>::infinity();
  // Zero initializes the current k order unless a hint was provided.
  int k_order = k_order_hint != nullptr ? *k_order_hint : 0;
  double k_order_maximum_step = kInfinity;
  // Computes the k-order path length approximation.
  double k_order_path_length = BruteForcePathLengthIntegral(
      rc, p_0, p_1, r, h, k_order, &k_order_maximum_step);
  // Estimates the error of a k-order approximation, increasing k
  // until said error falls within the specified tolerance.
  while (true) {
    double k_plus_1_order_maximum_step = kInfinity;
    // Computes the k+1-order path length approximation.
    const double k_plus_1_order_path_length =
        BruteForcePathLengthIntegral(rc, p_0, p_1, r, h, k_order + 1,
                                     &k_plus_1_order_maximum_step);
    // Estimates the error of the k-order path length approximation
    // by comparing it with the k+1-order one.
    const double k_order_error = std::abs(
        k_plus_1_order_path_length - k_order_path_length);
    // Not only the estimated error must be within tolerance but
    // also the maximum step taken by the k-order approximation
    // must be within a scale length to ensure it is valid (see
    // AdaptiveBruteForcePathLengthIntegral() function documentation).
    if (k_order_maximum_step < rc.scale_length() && k_order_error < tolerance) {
      break;
    }
    k_order_path_length = k_plus_1_order_path_length;
    k_order_maximum_step = k_plus_1_order_maximum_step;
    k_order += 1;
  }
  // Update k-order hint with actual k-order required to achieve
  // the desired accuracy.
  if (k_order_hint != nullptr) *k_order_hint = k_order;
  return k_order_path_length;
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
