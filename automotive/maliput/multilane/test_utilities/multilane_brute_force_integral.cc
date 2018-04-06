#include "drake/automotive/maliput/multilane/test_utilities/multilane_brute_force_integral.h"

#include <cmath>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

double BruteForcePathLengthIntegral(const RoadCurve& rc, double p_0, double p_1,
                                    double r, double h, int k_order) {
  DRAKE_THROW_UNLESS(0. <= p_0);
  DRAKE_THROW_UNLESS(p_0 <= p_1);
  DRAKE_THROW_UNLESS(p_1 <= 1.);
  DRAKE_THROW_UNLESS(k_order >= 0);
  const double d_p = (p_1 - p_0);
  const int iterations = std::pow(2, k_order);
  double length{0.};
  Vector3<double> geo_position_at_prev_p = rc.W_of_prh(p_0, r, h);
  for (int i = 1; i <= iterations; ++i) {
    const double p = p_0 + d_p * static_cast<double>(i) / iterations;
    const Vector3<double> geo_position_at_p = rc.W_of_prh(p, r, h);
    length += (geo_position_at_p - geo_position_at_prev_p).norm();
    geo_position_at_prev_p = geo_position_at_p;
  }
  return length;
}

double AdaptiveBruteForcePathLengthIntegral(
    const RoadCurve& rc, double p_0, double p_1, double r,
    double h, double tolerance, int* k_order_hint) {
  DRAKE_THROW_UNLESS(tolerance > 0.);
  bool already_within_tolerance = false;
  int k_order = k_order_hint != nullptr ? *k_order_hint : 0;
  double k_order_path_length = BruteForcePathLengthIntegral(
      rc, p_0, p_1, r, h, k_order);
  while (true) {
    const double k_plus_1_order_path_length =
        BruteForcePathLengthIntegral(rc, p_0, p_1, r, h, k_order + 1);
    const double error = std::abs(
        k_order_path_length - k_plus_1_order_path_length);
    if (error < tolerance) {
      // Compute twice to make sure this isn't error local minima.
      if (already_within_tolerance) break;
      already_within_tolerance = true;
    } else {
      already_within_tolerance = false;
    }
    k_order_path_length = k_plus_1_order_path_length;
    k_order += 1;
  }
  if (k_order_hint != nullptr) *k_order_hint = k_order;
  return k_order_path_length;
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
