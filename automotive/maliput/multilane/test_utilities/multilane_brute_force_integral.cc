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
  const double inf = std::numeric_limits<double>::infinity();
  if (maximum_step != nullptr) *maximum_step = -inf;
  Vector3<double> geo_position_at_prev_p = rc.W_of_prh(p_0, r, h);
  for (int i = 1; i <= iterations; ++i) {
    const double p = p_0 + d_p * static_cast<double>(i) / iterations;
    const Vector3<double> geo_position_at_p = rc.W_of_prh(p, r, h);
    const double ith_step_length = (
        geo_position_at_p - geo_position_at_prev_p).norm();
    if (maximum_step != nullptr) {
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
  const double inf = std::numeric_limits<double>::infinity();
  int k_order = k_order_hint != nullptr ? *k_order_hint : 0;
  double k_order_maximum_step = inf;
  double k_order_path_length = BruteForcePathLengthIntegral(
      rc, p_0, p_1, r, h, k_order, &k_order_maximum_step);
  while (true) {
    double k_plus_1_order_maximum_step = inf;
    const double k_plus_1_order_path_length =
        BruteForcePathLengthIntegral(rc, p_0, p_1, r, h, k_order + 1,
                                     &k_plus_1_order_maximum_step);
    const double k_order_error = std::abs(
        k_plus_1_order_path_length - k_order_path_length);
    if (k_order_maximum_step < rc.scale_length() && k_order_error < tolerance) {
      break;
    }
    k_order_path_length = k_plus_1_order_path_length;
    k_order_maximum_step = k_plus_1_order_maximum_step;
    k_order += 1;
  }
  if (k_order_hint != nullptr) *k_order_hint = k_order;
  return k_order_path_length;
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
