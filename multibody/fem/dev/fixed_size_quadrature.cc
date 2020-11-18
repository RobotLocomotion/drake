#include "drake/multibody/fem/dev/fixed_size_quadrature.h"
namespace drake {
namespace multibody {
namespace fem {
using std::array;
using std::make_pair;
using std::move;
using std::pair;
using std::sqrt;

template <typename T, int NaturalDimension, int QuadratureOrder>
pair<typename FixedSizeSimplexGaussianQuadrature<
         T, NaturalDimension, QuadratureOrder>::Traits::QuadratureLocationsType,
     typename FixedSizeSimplexGaussianQuadrature<
         T, NaturalDimension, QuadratureOrder>::Traits::WeightsType>
FixedSizeSimplexGaussianQuadrature<
    T, NaturalDimension, QuadratureOrder>::ComputePointsAndWeights() const {
  static_assert(1 <= QuadratureOrder && QuadratureOrder <= 3,
                "Only linear, quadratic and cubic quadratures are supported");
  if constexpr (NaturalDimension == 2) {
    // For a unit triangle, area = 0.5.
    if constexpr (QuadratureOrder == 1) {
      // quadrature point location,  weight/area
      //  (1/3, 1/3)                     1.0
      array<VectorD, kNumQuadratureLocations> points = {
          {{1.0 / 3.0, 1.0 / 3.0}}};
      array<T, kNumQuadratureLocations> weights = {{0.5}};
      return make_pair(move(points), move(weights));
      // TODO(xuchenhan-tri): fix the bug in cpplint as described in
      // https://github.com/google/styleguide/issues/541. A solution has been
      // proposed at https://github.com/cpplint/cpplint/pull/136.
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (QuadratureOrder == 2) {
      // quadrature point location,  weight/area
      //  (1/6, 1/6)                     1/3
      //  (2/3, 1/6)                     1/3
      //  (1/6, 2/3)                     1/3
      // Note: Here we choose r=1/2 in section 3 of [Hammer, 1956]. They also
      // mentioned the other choice with r=-1/2. We do not use r=-1/2 as it
      // lies out side of the element.
      array<VectorD, kNumQuadratureLocations> points;
      points[0] = {1.0 / 6.0, 1.0 / 6.0};
      points[1] = {2.0 / 3.0, 1.0 / 6.0};
      points[2] = {1.0 / 6.0, 2.0 / 3.0};
      array<T, kNumQuadratureLocations> weights = {
          {1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0}};
      return make_pair(move(points), move(weights));
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (QuadratureOrder == 3) {
      // quadrature point location,  weight/area
      //  (1/3, 1/3)                     -9/16
      //  (3/5, 1/5)                     25/48
      //  (1/5, 3/5)                     25/48
      //  (1/5, 1/5)                     25/48
      array<VectorD, kNumQuadratureLocations> points;
      points[0] = {1.0 / 3.0, 1.0 / 3.0};
      points[1] = {0.6, 0.2};
      points[2] = {0.2, 0.6};
      points[3] = {0.2, 0.2};
      array<T, kNumQuadratureLocations> weights = {
          {-9.0 / 32.0, 25.0 / 96.0, 25.0 / 96.0, 25.0 / 96.0}};
      return make_pair(move(points), move(weights));
    } else {
      DRAKE_UNREACHABLE();
    }
    // NOLINTNEXTLINE(readability/braces)
  } else if constexpr (NaturalDimension == 3) {
    // For a unit tetrahedron, area = 1/6.
    if constexpr (QuadratureOrder == 1) {
      // quadrature point location,  weight/area
      //  (1/4, 1/4, 1/4)                1.0
      array<VectorD, kNumQuadratureLocations> points = {{{0.25, 0.25, 0.25}}};
      array<T, kNumQuadratureLocations> weights = {{1.0 / 6.0}};
      return make_pair(move(points), move(weights));
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (QuadratureOrder == 2) {
      // quadrature point location,  weight/area
      //  (a, b, b)                      1/4
      //  (b, a, b)                      1/4
      //  (b, b, a)                      1/4
      //  (b, b, b)                      1/4
      // where a = (1+3*sqrt(1/5))/4, b = (1-1/sqrt(1/5))/4.
      array<VectorD, kNumQuadratureLocations> points;
      T a = (1.0 + 3.0 * sqrt(0.2)) / 4.0;
      T b = (1.0 - sqrt(0.2)) / 4.0;
      points[0] = {a, b, b};
      points[1] = {b, a, b};
      points[2] = {b, b, a};
      points[3] = {b, b, b};
      array<T, kNumQuadratureLocations> weights = {
          {1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0}};
      return make_pair(move(points), move(weights));
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (QuadratureOrder == 3) {
      // quadrature point location,  weight/area
      //  (1/4, 1/4, 1/4)               -4/5
      //  (a, b, b)                      9/20
      //  (b, a, b)                      9/20
      //  (b, b, a)                      9/20
      //  (b, b, b)                      9/20
      // where a = 1/2, b = 1/6.
      array<VectorD, kNumQuadratureLocations> points;
      T a = 0.5;
      T b = 1.0 / 6.0;
      points[0] = {0.25, 0.25, 0.25};
      points[1] = {a, b, b};
      points[2] = {b, a, b};
      points[3] = {b, b, a};
      points[4] = {b, b, b};
      array<T, kNumQuadratureLocations> weights = {
          {-2.0 / 15.0, 3.0 / 40.0, 3.0 / 40.0, 3.0 / 40.0, 3.0 / 40.0}};
      return make_pair(move(points), move(weights));
    } else {
      DRAKE_UNREACHABLE();
    }
  } else {
    DRAKE_UNREACHABLE();
  }
}
template class FixedSizeSimplexGaussianQuadrature<double, 2, 1>;
template class FixedSizeSimplexGaussianQuadrature<double, 3, 1>;
template class FixedSizeSimplexGaussianQuadrature<double, 2, 2>;
template class FixedSizeSimplexGaussianQuadrature<double, 3, 2>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
