#pragma once

#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Calculates the number of quadrature points used for simplices given the
 natural dimension and the order of the quadrature rule as template
 parameters. */
template <int NaturalDim, int Order>
constexpr int num_simplex_quadrature_locations() {
  static_assert(
      1 <= Order && Order <= 3,
      "Only linear, quadratic and cubic quadrature rules are supported.");
  if constexpr (Order == 1) {
    return 1;
  } else if constexpr (Order == 2) {
    return NaturalDim + 1;
  } else if constexpr (Order == 3) {
    return NaturalDim + 2;
  }
}

/** Calculates the Gaussian quadrature rule for 2D and 3D unit simplices
 (triangles and tetrahedrons up to cubic order as described section 3 in
 [Hammer, 1956] as well as section 9.10 of [Zienkiewics, 2005]. The 2D unit
 triangle has vertices located at (0,0), (1,0) and (0,1). The 3D unit
 tetrahedron has vertices located at (0,0,0), (1,0,0), (0,1,0) and (0,0,1).
 @tparam Order order of the quadrature rule. Must be 1, 2, or 3. The
 quadrature rule will be exact for polynomials of degree less than or equal to
 Order.
 @tparam NaturalDimension dimension of the unit simplex. Must be 2, or 3.

 [Hammer, 1956] P.C. Hammer, O.P. Marlowe, and A.H. Stroud. Numerical
 integration over simplexes and cones. Math. Tables Aids Comp. 10, 130-7, 1956.
 [Zienkiewicz, 2005] Zienkiewicz, Olek C., Robert L. Taylor, and Jian Z. Zhu.
 The finite element method: its basis and fundamentals. Elsevier, 2005. */
template <int NaturalDimension, int Order>
class SimplexGaussianQuadrature
    : public Quadrature<NaturalDimension,
          num_simplex_quadrature_locations<NaturalDimension, Order>()> {
 public:
  using Base = Quadrature<NaturalDimension,
      num_simplex_quadrature_locations<NaturalDimension, Order>()>;
  using VectorD = typename Base::VectorD;
  using LocationsType = typename Base::LocationsType;
  using WeightsType = typename Base::WeightsType;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimplexGaussianQuadrature);

  SimplexGaussianQuadrature() : Base(ComputePointsAndWeights()) {}

 private:
  /* Helper function to initialize quadrature locations and weights. */
  std::pair<LocationsType, WeightsType> ComputePointsAndWeights() const {
    if constexpr (NaturalDimension == 2) {
      // For a unit triangle, area = 0.5.
      if constexpr (Order == 1) {
        // quadrature point location,  weight/area
        //  (1/3, 1/3)                     1.0
        LocationsType points = {{{1.0 / 3.0, 1.0 / 3.0}}};
        WeightsType weights = {{0.5}};
        return make_pair(move(points), move(weights));
        // TODO(xuchenhan-tri): fix the bug in cpplint as described in
        // https://github.com/google/styleguide/issues/541. A solution has been
        // proposed at https://github.com/cpplint/cpplint/pull/136.
        // NOLINTNEXTLINE(readability/braces) false positive
      } else if constexpr (Order == 2) {
        // quadrature point location,  weight/area
        //  (1/6, 1/6)                     1/3
        //  (2/3, 1/6)                     1/3
        //  (1/6, 2/3)                     1/3
        // Note: Here we choose r=1/2 in section 3 of [Hammer, 1956]. They also
        // mentioned the other choice with r=-1/2. We do not use r=-1/2 as it
        // lies out side of the element.
        LocationsType points;
        points[0] = {1.0 / 6.0, 1.0 / 6.0};
        points[1] = {2.0 / 3.0, 1.0 / 6.0};
        points[2] = {1.0 / 6.0, 2.0 / 3.0};
        WeightsType weights = {{1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0}};
        return make_pair(move(points), move(weights));
        // NOLINTNEXTLINE(readability/braces) false positive
      } else if constexpr (Order == 3) {
        // quadrature point location,  weight/area
        //  (1/3, 1/3)                     -9/16
        //  (3/5, 1/5)                     25/48
        //  (1/5, 3/5)                     25/48
        //  (1/5, 1/5)                     25/48
        LocationsType points;
        points[0] = {1.0 / 3.0, 1.0 / 3.0};
        points[1] = {0.6, 0.2};
        points[2] = {0.2, 0.6};
        points[3] = {0.2, 0.2};
        WeightsType weights = {
            {-9.0 / 32.0, 25.0 / 96.0, 25.0 / 96.0, 25.0 / 96.0}};
        return make_pair(move(points), move(weights));
      } else {
        DRAKE_UNREACHABLE();
      }
      // NOLINTNEXTLINE(readability/braces) false positive
    } else if constexpr (NaturalDimension == 3) {
      // For a unit tetrahedron, area = 1/6.
      if constexpr (Order == 1) {
        // quadrature point location,  weight/area
        //  (1/4, 1/4, 1/4)                1.0
        LocationsType points = {{{0.25, 0.25, 0.25}}};
        WeightsType weights = {{1.0 / 6.0}};
        return make_pair(move(points), move(weights));
        // NOLINTNEXTLINE(readability/braces) false positive
      } else if constexpr (Order == 2) {
        // quadrature point location,  weight/area
        //  (a, b, b)                      1/4
        //  (b, a, b)                      1/4
        //  (b, b, a)                      1/4
        //  (b, b, b)                      1/4
        // where a = (1+3*sqrt(1/5))/4, b = (1-1/sqrt(1/5))/4.
        LocationsType points;
        double a = (1.0 + 3.0 * sqrt(0.2)) / 4.0;
        double b = (1.0 - sqrt(0.2)) / 4.0;
        points[0] = {a, b, b};
        points[1] = {b, a, b};
        points[2] = {b, b, a};
        points[3] = {b, b, b};
        WeightsType weights = {
            {1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0}};
        return make_pair(move(points), move(weights));
        // NOLINTNEXTLINE(readability/braces) false positive
      } else if constexpr (Order == 3) {
        // quadrature point location,  weight/area
        //  (1/4, 1/4, 1/4)               -4/5
        //  (a, b, b)                      9/20
        //  (b, a, b)                      9/20
        //  (b, b, a)                      9/20
        //  (b, b, b)                      9/20
        // where a = 1/2, b = 1/6.
        LocationsType points;
        double a = 0.5;
        double b = 1.0 / 6.0;
        points[0] = {0.25, 0.25, 0.25};
        points[1] = {a, b, b};
        points[2] = {b, a, b};
        points[3] = {b, b, a};
        points[4] = {b, b, b};
        WeightsType weights = {
            {-2.0 / 15.0, 3.0 / 40.0, 3.0 / 40.0, 3.0 / 40.0, 3.0 / 40.0}};
        return make_pair(move(points), move(weights));
      } else {
        DRAKE_UNREACHABLE();
      }
    } else {
      DRAKE_UNREACHABLE();
    }
  }
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
