#include "drake/multibody/fem/simplex_gaussian_quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using std::make_pair;
using std::move;

/* Linear triangle quadrature. */
template <>
std::pair<SimplexGaussianQuadrature<2, 1>::LocationsType,
          SimplexGaussianQuadrature<2, 1>::WeightsType>
SimplexGaussianQuadrature<2, 1>::ComputePointsAndWeights() {
  /* quadrature point location,  weight/area
      (1/3, 1/3)                     1.0        */
  LocationsType points = {{{1.0 / 3.0, 1.0 / 3.0}}};
  /* For a unit triangle, area = 0.5. */
  WeightsType weights = {{0.5}};
  return make_pair(move(points), move(weights));
}

/* Quadratic triangle quadrature. */
template <>
std::pair<SimplexGaussianQuadrature<2, 2>::LocationsType,
          SimplexGaussianQuadrature<2, 2>::WeightsType>
SimplexGaussianQuadrature<2, 2>::ComputePointsAndWeights() {
  /* quadrature point location,  weight/area
       (1/6, 1/6)                     1/3
       (2/3, 1/6)                     1/3
       (1/6, 2/3)                     1/3
   Note: Here we choose r=1/2 in section 3 of [Hammer, 1956]. They also
   mentioned the other choice with r=-1/2. We do not use r=-1/2 as it
   lies out side of the element. */
  LocationsType points;
  points[0] = {1.0 / 6.0, 1.0 / 6.0};
  points[1] = {2.0 / 3.0, 1.0 / 6.0};
  points[2] = {1.0 / 6.0, 2.0 / 3.0};
  /* For a unit triangle, area = 0.5. */
  WeightsType weights = {{1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0}};
  return make_pair(move(points), move(weights));
}

/* Cubic triangle quadrature. */
template <>
std::pair<SimplexGaussianQuadrature<2, 3>::LocationsType,
          SimplexGaussianQuadrature<2, 3>::WeightsType>
SimplexGaussianQuadrature<2, 3>::ComputePointsAndWeights() {
  /* quadrature point location,  weight/area
      (1/3, 1/3)                     -9/16
      (3/5, 1/5)                     25/48
      (1/5, 3/5)                     25/48
      (1/5, 1/5)                     25/48         */
  LocationsType points;
  points[0] = {1.0 / 3.0, 1.0 / 3.0};
  points[1] = {0.6, 0.2};
  points[2] = {0.2, 0.6};
  points[3] = {0.2, 0.2};
  /* For a unit triangle, area = 0.5. */
  WeightsType weights = {{-9.0 / 32.0, 25.0 / 96.0, 25.0 / 96.0, 25.0 / 96.0}};
  return make_pair(move(points), move(weights));
}

/* Linear tetrahedral quadrature. */
template <>
std::pair<SimplexGaussianQuadrature<3, 1>::LocationsType,
          SimplexGaussianQuadrature<3, 1>::WeightsType>
SimplexGaussianQuadrature<3, 1>::ComputePointsAndWeights() {
  /* quadrature point location,  weight/area
      (1/4, 1/4, 1/4)                1.0              */
  LocationsType points = {{{0.25, 0.25, 0.25}}};
  /* For a unit tetrahedron, area = 1/6. */
  WeightsType weights = {{1.0 / 6.0}};
  return make_pair(move(points), move(weights));
}

/* Quadratic tetrahedral quadrature. */
template <>
std::pair<SimplexGaussianQuadrature<3, 2>::LocationsType,
          SimplexGaussianQuadrature<3, 2>::WeightsType>
SimplexGaussianQuadrature<3, 2>::ComputePointsAndWeights() {
  /* quadrature point location,  weight/area
      (a, b, b)                      1/4
      (b, a, b)                      1/4
      (b, b, a)                      1/4
      (b, b, b)                      1/4
   where a = (1+3*sqrt(1/5))/4, b = (1-1/sqrt(1/5))/4. */
  LocationsType points;
  double a = (1.0 + 3.0 * sqrt(0.2)) / 4.0;
  double b = (1.0 - sqrt(0.2)) / 4.0;
  points[0] = {a, b, b};
  points[1] = {b, a, b};
  points[2] = {b, b, a};
  points[3] = {b, b, b};
  /* For a unit tetrahedron, area = 1/6. */
  WeightsType weights = {{1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0}};
  return make_pair(move(points), move(weights));
}

/* Cubic tetrahedral quadrature. */
template <>
std::pair<SimplexGaussianQuadrature<3, 3>::LocationsType,
          SimplexGaussianQuadrature<3, 3>::WeightsType>
SimplexGaussianQuadrature<3, 3>::ComputePointsAndWeights() {
  /* quadrature point location,  weight/area
      (1/4, 1/4, 1/4)               -4/5
      (a, b, b)                      9/20
      (b, a, b)                      9/20
      (b, b, a)                      9/20
      (b, b, b)                      9/20
   where a = 1/2, b = 1/6. */
  LocationsType points;
  double a = 0.5;
  double b = 1.0 / 6.0;
  points[0] = {0.25, 0.25, 0.25};
  points[1] = {a, b, b};
  points[2] = {b, a, b};
  points[3] = {b, b, a};
  points[4] = {b, b, b};
  /* For a unit tetrahedron, area = 1/6. */
  WeightsType weights = {
      {-2.0 / 15.0, 3.0 / 40.0, 3.0 / 40.0, 3.0 / 40.0, 3.0 / 40.0}};
  return make_pair(move(points), move(weights));
}
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
