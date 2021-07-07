#pragma once

#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Calculates the number of quadrature points used for simplices given the
 natural dimension and the order of the quadrature rule as template
 parameters. */
template <int natural_dimension, int order>
constexpr int num_simplex_quadrature_locations() {
  static_assert(
      1 <= order && order <= 3,
      "Only linear, quadratic and cubic quadrature rules are supported.");
  if constexpr (order == 1) {
    return 1;
  } else if constexpr (order == 2) {
    return natural_dimension + 1;
  } else if constexpr (order == 3) {
    return natural_dimension + 2;
  }
}

// TODO(xuchenhan-tri): Support natural_dimension = 1 if it turns out useful.
/* Calculates the Gaussian quadrature rule for 2D and 3D unit simplices
 (triangles and tetrahedrons up to cubic order as described section 3 in
 [Hammer, 1956] as well as section 9.10 of [Zienkiewics, 2005]. The 2D unit
 triangle has vertices located at (0,0), (1,0) and (0,1). The 3D unit
 tetrahedron has vertices located at (0,0,0), (1,0,0), (0,1,0) and (0,0,1).
 @tparam order              Order of the quadrature rule. Must be 1, 2, or 3.
                            The quadrature rule will be exact for polynomials of
                            degree less than or equal to `order`.
 @tparam natural_dimension  Dimension of the unit simplex. Must be 2 (triangles)
                            or 3 (tetrahedrons).

 [Hammer, 1956] P.C. Hammer, O.P. Marlowe, and A.H. Stroud. Numerical
 integration over simplexes and cones. Math. Tables Aids Comp. 10, 130-7, 1956.
 [Zienkiewicz, 2005] Zienkiewicz, Olek C., Robert L. Taylor, and Jian Z. Zhu.
 The finite element method: its basis and fundamentals. Elsevier, 2005. */
template <int natural_dimension, int order>
class SimplexGaussianQuadrature
    : public Quadrature<natural_dimension, num_simplex_quadrature_locations<
                                               natural_dimension, order>()> {
 public:
  using Base =
      Quadrature<natural_dimension,
                 num_simplex_quadrature_locations<natural_dimension, order>()>;
  using VectorD = typename Base::VectorD;
  using LocationsType = typename Base::LocationsType;
  using WeightsType = typename Base::WeightsType;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimplexGaussianQuadrature);

  SimplexGaussianQuadrature() : Base(ComputePointsAndWeights()) {}

 private:
  /* Helper function to initialize quadrature locations and weights. */
  static std::pair<LocationsType, WeightsType> ComputePointsAndWeights();
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
