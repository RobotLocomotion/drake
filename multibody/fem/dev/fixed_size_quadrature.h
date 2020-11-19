#pragma once

#include <array>
#include <string>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
/** A class for quadratures that facilitates numerical integrations in FEM. This
 class provides the common interface and the data for all the quadrature rules
 with no heap allocation or virtual methods. The specification of particular
 quadrature rules will be provided in the derived classes.
 @tparam T the scalar type of the function being integrated over.
 @tparam NaturalDimension dimension of the domain of integration.
 */
template <typename T, int NaturalDimension, int NumLocations>
class FixedSizeQuadrature {
 public:
  static_assert(1 <= NaturalDimension && NaturalDimension <= 3,
                "Only 1,2 and 3 dimensional shapes are supported.");
  static_assert(1 <= NumLocations,
                "There has to be at least one quadrature point.");

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FixedSizeQuadrature);

  using VectorD = Eigen::Matrix<T, NaturalDimension, 1>;
  using LocationsType = std::array<VectorD, NumLocations>;
  using WeightsType = std::array<T, NumLocations>;

  /** The dimension of the parent domain. */
  static constexpr int natural_dimension() { return NaturalDimension; }

  /** The number of quadrature locations. */
  static constexpr int num_quadrature_points() { return NumLocations; }

  /** The position in parent coordinates of all quadrature points. */
  const LocationsType& get_points() const { return points_; }

  /** The position in parent coordinates of the q-th quadrature point. */
  const VectorD& get_point(int q) const {
    DRAKE_ASSERT(q >= 0);
    DRAKE_ASSERT(q < NumLocations);
    return points_[q];
  }

  /** The weight of the q-th quadrature point. */
  T get_weight(int q) const {
    DRAKE_ASSERT(q >= 0);
    DRAKE_ASSERT(q < NumLocations);
    return weights_[q];
  }

 protected:
  /** Construct a Quadrature object given the quadrature locations and the
   weights of the quadrature points. */
  explicit FixedSizeQuadrature(
      std::pair<LocationsType, WeightsType>&& points_and_weights)
      : points_(std::move(points_and_weights.first)),
        weights_(std::move(points_and_weights.second)) {}

 private:
  LocationsType points_;
  WeightsType weights_;
};

/** Calculates the number of quadrature points used for simplices given the
 natural dimension and the order of the quadrature rule as template
 parameters. */
template <int NaturalDim, int Order>
constexpr int SimplexQuadratureNumLocations() {
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
template <typename T, int NaturalDimension, int Order>
class FixedSizeSimplexGaussianQuadrature
    : public FixedSizeQuadrature<
          T, NaturalDimension,
          SimplexQuadratureNumLocations<NaturalDimension, Order>()> {
 public:
  static_assert(
      1 <= Order && Order <= 3,
      "Only linear, quadratic and cubic quadrature rules are supported.");

  static constexpr int num_quadrature_points() {
    return SimplexQuadratureNumLocations<NaturalDimension, Order>();
  }

  using Base =
      FixedSizeQuadrature<T, NaturalDimension, num_quadrature_points()>;
  using VectorD = typename Base::VectorD;
  using LocationsType = typename Base::LocationsType;
  using WeightsType = typename Base::WeightsType;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FixedSizeSimplexGaussianQuadrature);

  FixedSizeSimplexGaussianQuadrature()
      : FixedSizeQuadrature<T, NaturalDimension, num_quadrature_points()>(
            ComputePointsAndWeights()) {}

 private:
  /* Helper function to initialize quadrature locations and weights. */
  std::pair<LocationsType, WeightsType> ComputePointsAndWeights() const;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
