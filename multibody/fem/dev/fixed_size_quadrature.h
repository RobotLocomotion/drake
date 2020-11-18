#pragma once

#include <array>
#include <string>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
/* Forward declaration for template specialization. */
template <typename T, int NaturalDimensions, int QuadratureOrder>
class FixedSizeSimplexGaussianQuadrature;

template <class...>
struct QuadratureTraits {};

/* Traits for linear quadratures. */
template <typename T, int NaturalDimensions>
struct QuadratureTraits<
    FixedSizeSimplexGaussianQuadrature<T, NaturalDimensions, 1>> {
  static constexpr int kNumQuadratureLocations = 1;
  using VectorD = Eigen::Matrix<T, NaturalDimensions, 1>;
  using QuadratureLocationsType = std::array<VectorD, kNumQuadratureLocations>;
  using WeightsType = std::array<T, kNumQuadratureLocations>;
};

/* Traits for quadratic quadratures. */
template <typename T, int NaturalDimensions>
struct QuadratureTraits<
    FixedSizeSimplexGaussianQuadrature<T, NaturalDimensions, 2>> {
  static constexpr int kNumQuadratureLocations = NaturalDimensions + 1;
  using VectorD = Eigen::Matrix<T, NaturalDimensions, 1>;
  using QuadratureLocationsType = std::array<VectorD, kNumQuadratureLocations>;
  using WeightsType = std::array<T, kNumQuadratureLocations>;
};

/* Traits for cubic quadratures. */
template <typename T, int NaturalDimensions>
struct QuadratureTraits<
    FixedSizeSimplexGaussianQuadrature<T, NaturalDimensions, 3>> {
  static constexpr int kNumQuadratureLocations = NaturalDimensions + 2;
  using VectorD = Eigen::Matrix<T, NaturalDimensions, 1>;
  using QuadratureLocationsType = std::array<VectorD, kNumQuadratureLocations>;
  using WeightsType = std::array<T, kNumQuadratureLocations>;
};

/** A class for quadratures that facilitates numerical integrations in FEM. This
 class provides the common interface and the data for all the quadrature rules
 with no heap allocation or virtual methods. The specification of particular
 quadrature rules will be provided in the derived classes.
 @tparam T the scalar type of the function being integrated over.
 @tparam NaturalDimension dimension of the domain of integration.
 */
template <typename T, int NaturalDimension, int NumQuadratureLocations>
class FixedSizeQuadrature {
 public:
  static_assert(1 <= NaturalDimension && NaturalDimension <= 3);
  static_assert(1 <= NumQuadratureLocations);
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FixedSizeQuadrature);

  using VectorD = Eigen::Matrix<T, NaturalDimension, 1>;

  /** The dimension of the parent domain. */
  static constexpr int kNaturalDim = NaturalDimension;

  /** The number of quadrature locations. */
  static constexpr int kNumQuadratureLocations = NumQuadratureLocations;

  /** The position in parent coordinates of all quadrature points. */
  const std::array<VectorD, NumQuadratureLocations>& get_points() const {
    return points_;
  }

  /** The position in parent coordinates of the q-th quadrature point. */
  const VectorD& get_point(int q) const {
    DRAKE_ASSERT(q >= 0);
    DRAKE_ASSERT(q < NumQuadratureLocations);
    return points_[q];
  }

  /** The weight of the q-th quadrature point. */
  T get_weight(int q) const {
    DRAKE_ASSERT(q >= 0);
    DRAKE_ASSERT(q < NumQuadratureLocations);
    return weights_[q];
  }

 protected:
  explicit FixedSizeQuadrature(
      std::pair<std::array<VectorD, NumQuadratureLocations>,
                std::array<T, NumQuadratureLocations>>&& points_and_weights)
      : points_(std::move(points_and_weights.first)),
        weights_(std::move(points_and_weights.second)) {}

 private:
  std::array<VectorD, NumQuadratureLocations> points_;
  std::array<T, NumQuadratureLocations> weights_;
};

/** Calculates the Gaussian quadrature rule for 2D and 3D unit simplices
 (triangles and tetrahedrons up to cubic order as described section 3 in
 [Hammer, 1956] as well as section 9.10 of [Zienkiewics, 2005]. The 2D unit
 triangle has vertices located at (0,0), (1,0) and (0,1). The 3D unit
 tetrahedron has vertices located at (0,0,0), (1,0,0), (0,1,0) and (0,0,1).
 @tparam QuadratureOrder order of the quadrature rule. Must be 1, 2, or 3. The
 quadrature rule will be exact for polynomials of degree less than or equal to
 QuadratureOrder.
 @tparam NaturalDimension dimension of the unit simplex. Must be 2, or 3.

 [Hammer, 1956] P.C. Hammer, O.P. Marlowe, and A.H. Stroud. Numerical
 integration over simplexes and cones. Math. Tables Aids Comp. 10, 130-7, 1956.
 [Zienkiewicz, 2005] Zienkiewicz, Olek C., Robert L. Taylor, and Jian Z. Zhu.
 The finite element method: its basis and fundamentals. Elsevier, 2005. */
template <typename T, int NaturalDimension, int QuadratureOrder>
class FixedSizeSimplexGaussianQuadrature
    : public FixedSizeQuadrature<
          T, NaturalDimension,
          QuadratureTraits<FixedSizeSimplexGaussianQuadrature<
              T, NaturalDimension, QuadratureOrder>>::kNumQuadratureLocations> {
 public:
  using Traits = QuadratureTraits<
      FixedSizeSimplexGaussianQuadrature<T, NaturalDimension, QuadratureOrder>>;
  static constexpr int kNumQuadratureLocations =
      Traits::kNumQuadratureLocations;
  using VectorD = Eigen::Matrix<T, NaturalDimension, 1>;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FixedSizeSimplexGaussianQuadrature);
  FixedSizeSimplexGaussianQuadrature()
      : FixedSizeQuadrature<T, NaturalDimension, kNumQuadratureLocations>(
            ComputePointsAndWeights()) {}

 private:
  /* Helper function to initialize quadrature locations and weights. */
  std::pair<typename Traits::QuadratureLocationsType,
            typename Traits::WeightsType>
  ComputePointsAndWeights() const;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
