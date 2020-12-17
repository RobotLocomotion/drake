#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/isoparametric_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The number of nodes of 1D simplices (segments) is 2. The number of nodes of
 2D simplices (triangles) is 3. The number of nodes of 3D simplices
 (tetrahedron) is 4. */
template <int NaturalDimension>
constexpr int num_linear_simplex_element_nodes() {
  return NaturalDimension + 1;
}

/** Traits for LinearSimplexElement. */
template <typename T, int NaturalDimension, int SpatialDimension,
          int NumSampleLocations>
struct LinearSimplexElementTraits {
  using Scalar = T;
  static constexpr int kNaturalDimension = NaturalDimension;
  static constexpr int kSpatialDimension = SpatialDimension;
  static constexpr int kNumSampleLocations = NumSampleLocations;
  static constexpr int kNumNodes =
      num_linear_simplex_element_nodes<NaturalDimension>();
};

/** A concrete IsoparametricElement for linear simplex elements
 (segments, triangles and tetrahedrons). In parent domain, the simplex's node
 are laid out in the following way: the 0-th node is placed at the origin and
 the i-th node for 1 <= i <= NaturalDimension is placed at the point whose i-th
 coordinate is 1 and all other coordinates are 0.
 @tparam NaturalDimension The dimension of the parent domain.
 @tparam SpatialDimension The dimension of the spatial domain.
 @tparam NumSampleLocations The number of locations to evaluate interpolations
 and other calculations are performed. */
template <typename T, int NaturalDimension, int SpatialDimension,
          int NumSampleLocations>
class LinearSimplexElement
    : public IsoparametricElement<
          LinearSimplexElement<T, NaturalDimension, SpatialDimension,
                               NumSampleLocations>,
          LinearSimplexElementTraits<T, NaturalDimension, SpatialDimension,
                                     NumSampleLocations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearSimplexElement);

  static_assert(1 <= NaturalDimension && NaturalDimension <= 3,
                "Only 1, 2 and 3 dimensional manifolds are supported.");
  static_assert(1 <= SpatialDimension && SpatialDimension <= 3,
                "Only 1, 2 and 3 spatial dimensions are supported.");
  static_assert(NaturalDimension <= SpatialDimension,
                "NaturalDimension must be smaller than or equal to the "
                "SpatialDimension.");
  using ElementType =
      LinearSimplexElement<T, NaturalDimension, SpatialDimension,
                           NumSampleLocations>;
  using Traits =
      LinearSimplexElementTraits<T, NaturalDimension, SpatialDimension,
                                 NumSampleLocations>;
  using Base = IsoparametricElement<ElementType, Traits>;
  using Base::natural_dimension;
  using Base::num_nodes;
  using LocationsType = typename Base::LocationsType;

  template <typename U>
  using ArrayType = typename Base::template ArrayType<U>;

  using JacobianMatrix = typename Base::JacobianMatrix;

  /** Constructs a linear simplex isoparametric element and precomputes the
   shape functions as well as their gradients. */
  explicit LinearSimplexElement(LocationsType locations)
      : Base(std::move(locations)),
        S_(GetShapeFunctionsHelper()),
        dSdxi_(GetGradientInParentCoordinatesHelper()) {}

  /** Implements Base::GetShapeFunctions(). */
  const ArrayType<Vector<T, num_nodes()>>& GetShapeFunctions() const {
    return S_;
  }

  /** Implements Base::GetGradientInParentCoordinates(). */
  const ArrayType<Eigen::Matrix<T, num_nodes(), natural_dimension()>>&
  GetGradientInParentCoordinates() const {
    return dSdxi_;
  }

 private:
  /* Precomputes the shape functions. */
  ArrayType<Vector<T, num_nodes()>> GetShapeFunctionsHelper() const {
    ArrayType<Vector<T, num_nodes()>> S;
    const LocationsType& locations = this->locations();
    for (int q = 0; q < this->num_sample_locations(); ++q) {
      Vector<T, num_nodes()> Sq;
      // Sₐ = ξₐ₋₁ for a = 1, ..., NumNodes - 1
      for (int a = 1; a < num_nodes(); ++a) {
        Sq(a) = locations[q](a - 1);
      }
      // S₀ = 1−ξ₀ − ... − ξₙ₋₂
      Sq(0) = 0;
      Sq(0) = 1 - Sq.sum();
      S[q] = Sq;
    }
    return S;
  }

  /* Precomputes the shape function gradients in parent coordinates. */
  static const ArrayType<Eigen::Matrix<T, num_nodes(), natural_dimension()>>
  GetGradientInParentCoordinatesHelper() {
    Eigen::Matrix<T, num_nodes(), natural_dimension()> dSdxi_q;
    dSdxi_q.template topRows<1>() =
        -1.0 * Vector<T, natural_dimension()>::Ones();
    dSdxi_q.template bottomRows<natural_dimension()>() =
        Eigen::Matrix<T, natural_dimension(), natural_dimension()>::Identity();
    ArrayType<Eigen::Matrix<T, num_nodes(), natural_dimension()>> dSdxi;
    dSdxi.fill(dSdxi_q);
    return dSdxi;
  }

  /* Shape functions evaluated at points specified at construction. */
  ArrayType<Vector<T, num_nodes()>> S_;
  /* Shape function derivatives evaluated at points specified at construction.
   */
  ArrayType<Eigen::Matrix<T, num_nodes(), natural_dimension()>> dSdxi_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
