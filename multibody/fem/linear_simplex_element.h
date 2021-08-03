#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/isoparametric_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* The number of nodes of 1D simplices (segments) is 2. The number of nodes of
 2D simplices (triangles) is 3. The number of nodes of 3D simplices
 (tetrahedron) is 4. */
template <int natural_dimension>
constexpr int num_linear_simplex_element_nodes() {
  return natural_dimension + 1;
}

/* Traits for LinearSimplexElement. */
template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
struct LinearSimplexElementTraits {
  using Scalar = T;
  static constexpr int natural_dimension = natural_dimension_at_compile_time;
  static constexpr int spatial_dimension = spatial_dimension_at_compile_time;
  static constexpr int num_sample_locations =
      num_sample_locations_at_compile_time;
  static constexpr int num_nodes =
      num_linear_simplex_element_nodes<natural_dimension>();
};

/* A concrete IsoparametricElement for linear simplex elements
 (segments, triangles and tetrahedrons). In parent domain, the simplex's node
 are laid out in the following way: the 0-th node is placed at the origin and
 the i-th node for 1 <= i <= natural_dimension is placed at the point whose i-th
 coordinate is 1 and all other coordinates are 0.
 @tparam natural_dimension_at_compile_time    The dimension of the parent
                                              domain.
 @tparam spatial_dimension_at_compile_time    The dimension of the spatial
                                              domain.
 @tparam num_sample_locations_at_compile_time The number of locations to
                                              evaluate interpolations and
                                              other calculations are performed.
 */
template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
class LinearSimplexElement
    : public IsoparametricElement<
          LinearSimplexElement<T, natural_dimension_at_compile_time,
                               spatial_dimension_at_compile_time,
                               num_sample_locations_at_compile_time>,
          LinearSimplexElementTraits<T, natural_dimension_at_compile_time,
                                     spatial_dimension_at_compile_time,
                                     num_sample_locations_at_compile_time>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearSimplexElement);

  using Element = LinearSimplexElement<T, natural_dimension_at_compile_time,
                                       spatial_dimension_at_compile_time,
                                       num_sample_locations_at_compile_time>;
  using Traits =
      LinearSimplexElementTraits<T, natural_dimension_at_compile_time,
                                 spatial_dimension_at_compile_time,
                                 num_sample_locations_at_compile_time>;
  using Base = IsoparametricElement<Element, Traits>;
  using Base::natural_dimension;
  using Base::num_nodes;
  using Base::num_sample_locations;
  using Base::spatial_dimension;
  using VectorNumNodes = typename Base::VectorNumNodes;
  using GradientInParentCoordinates =
      typename Base::GradientInParentCoordinates;
  using GradientInSpatialCoordinates =
      typename Base::GradientInSpatialCoordinates;
  using JacobianMatrix = typename Base::JacobianMatrix;
  using PseudoinverseJacobianMatrix =
      typename Base::PseudoinverseJacobianMatrix;

  template <typename U>
  using ArrayNumSamples = typename Base::template ArrayNumSamples<U>;

  static_assert(1 <= natural_dimension && natural_dimension <= 3,
                "Only 1, 2 and 3 dimensional manifolds are supported.");
  static_assert(1 <= spatial_dimension && spatial_dimension <= 3,
                "Only 1, 2 and 3 spatial dimensions are supported.");
  static_assert(natural_dimension <= spatial_dimension,
                "natural_dimension must be smaller than or equal to the "
                "spatial_dimension.");

  /* Constructs a linear simplex isoparametric element and precomputes the
   shape functions as well as their gradients. */
  explicit LinearSimplexElement(
      const ArrayNumSamples<Vector<double, natural_dimension>>& locations)
      : Base(locations, CalcShapeFunctions(locations),
             CalcGradientInParentCoordinates()) {}

  /* Implements Base::CalcGradientInSpatialCoordinates(). */
  ArrayNumSamples<GradientInSpatialCoordinates>
  CalcGradientInSpatialCoordinates(
      const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension, num_nodes>>&
          xa) const;

  /* Implements Base::CalcJacobian(). */
  ArrayNumSamples<JacobianMatrix> CalcJacobian(
      const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension, num_nodes>>&
          xa) const;

  /* Implements Base::CalcJacobianPseudoinverse(). */
  ArrayNumSamples<PseudoinverseJacobianMatrix> CalcJacobianPseudoinverse(
      const ArrayNumSamples<JacobianMatrix>& jacobian) const;

 private:
  /* Precomputes the shape functions. */
  ArrayNumSamples<VectorNumNodes> CalcShapeFunctions(
      const ArrayNumSamples<Vector<double, natural_dimension>>& locations)
      const;

  /* Precomputes the shape function gradients in parent coordinates. */
  static const ArrayNumSamples<GradientInParentCoordinates>
  CalcGradientInParentCoordinates();
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
