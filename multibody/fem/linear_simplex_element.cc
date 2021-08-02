#include "drake/multibody/fem/linear_simplex_element.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
typename LinearSimplexElement<T, natural_dimension_at_compile_time,
                              spatial_dimension_at_compile_time,
                              num_sample_locations_at_compile_time>::
    template ArrayNumSamples<typename IsoparametricElement<
        LinearSimplexElement<T, natural_dimension_at_compile_time,
                             spatial_dimension_at_compile_time,
                             num_sample_locations_at_compile_time>,
        LinearSimplexElementTraits<T, natural_dimension_at_compile_time,
                                   spatial_dimension_at_compile_time,
                                   num_sample_locations_at_compile_time>>::
                                 GradientInSpatialCoordinates>
    LinearSimplexElement<T, natural_dimension_at_compile_time,
                         spatial_dimension_at_compile_time,
                         num_sample_locations_at_compile_time>::
        CalcGradientInSpatialCoordinates(
            const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension,
                                                 num_nodes>>& xa) const {
  return this->DefaultCalcGradientInSpatialCoordinates(xa);
}

template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
typename LinearSimplexElement<T, natural_dimension_at_compile_time,
                              spatial_dimension_at_compile_time,
                              num_sample_locations_at_compile_time>::
    template ArrayNumSamples<typename IsoparametricElement<
        LinearSimplexElement<T, natural_dimension_at_compile_time,
                             spatial_dimension_at_compile_time,
                             num_sample_locations_at_compile_time>,
        LinearSimplexElementTraits<T, natural_dimension_at_compile_time,
                                   spatial_dimension_at_compile_time,
                                   num_sample_locations_at_compile_time>>::
                                 JacobianMatrix>
    LinearSimplexElement<T, natural_dimension_at_compile_time,
                         spatial_dimension_at_compile_time,
                         num_sample_locations_at_compile_time>::
        CalcJacobian(const Eigen::Ref<
                     const Eigen::Matrix<T, spatial_dimension, num_nodes>>& xa)
            const {
  return this->DefaultCalcJacobian(xa);
}

template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
typename LinearSimplexElement<T, natural_dimension_at_compile_time,
                              spatial_dimension_at_compile_time,
                              num_sample_locations_at_compile_time>::
    template ArrayNumSamples<typename IsoparametricElement<
        LinearSimplexElement<T, natural_dimension_at_compile_time,
                             spatial_dimension_at_compile_time,
                             num_sample_locations_at_compile_time>,
        LinearSimplexElementTraits<T, natural_dimension_at_compile_time,
                                   spatial_dimension_at_compile_time,
                                   num_sample_locations_at_compile_time>>::
                                 PseudoinverseJacobianMatrix>
    LinearSimplexElement<T, natural_dimension_at_compile_time,
                         spatial_dimension_at_compile_time,
                         num_sample_locations_at_compile_time>::
        CalcJacobianPseudoinverse(
            const ArrayNumSamples<JacobianMatrix>& jacobian) const {
  return this->DefaultCalcJacobianPseudoinverse(jacobian);
}

template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
typename LinearSimplexElement<T, natural_dimension_at_compile_time,
                              spatial_dimension_at_compile_time,
                              num_sample_locations_at_compile_time>::
    template ArrayNumSamples<typename LinearSimplexElement<
        T, natural_dimension_at_compile_time, spatial_dimension_at_compile_time,
        num_sample_locations_at_compile_time>::VectorNumNodes>
    LinearSimplexElement<T, natural_dimension_at_compile_time,
                         spatial_dimension_at_compile_time,
                         num_sample_locations_at_compile_time>::
        CalcShapeFunctions(
            const ArrayNumSamples<Vector<double, natural_dimension>>& locations)
            const {
  ArrayNumSamples<VectorNumNodes> S;
  for (int q = 0; q < num_sample_locations; ++q) {
    VectorNumNodes Sq;
    /* Sₐ = ξₐ₋₁ for a = 1, ..., num_nodes - 1. */
    for (int a = 1; a < num_nodes; ++a) {
      Sq(a) = locations[q](a - 1);
    }
    /* S₀ = 1−ξ₀ − ... − ξₙ₋₂. */
    Sq(0) = 1 - Sq.template tail<num_nodes - 1>().sum();
    S[q] = std::move(Sq);
  }
  return S;
}

template <typename T, int natural_dimension_at_compile_time,
          int spatial_dimension_at_compile_time,
          int num_sample_locations_at_compile_time>
const typename LinearSimplexElement<T, natural_dimension_at_compile_time,
                                    spatial_dimension_at_compile_time,
                                    num_sample_locations_at_compile_time>::
    template ArrayNumSamples<typename LinearSimplexElement<
        T, natural_dimension_at_compile_time, spatial_dimension_at_compile_time,
        num_sample_locations_at_compile_time>::GradientInParentCoordinates>
    LinearSimplexElement<T, natural_dimension_at_compile_time,
                         spatial_dimension_at_compile_time,
                         num_sample_locations_at_compile_time>::
        CalcGradientInParentCoordinates() {
  GradientInParentCoordinates dSdxi_q;
  dSdxi_q.template topRows<1>() = -1.0 * Vector<T, natural_dimension>::Ones();
  dSdxi_q.template bottomRows<natural_dimension>() =
      Eigen::Matrix<T, natural_dimension, natural_dimension>::Identity();
  ArrayNumSamples<GradientInParentCoordinates> dSdxi;
  dSdxi.fill(dSdxi_q);
  return dSdxi;
}

template class LinearSimplexElement<double, 2, 2, 2>;
template class LinearSimplexElement<double, 2, 2, 4>;
template class LinearSimplexElement<double, 2, 3, 4>;
template class LinearSimplexElement<double, 3, 3, 1>;
template class LinearSimplexElement<double, 3, 3, 2>;
template class LinearSimplexElement<double, 3, 3, 5>;
template class LinearSimplexElement<AutoDiffXd, 2, 2, 2>;
template class LinearSimplexElement<AutoDiffXd, 2, 2, 4>;
template class LinearSimplexElement<AutoDiffXd, 2, 3, 4>;
template class LinearSimplexElement<AutoDiffXd, 3, 3, 1>;
template class LinearSimplexElement<AutoDiffXd, 3, 3, 2>;
template class LinearSimplexElement<AutoDiffXd, 3, 3, 5>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
