#include "drake/multibody/fem/dev/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T, int NaturalDim>
std::vector<VectorX<T>>
LinearSimplexElement<T, NaturalDim>::CalcShapeFunctionsHelper() const {
  std::vector<VectorX<T>> S(this->num_sample_locations());
  const auto& locations = this->locations();
  for (int q = 0; q < this->num_sample_locations(); ++q) {
    VectorX<T> Sq = VectorX<T>::Zero(num_nodes());
    // Sₐ = ξₐ₋₁ for a = 1, ..., num_nodes() - 1
    for (int a = 1; a < num_nodes(); ++a) {
      Sq(a) = locations[q](a - 1);
    }
    // S₀ = 1−ξ₀ − ... − ξₙ₋₂
    Sq(0) = 1 - Sq.sum();
    S[q] = Sq;
  }
  return S;
}

template <typename T, int NaturalDim>
std::vector<MatrixX<T>> LinearSimplexElement<
    T, NaturalDim>::CalcGradientInParentCoordinatesHelper() const {
  MatrixX<T> dSdxi_q = MatrixX<T>::Zero(num_nodes(), NaturalDim);
  dSdxi_q.template topRows<1>() = -1.0 * VectorX<T>::Ones(NaturalDim);
  dSdxi_q.template bottomRows<NaturalDim>() =
      MatrixX<T>::Identity(NaturalDim, NaturalDim);
  std::vector<MatrixX<T>> dSdxi(this->num_sample_locations(), dSdxi_q);
  return dSdxi;
}

template class LinearSimplexElement<double, 2>;
template class LinearSimplexElement<double, 3>;
template class LinearSimplexElement<AutoDiffXd, 2>;
template class LinearSimplexElement<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
