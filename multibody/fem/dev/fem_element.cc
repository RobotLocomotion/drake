#include "drake/multibody/fem/dev/fem_element.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T, int NaturalDim>
VectorX<T> FemElement<T, NaturalDim>::CalcResidual(const FemState<T>& s) const {
  VectorX<T> residual(num_nodes() * num_spatial_dim());
  CalcResidual(s, &residual);
  return residual;
}

template <typename T, int NaturalDim>
void FemElement<T, NaturalDim>::CalcResidual(
    const FemState<T>& s, EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  DRAKE_DEMAND(residual->size() == num_spatial_dim() * num_nodes());
  DoCalcResidual(s, residual);
}

template <typename T, int NaturalDim>
FemElement<T, NaturalDim>::FemElement(
    ElementIndex element_index,
    const IsoparametricElement<T, NaturalDim>& shape,
    const Quadrature<T, NaturalDim>& quadrature,
    const std::vector<NodeIndex>& node_indices)
    : element_index_(element_index),
      shape_(shape),
      quadrature_(quadrature),
      node_indices_(node_indices) {
  DRAKE_DEMAND(element_index_.is_valid());
  DRAKE_DEMAND(shape_.num_nodes() == static_cast<int>(node_indices_.size()));
}

template class FemElement<double, 2>;
template class FemElement<double, 3>;
template class FemElement<AutoDiffXd, 2>;
template class FemElement<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
