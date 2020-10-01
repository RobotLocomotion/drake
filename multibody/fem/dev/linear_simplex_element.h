#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/isoparametric_element.h"

namespace drake {
namespace fem {
template <typename T, int NaturalDim>
class LinearSimplexElement : public IsoparametricElement<T, NaturalDim> {
 public:
  using typename IsoparametricElement<T, NaturalDim>::VectorD;
  explicit LinearSimplexElement(std::vector<VectorD> quadrature_locations)
      : IsoparametricElement<T, NaturalDim>(std::move(quadrature_locations)) {
    S_ = CalcShapeFunctionsHelper();
    dSdxi_ = CalcGradientInParentCoordinatesHelper();
  }

  int num_nodes() const final { return NaturalDim + 1; }

  const std::vector<VectorX<T>>& CalcShapeFunctions() const { return S_; }

  const std::vector<MatrixX<T>>& CalcGradientInParentCoordinates() const {
    return dSdxi_;
  }

 private:
  std::vector<VectorX<T>> CalcShapeFunctionsHelper() const;

  std::vector<MatrixX<T>> CalcGradientInParentCoordinatesHelper() const;

  // Shape functions evaluated at quadrature points specified by quadrature_.
  std::vector<VectorX<T>> S_;
  // Shape function derivatives evaluated at quadrature points specified by
  // quadrature_.
  std::vector<MatrixX<T>> dSdxi_;
};
}  // namespace fem
}  // namespace drake
