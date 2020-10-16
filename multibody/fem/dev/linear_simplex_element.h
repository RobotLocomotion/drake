#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/isoparametric_element.h"

namespace drake {
namespace multibody {
namespace fem {
/** A concrete isoparametric element for linear simplex elements (triangles and
 tetrahedrons). In parent domain, the simplex's node are laid out in the
 following way: the 0-th node is placed at the origin and the i-th node for 0 <
 i <= NaturalDim is placed at the point whose i-th coordinate is 1 and all other
 coordinates are 0. */
template <typename T, int NaturalDim>
class LinearSimplexElement : public IsoparametricElement<T, NaturalDim> {
 public:
  using typename IsoparametricElement<T, NaturalDim>::VectorD;
  explicit LinearSimplexElement(std::vector<VectorD> locations)
      : IsoparametricElement<T, NaturalDim>(std::move(locations)) {
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

  // Shape functions evaluated at points specified at construction.
  std::vector<VectorX<T>> S_;
  // Shape function derivatives evaluated at points specified at construction.
  std::vector<MatrixX<T>> dSdxi_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
