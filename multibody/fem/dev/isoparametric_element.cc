#include "drake/multibody/fem/dev/isoparametric_element.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T, int NaturalDim>
std::vector<MatrixX<T>>
IsoparametricElement<T, NaturalDim>::CalcJacobian(
    const Eigen::Ref<const MatrixX<T>>& xa) const {
  DRAKE_DEMAND(xa.cols() == num_nodes());
  std::vector<MatrixX<T>> dxdxi(num_sample_locations());
  const std::vector<MatrixX<T>>& dSdxi = CalcGradientInParentCoordinates();
  for (int q = 0; q < num_sample_locations(); ++q) {
    dxdxi[q] = xa * dSdxi[q];
  }
  return dxdxi;
}

template <typename T, int NaturalDim>
std::vector<MatrixX<T>>
IsoparametricElement<T, NaturalDim>::CalcJacobianInverse(
    const Eigen::Ref<const MatrixX<T>>& xa) const {
  DRAKE_DEMAND(xa.cols() == num_nodes());
  // Number of spatial dimension.
  const int nsd = xa.rows();
  DRAKE_ASSERT(nsd >= NaturalDim);
  std::vector<MatrixX<T>> dxdxi = CalcJacobian(xa);
  return CalcJacobianInverse(dxdxi);
}

template <typename T, int NaturalDim>
std::vector<MatrixX<T>>
IsoparametricElement<T, NaturalDim>::CalcJacobianInverse(
    const std::vector<MatrixX<T>>& dxdxi) const {
  DRAKE_DEMAND(static_cast<int>(dxdxi.size()) == num_sample_locations());
  std::vector<MatrixX<T>> dxidx(num_sample_locations());
  for (int q = 0; q < num_sample_locations(); ++q) {
    Eigen::HouseholderQR<MatrixX<T>> qr(dxdxi[q]);
    const int nsd = dxdxi[q].rows();
    DRAKE_DEMAND(dxdxi[q].cols() == NaturalDim);
    DRAKE_DEMAND(nsd >= NaturalDim);
    MatrixX<T> dxdxi_rotated(NaturalDim, NaturalDim);
    // J = QR and we need to solve R*x = I which is equivalent to Jx = Q*I.
    auto rhs = qr.householderQ() * MatrixX<T>::Identity(nsd, NaturalDim);
    auto dxidx_rotated = qr.solve(rhs);
    MatrixX<T> local_dxidx = MatrixX<T>::Zero(NaturalDim, nsd);
    local_dxidx.topLeftCorner(NaturalDim, NaturalDim) = dxidx_rotated;
    dxidx[q] = local_dxidx * qr.householderQ().transpose();
  }
  return dxidx;
}

template <typename T, int NaturalDim>
std::vector<T> IsoparametricElement<T, NaturalDim>::InterpolateScalar(
    const Eigen::Ref<const VectorX<T>>& ua) const {
  DRAKE_DEMAND(ua.size() == num_nodes());
  const std::vector<VectorX<T>>& S = CalcShapeFunctions();
  std::vector<T> interpolated_value(num_sample_locations());
  for (int q = 0; q < num_sample_locations(); ++q) {
    interpolated_value[q] = ua.dot(S[q]);
  }
  return interpolated_value;
}

template <typename T, int NaturalDim>
std::vector<VectorX<T>> IsoparametricElement<T, NaturalDim>::InterpolateVector(
    const Eigen::Ref<const MatrixX<T>>& ua) const {
  DRAKE_DEMAND(ua.cols() == num_nodes());
  const std::vector<VectorX<T>>& S = CalcShapeFunctions();
  std::vector<VectorX<T>> interpolated_value(num_sample_locations());
  for (int q = 0; q < num_sample_locations(); ++q) {
    interpolated_value[q] = ua * S[q];
  }
  return interpolated_value;
}

template class IsoparametricElement<double, 2>;
template class IsoparametricElement<double, 3>;
template class IsoparametricElement<AutoDiffXd, 2>;
template class IsoparametricElement<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
