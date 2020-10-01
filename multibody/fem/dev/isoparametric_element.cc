#include "drake/multibody/fem/dev/isoparametric_element.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace fem {
template <typename T, int NaturalDim>
std::vector<MatrixX<T>>
IsoparametricElement<T, NaturalDim>::CalcElementJacobian(
    const Eigen::Ref<const MatrixX<T>>& xa) const {
  DRAKE_DEMAND(xa.cols() == num_nodes());
  std::vector<MatrixX<T>> J(num_quads());
  const std::vector<MatrixX<T>>& dSdxi = CalcGradientInParentCoordinates();
  for (int q = 0; q < num_quads(); ++q) {
    J[q] = xa * dSdxi[q];
  }
  return J;
}

template <typename T, int NaturalDim>
std::vector<MatrixX<T>>
IsoparametricElement<T, NaturalDim>::CalcElementJacobianInverse(
    const Eigen::Ref<const MatrixX<T>>& xa) const {
  DRAKE_DEMAND(xa.cols() == num_nodes());
  // Number of spatial dimension.
  const int nsd = xa.rows();
  DRAKE_ASSERT(nsd >= NaturalDim);
  std::vector<MatrixX<T>> J = CalcElementJacobian(xa);
  return CalcElementJacobianInverse(J);
}

template <typename T, int NaturalDim>
std::vector<MatrixX<T>>
IsoparametricElement<T, NaturalDim>::CalcElementJacobianInverse(
    const std::vector<MatrixX<T>>& jacobian) const {
  DRAKE_DEMAND(static_cast<int>(jacobian.size()) == num_quads());
  std::vector<MatrixX<T>> J_inv(num_quads());
  // Number of spatial dimension.
  for (int q = 0; q < num_quads(); ++q) {
    Eigen::HouseholderQR<MatrixX<T>> qr(jacobian[q]);
    const int nsd = jacobian[q].rows();
    DRAKE_DEMAND(jacobian[q].cols() == NaturalDim);
    DRAKE_DEMAND(nsd >= NaturalDim);
    MatrixX<T> J_rotated(NaturalDim, NaturalDim);
    // J = QR and we need to solve R*x = I which is equivalent to Jx = Q*I.
    auto rhs = qr.householderQ() * MatrixX<T>::Identity(nsd, NaturalDim);
    auto J_rotated_inv = qr.solve(rhs);
    MatrixX<T> local_J_inv = MatrixX<T>::Zero(NaturalDim, nsd);
    local_J_inv.topLeftCorner(NaturalDim, NaturalDim) = J_rotated_inv;
    J_inv[q] = local_J_inv * qr.householderQ().transpose();
  }
  return J_inv;
}

template <typename T, int NaturalDim>
std::vector<T> IsoparametricElement<T, NaturalDim>::InterpolateScalar(
    const Eigen::Ref<const VectorX<T>>& ua) const {
  DRAKE_DEMAND(ua.size() == num_nodes());
  const std::vector<VectorX<T>>& S = CalcShapeFunctions();
  std::vector<T> interpolated_value(num_quads());
  for (int q = 0; q < num_quads(); ++q) {
    interpolated_value[q] = ua.dot(S[q]);
  }
  return interpolated_value;
}

template <typename T, int NaturalDim>
std::vector<VectorX<T>> IsoparametricElement<T, NaturalDim>::InterpolateVector(
    const Eigen::Ref<const MatrixX<T>>& ua) const {
  DRAKE_DEMAND(ua.cols() == num_nodes());
  const std::vector<VectorX<T>>& S = CalcShapeFunctions();
  std::vector<VectorX<T>> interpolated_value(num_quads());
  for (int q = 0; q < num_quads(); ++q) {
    interpolated_value[q] = ua * S[q];
  }
  return interpolated_value;
}

template class IsoparametricElement<double, 2>;
template class IsoparametricElement<double, 3>;
template class IsoparametricElement<AutoDiffXd, 2>;
template class IsoparametricElement<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace drake
