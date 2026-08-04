#include "drake/common/ad/internal/derivatives_xpr.h"

#include "drake/common/ad/internal/partials.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace ad {

using Stride = internal::DerivativesStride;

DerivativesConstXpr::DerivativesConstXpr(double coeff, const double* data,
                                         int size, int stride)
    : internal::DerivativesConstXprBase(
          coeff * Eigen::Map<const Eigen::VectorXd, /* Options = */ 0, Stride>(
                      data, size, Stride(0, stride))) {
  DRAKE_DEMAND(size == 0 || data != nullptr);
}

DerivativesMutableXpr::DerivativesMutableXpr(internal::Partials* backreference,
                                             double* data, int size)
    : Eigen::Map<Eigen::VectorXd>(data, size), backreference_{backreference} {
  DRAKE_DEMAND(size == 0 || data != nullptr);
  DRAKE_DEMAND(backreference_ != nullptr);
}

DerivativesMutableXpr DerivativesMutableXpr::resize(Eigen::Index rows,
                                                    Eigen::Index cols) {
  DRAKE_THROW_UNLESS(rows >= 0);
  DRAKE_THROW_UNLESS(cols == 1);
  return SetFrom(Eigen::VectorXd::Zero(rows));
}

DerivativesMutableXpr DerivativesMutableXpr::conservativeResize(
    Eigen::Index rows, Eigen::Index cols) {
  DRAKE_THROW_UNLESS(rows >= 0);
  DRAKE_THROW_UNLESS(cols == 1);

  if (rows != size()) {
    const int old_size = size();
    Eigen::VectorXd new_value(rows);
    for (int i = 0; i < old_size; ++i) {
      if (i >= rows) {
        break;
      }
      new_value[i] = coeff(i);
    }
    for (int i = old_size; i < rows; ++i) {
      new_value[i] = 0.0;
    }
    SetFrom(new_value);
  }

  return *this;
}

DerivativesMutableXpr DerivativesMutableXpr::SetFrom(
    const Eigen::Ref<const Eigen::VectorXd>& other) {
  DRAKE_DEMAND(backreference_ != nullptr);
  return backreference_->SetFrom(other);
}

}  // namespace ad
}  // namespace drake
