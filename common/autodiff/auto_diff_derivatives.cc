#include "drake/common/autodiff/auto_diff_derivatives.h"

#include "drake/common/autodiff/internal/partials.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace autodiff {

AutoDiffDerivativesConstXpr::AutoDiffDerivativesConstXpr(
    double coeff, const double* data, int size)
    : internal::AutoDiffDerivativesConstXprBase(
          coeff * Eigen::Map<const Eigen::VectorXd>(data, size)) {}

AutoDiffDerivativesMutableXpr::AutoDiffDerivativesMutableXpr(
    internal::Partials* backreference)
    : Eigen::Map<Eigen::VectorXd>(
          // The const_cast is safe because only Partials::MakeMutableXpr
          // (a non-const member function) calls this and when it does so
          // the storage is exclusively owned.
          const_cast<double*>(backreference->storage_.data()),
          backreference->size_),
      backreference_{backreference} {}

AutoDiffDerivativesMutableXpr AutoDiffDerivativesMutableXpr::resize(
    Eigen::Index rows, Eigen::Index cols) {
  DRAKE_THROW_UNLESS(rows >= 0);
  DRAKE_THROW_UNLESS(cols == 1);
  return SetFrom(Eigen::VectorXd::Zero(rows));
}

AutoDiffDerivativesMutableXpr AutoDiffDerivativesMutableXpr::SetFrom(
    const Eigen::Ref<const Eigen::VectorXd>& other) {
  DRAKE_DEMAND(backreference_ != nullptr);
  return backreference_->SetFrom(other);
}

}  // namespace autodiff
}  // namespace drake
