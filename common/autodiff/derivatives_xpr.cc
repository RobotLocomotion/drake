#include "drake/common/autodiff/derivatives_xpr.h"

#include "drake/common/autodiff/internal/partials.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace autodiff {

// TODO(jwnimmer-tri) Use placement-new when changing the maps, so that the
// user doesn't need to care about the returned value. No, I'm not kidding.
// https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html#TutorialMapPlacementNew

using Stride = internal::DerivativesStride;

DerivativesConstXpr::DerivativesConstXpr(
    double coeff, const double* data, int size, int stride)
    : internal::DerivativesConstXprBase(
          coeff * Eigen::Map<const Eigen::VectorXd, 0 /* Options */, Stride>(
              data, size, Stride(0, stride))) {
  DRAKE_DEMAND(size == 0 || data != nullptr);
}

namespace {
// Helper for both implementations of with_explicit_zeros().
// Returns true iff the zero-padding should be added.
bool NeedsExplicitZeros(Eigen::Index old_size, Eigen::Index new_size) {
  DRAKE_THROW_UNLESS(old_size >= 0);
  DRAKE_THROW_UNLESS(new_size >= 0);
  if (old_size == 0) {
    return new_size > 0;
  }
  DRAKE_THROW_UNLESS(new_size == old_size);
  return false;
}
}  // namespace

DerivativesConstXpr
DerivativesConstXpr::with_explicit_zeros(Eigen::Index size) {
  static const double kZero{0.0};
  if (NeedsExplicitZeros(this->size(), size)) {
    const double coeff = 0.0;
    const int stride = 0;
    return DerivativesConstXpr(coeff, &kZero, size, stride);
  }
  return *this;
}

DerivativesMutableXpr::DerivativesMutableXpr(
    internal::Partials* backreference, double* data, int size)
    : Eigen::Map<Eigen::VectorXd>(data, size),
      backreference_{backreference} {
  DRAKE_ASSERT(backreference_ != nullptr);
}

DerivativesMutableXpr
DerivativesMutableXpr::with_explicit_zeros(Eigen::Index size) {
  return NeedsExplicitZeros(this->size(), size) ? resize(size) : *this;
}

DerivativesMutableXpr DerivativesMutableXpr::resize(
    Eigen::Index rows, Eigen::Index cols) {
  DRAKE_THROW_UNLESS(rows >= 0);
  DRAKE_THROW_UNLESS(cols == 1);
  return SetFrom(Eigen::VectorXd::Zero(rows));
}

DerivativesMutableXpr
DerivativesMutableXpr::conservativeResize(Eigen::Index rows,
                                                  Eigen::Index cols) {
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

}  // namespace autodiff
}  // namespace drake
