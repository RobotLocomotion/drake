#include "drake/common/ad/internal/partials.h"

#include <cstdint>
#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace ad {
namespace internal {

namespace {

// Narrow an Eigen::Index to a plain int index, throwing when out-of-range.
// We do not allow more than ~2.1 million derivatives.
int IndexToInt(Eigen::Index index) {
  if (index < 0) {
    throw std::out_of_range(fmt::format(
        "AutoDiff derivatives size or offset {} is negative", index));
  }
  if (index > INT32_MAX) {
    throw std::out_of_range(fmt::format(
        "AutoDiff derivatives size or offset {} is too large", index));
  }
  return static_cast<int>(index);
}

}  // namespace

Partials::Partials(Eigen::Index size, Eigen::Index offset, double coeff) {
  if (IndexToInt(offset) >= IndexToInt(size)) {
    throw std::out_of_range(fmt::format(
        "AutoDiff derivatives offset {} must be strictly less than size {}",
        offset, size));
  }
  storage_ = Eigen::VectorXd::Zero(size);
  if (coeff != 0.0) {
    coeff_ = 1.0;
    storage_[offset] = coeff;
  }
}

Partials::Partials(const Eigen::Ref<const Eigen::VectorXd>& value)
    : coeff_{1.0}, storage_{value} {
  // Called for its side-effect of throwing on too-large sizes.
  IndexToInt(value.size());
}

void Partials::MatchSizeOf(const Partials& other) {
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    coeff_ = 0.0;
    storage_ = Eigen::VectorXd::Zero(other.size());
    return;
  }
  ThrowIfDifferentSize(other);
}

void Partials::AddScaled(double scale, const Partials& other) {
  // Handle the case of this and/or other being empty.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    *this = other;
    coeff_ *= scale;
    return;
  }

  ThrowIfDifferentSize(other);

  // We use a linear combination and end up with a new coeff of 1.0.
  double* this_data = storage_.data();
  const double* const other_data = other.storage_.data();
  for (int i = 0; i < storage_.size(); ++i) {
    this_data[i] =
        coeff_ * this_data[i] + (scale * other.coeff_) * other_data[i];
  }
  coeff_ = 1.0;
}

DerivativesConstXpr Partials::make_const_xpr() const {
  constexpr int stride = 1;
  return DerivativesConstXpr{coeff_, storage_.data(), size(), stride};
}

Eigen::VectorXd& Partials::GetRawStorageMutable() {
  if (coeff_ != 1.0) {
    storage_ *= coeff_;
    coeff_ = 1.0;
  }
  return storage_;
}

void Partials::ThrowIfDifferentSize(const Partials& other) {
  // If this check trips, then that means the user tried to mix AutoDiff partial
  // derivatives of non-unform sizes.
  if (size() != other.size()) {
    throw std::logic_error(fmt::format(
        "The size of AutoDiff partial derivative vectors must be uniform"
        " throughout the computation, but two different sizes ({} and {})"
        " were encountered at runtime.",
        size(), other.size()));
  }
}

}  // namespace internal
}  // namespace ad
}  // namespace drake
