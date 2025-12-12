#include "drake/common/ad/internal/partials.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::VectorXd;

// Narrow an Eigen::Index to a plain int index, throwing when out-of-range.
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

Partials::Partials(Eigen::Index size, Eigen::Index offset)
    : coeff_{1.0}, storage_{VectorXd::Zero(IndexToInt(size))} {
  if (IndexToInt(offset) >= size) {
    throw std::out_of_range(fmt::format(
        "AutoDiff offset {} must be strictly less than size {}", offset, size));
  }
  storage_[offset] = 1.0;
}

Partials::Partials(const Eigen::Ref<const VectorXd>& value)
    : coeff_{1.0}, storage_{value} {
  // Called for its side-effect of throwing on too-large sizes. (Unfortunately,
  // it's unrealistic to unit test this because it requires a >16 GiB value.)
  IndexToInt(value.size());
}

void Partials::MatchSizeOf(const Partials& other) {
  // Partials with size() == 0 may be freely combined with any other size.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    storage_ = VectorXd::Zero(other.size());
    coeff_ = 1.0;
    return;
  }
  ThrowIfDifferentSize(other);
}

void Partials::SetZero() {
  storage_.setZero();
  // The `coeff_` here could be set to nearly any number (especially 0.0), but
  // we'll stick with 1.0 as the "canonical" spelling of "unscaled storage".
  // That might open the door to future optimizations (e.g., skipping extra
  // rescaling during linear combinations in case the coeff was exactly 1.)
  coeff_ = 1.0;
}

void Partials::Mul(double factor) {
  if (std::isfinite(factor)) [[likely]] {
    // The `coeff_` will remain finite; multiplying two finite numbers always
    // produces a finite result.
    coeff_ *= factor;
  } else {
    // Do a "sparse" multiply of only the non-zero values in storage. If the
    // value is zero or NaN, then we'll leave it alone.
    const double total_factor = coeff_ * factor;  // This will be non-finite.
    for (int i = 0; i < size(); ++i) {
      const double x = storage_[i];
      if ((x < 0) || (x > 0)) {
        storage_[i] = x * total_factor;
      }
    }
    coeff_ = 1.0;
  }
}

void Partials::Div(double factor) {
  if (!std::isnan(factor) && factor != 0.0) [[likely]] {
    // The `coeff_` will remain finite; dividing finite by ±∞ produces ±0.
    coeff_ /= factor;
  } else {
    // Do a "sparse" division of only the non-zero values in storage. If the
    // value is zero or NaN, then we'll leave it alone.
    const double total_factor = coeff_ / factor;
    for (int i = 0; i < size(); ++i) {
      const double x = storage_[i];
      if ((x < 0) || (x > 0)) {
        storage_[i] = x * total_factor;
      }
    }
    coeff_ = 1.0;
  }
}

void Partials::Add(const Partials& other) {
  AddScaled(1.0, other);
}

void Partials::AddScaled(double scale, const Partials& other) {
  // Handle the case of this and/or other being empty (treated as all-zero).
  // Partials with size() == 0 may be freely combined with any other size.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    *this = other;
    Mul(scale);
    return;
  }
  ThrowIfDifferentSize(other);

  // Common case: perform the linear combination.
  //
  // TODO(jwnimmer-tri) Profile whether it's worth avoiding the multiplies in
  // the special case(s) where one or both of the `coeff_` factors are exactly
  // 0.0 or 1.0. So far, it doesn't benchmark faster in any suites we have, but
  // maybe it will in the future.
  if (std::isfinite(scale)) [[likely]] {
    // N.B. Using a single Eigen expression for the following line is crucial
    // for performance. It compiles this better than a hand-written for-loop.
    storage_.noalias() =
        coeff_ * storage_ + (scale * other.coeff_) * other.storage_;
    coeff_ = 1.0;
    return;
  }

  // Unlikely: non-finite scale; zero partials in `other` must not be scaled.
  for (int i = 0; i < size(); ++i) {
    const double this_datum = coeff_ * storage_[i];
    const double other_datum = other.coeff_ * other.storage_[i];
    if ((other_datum < 0) || (other_datum > 0)) {
      storage_[i] = this_datum + other_datum * scale;
    } else {
      storage_[i] = this_datum;
    }
  }
  coeff_ = 1.0;
}

DerivativesConstXpr Partials::make_const_xpr() const {
  return DerivativesConstXpr{coeff_, storage_.data(), size(), /* stride = */ 1};
}

VectorXd& Partials::GetRawStorageMutable() {
  if (coeff_ != 1.0) {
    storage_ *= coeff_;
    coeff_ = 1.0;
  }
  return storage_;
}

void Partials::ThrowIfDifferentSize(const Partials& other) {
  // If this check trips, then that means the user tried to mix AutoDiff partial
  // derivatives of non-uniform sizes.
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
