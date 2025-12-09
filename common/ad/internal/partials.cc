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

Partials::Partials(Eigen::Index size, Eigen::Index offset, double coeff)
    : storage_{VectorXd::Zero(IndexToInt(size))} {
  if (IndexToInt(offset) >= size) {
    throw std::out_of_range(fmt::format(
        "AutoDiff offset {} must be strictly less than size {}", offset, size));
  }
  if (coeff != 0.0) {
    coeff_ = 1.0;
    storage_[offset] = coeff;
  }
}

Partials::Partials(const Eigen::Ref<const VectorXd>& value)
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
    storage_ = VectorXd::Zero(other.size());
    return;
  }
  ThrowIfDifferentSize(other);
}

// TODO(jwnimmer-tri) Try to inline this again?
void Partials::Mul(double factor) {
  if (std::isfinite(factor)) [[likely]] {
    coeff_ *= factor;
  } else {
    const double new_coeff = coeff_ * factor;
    for (int i = 0; i < size(); ++i) {
      const double x = storage_[i];
      storage_[i] = (x == 0) ? 0 : x * new_coeff;
    }
    coeff_ = 1.0;
  }
}

// TODO(jwnimmer-tri) Try to inline this again?
void Partials::Div(double factor) {
  if (!std::isnan(factor) && factor != 0) [[likely]] {
    coeff_ /= factor;
  } else {
    const double new_coeff = coeff_ / factor;
    for (int i = 0; i < size(); ++i) {
      const double x = storage_[i];
      storage_[i] = (x == 0) ? 0 : x * new_coeff;
    }
    coeff_ = 1.0;
  }
}

void Partials::AddScaled(double scale, const Partials& other) {
  // Handle the case of this and/or other being empty.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    *this = other;
    Mul(scale);
    return;
  }
  ThrowIfDifferentSize(other);

  // Special case: if all coefficients are no-ops, we don't need any multiplies.
  if (coeff_ == 1.0 && other.coeff_ == 1.0 && scale == 1.0) {
    double* this_data = storage_.data();
    const double* const other_data = other.storage_.data();
    for (int i = 0; i < storage_.size(); ++i) {
      this_data[i] += other_data[i];
    }
    return;
  }

  // Common case: perform the linear combination and end up with a coeff of 1.0.
  if (std::isfinite(scale)) [[likely]] {
    double* this_data = storage_.data();
    const double* const other_data = other.storage_.data();
    const double other_coeff_scaled = scale * other.coeff_;
    for (int i = 0; i < storage_.size(); ++i) {
      this_data[i] = coeff_ * this_data[i] + other_coeff_scaled * other_data[i];
    }
    coeff_ = 1.0;
    return;
  }

  // Unlikely: non-finite scale; any zero partials must remain unchanged.
  for (int i = 0; i < size(); ++i) {
    const double this_datum = coeff_ * storage_[i];
    const double other_datum = other.coeff_ * other.storage_[i];
    storage_[i] = this_datum + ((other_datum == 0) ? 0 : other_datum * scale);
  }
  coeff_ = 1.0;
}

DerivativesConstXpr Partials::make_const_xpr() const {
  if (coeff_ == 0.0) {
    // A call to `Partials::SetZero()` doesn't wipe the `storage_`, it only
    // resets the `coeff_` to zero. In case there were any non-finite numbers
    // hanging around in `storage_`, we must be careful not to multiply them.
    // Therefore we can't use `storage_.data()`, so instead use a static zero
    // with zero stride to populate the Eigen::Map.
    static const double kZero{0.0};
    constexpr int stride = 0;
    return DerivativesConstXpr{coeff_, &kZero, size(), stride};
  } else {
    constexpr int stride = 1;
    return DerivativesConstXpr{coeff_, storage_.data(), size(), stride};
  }
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
