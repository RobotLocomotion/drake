#include "drake/common/ad/internal/partials.h"

#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace ad {
namespace internal {

namespace {

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
    : derivatives_{Eigen::VectorXd::Zero(IndexToInt(size))} {
  if (IndexToInt(offset) >= size) {
    throw std::out_of_range(fmt::format(
        "AutoDiff offset {} must be strictly less than size {}", offset, size));
  }
  derivatives_[offset] = coeff;
}

Partials::Partials(const Eigen::Ref<const Eigen::VectorXd>& value)
    : derivatives_{value} {}

void Partials::MatchSizeOf(const Partials& other) {
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    derivatives_ = Eigen::VectorXd::Zero(other.size());
    return;
  }
  ThrowIfDifferentSize(other);
}

void Partials::Mul(double factor) {
  if (std::isfinite(factor)) [[likely]] {
    derivatives_ *= factor;
  } else {
    for (int i = 0; i < size(); ++i) {
      const double x = derivatives_[i];
      derivatives_[i] = (x == 0) ? 0 : x * factor;
    }
  }
}

void Partials::Div(double factor) {
  if (!std::isnan(factor) && factor != 0) [[likely]] {
    derivatives_ /= factor;
  } else {
    for (int i = 0; i < size(); ++i) {
      const double x = derivatives_[i];
      derivatives_[i] = (x == 0) ? 0 : x / factor;
    }
  }
}

void Partials::Add(const Partials& other) {
  AddScaled(1.0, other);
}

void Partials::AddScaled(double scale, const Partials& other) {
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    if (std::isfinite(scale)) [[likely]] {
      derivatives_ = other.derivatives_ * scale;
    } else {
      derivatives_ = Eigen::VectorXd(other.size());
      for (int i = 0; i < other.size(); ++i) {
        const double x = other.derivatives_[i];
        derivatives_[i] = (x == 0) ? 0 : x * scale;
      }
    }
    return;
  }
  ThrowIfDifferentSize(other);
  if (std::isfinite(scale)) [[likely]] {
    derivatives_ += other.derivatives_ * scale;
  } else {
    for (int i = 0; i < size(); ++i) {
      const double x = other.derivatives_[i];
      derivatives_[i] += (x == 0) ? 0 : x * scale;
    }
  }
}

void Partials::ThrowIfDifferentSize(const Partials& other) {
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
