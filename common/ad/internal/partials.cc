#include "drake/common/ad/internal/partials.h"

#include <algorithm>
#include <array>
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

/* A global data array that looks like this:

  [0 ... 0 1 0 ... 0]
           ^ There's a 1.0 in the middle at kMaxUnownedSize + 1.

By offsetting into this array, we can provide a zero vector or unit vector. */
constexpr std::array<double, 2 * StorageVec::kMaxUnownedSize + 1>
    kStaticVectorStorage = []() {
      std::array<double, 2 * StorageVec::kMaxUnownedSize + 1> value{};
      value[StorageVec::kMaxUnownedSize + 1] = 1.0;
      return value;
    }();

/* Returns a pointer to readonly memory of size kMaxUnownedSize containing all
zeros. */
const double* GetStaticZeroVector() {
  return kStaticVectorStorage.data();
}

/* Returns a pointer to readonly memory containing a unit vector using the given
vector `offset` as the sole non-zero (1.0) element.  The implied array size is
kMaxUnownedSize. For example, `offset == 0` returns [1, 0, 0, 0, ...] and
`offset == 2` returns `[0, 0, 1, 0, ...]`.
@pre 0 <= offset < kMaxUnownedSize */
const double* GetStaticUnitVector(int offset) {
  DRAKE_DEMAND(0 <= offset && offset < StorageVec::kMaxUnownedSize);
  return kStaticVectorStorage.data() + StorageVec::kMaxUnownedSize + 1 - offset;
}

/* Given a return value from GetStaticUnitVector(), returns the `offset`
argument that was originally passed in. */
int GetStaticUnitVectorOffset(const double* data) {
  return GetStaticUnitVector(0) - data;
}

}  // namespace

StorageVec StorageVec::MakeZero(int size) {
  DRAKE_ASSERT(1 <= size && size <= INT32_MAX);
  StorageVec result;
  result.size_ = size;
  result.is_owned_ = false;
  result.data_ = const_cast<double*>(GetStaticZeroVector());
  DRAKE_ASSERT_VOID(result.CheckInvariants());
  return result;
}

StorageVec StorageVec::MakeUnit(int size, int offset) {
  DRAKE_ASSERT(1 <= size && size <= INT32_MAX);
  DRAKE_ASSERT(0 <= offset && offset < size);
  StorageVec result;
  // For very large sizes, make_const_xpr() can't use GetStaticUnitVector() for
  // readback, so the storage will need to live on the heap. If this ends up
  // being a problem, we can increase kMaxUnownedSize or add some kind of
  // compressed (i.e., sparse list) storage option.
  if (size <= StorageVec::kMaxUnownedSize) {
    result.size_ = size;
    result.is_owned_ = false;
    result.data_ = const_cast<double*>(GetStaticUnitVector(offset));
  } else {
    result = Allocate(size);
    std::fill(result.data_, result.data_ + result.size_, 0.0);
    result.data_[offset] = 1.0;
  }
  DRAKE_ASSERT_VOID(result.CheckInvariants());
  return result;
}

StorageVec StorageVec::Allocate(int size) {
  DRAKE_ASSERT(2 <= size && size <= INT32_MAX);
  StorageVec result;
  result.size_ = size;
  result.is_owned_ = true;
  result.data_ = new double[size];
  DRAKE_ASSERT_VOID(result.CheckInvariants());
  return result;
}

StorageVec::StorageVec(const StorageVec& other) noexcept {
  DRAKE_ASSERT(0 <= other.size_ && other.size_ <= INT32_MAX);
  if (other.size_ > 0) {
    size_ = other.size_;
    is_owned_ = other.is_owned_;
    if (other.is_owned_) {
      data_ = new double[size_];
      std::copy(other.data_, other.data_ + size_, data_);
    } else {
      data_ = other.data_;
    }
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
}

StorageVec& StorageVec::operator=(const StorageVec& other) noexcept {
  DRAKE_ASSERT(0 <= other.size_ && other.size_ <= INT32_MAX);
  if (this != &other) {
    if (is_owned_ == false && other.is_owned_ == false) {
      size_ = other.size_;
      data_ = other.data_;
    } else if (is_owned_ == true && other.is_owned_ == false) {
      delete[] data_;
      size_ = other.size_;
      is_owned_ = false;
      data_ = other.data_;
    } else if (is_owned_ == false && other.is_owned_ == true) {
      size_ = other.size_;
      is_owned_ = true;
      data_ = new double[size_];
      std::copy(other.data_, other.data_ + other.size_, data_);
    } else /* both owned */ {
      if (size_ == other.size_) {
        std::copy(other.data_, other.data_ + other.size_, data_);
      } else {
        size_ = other.size_;
        delete[] data_;
        data_ = new double[size_];
        std::copy(other.data_, other.data_ + other.size_, data_);
      }
    }
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
  return *this;
}

StorageVec::~StorageVec() {
  DRAKE_ASSERT_VOID(CheckInvariants());
  if (is_owned_) {
    delete[] data_;
  }
}

void StorageVec::ConvertToOwned() {
  DRAKE_ASSERT(!is_owned());
  DRAKE_ASSERT(size_ >= 2);
  auto* owned_data = new double[size_];
  std::copy(data_, data_ + size_, owned_data);
  data_ = owned_data;
  is_owned_ = true;
  DRAKE_ASSERT_VOID(CheckInvariants());
}

void StorageVec::CheckInvariants() const {
  DRAKE_DEMAND(size_ >= 0);
  if (size_ == 0) {
    DRAKE_DEMAND(is_owned_ == false);
    DRAKE_DEMAND(data_ == nullptr);
  } else if (size_ == 1) {
    DRAKE_DEMAND(is_owned_ == false);
    DRAKE_DEMAND(data_ != nullptr);
    DRAKE_DEMAND(*data_ == 1.0);
  } else /* size_ >= 2 */ {
    DRAKE_DEMAND(data_ != nullptr);
  }
}

Partials::Partials(Eigen::Index size, Eigen::Index offset) : coeff_{1.0} {
  if (IndexToInt(offset) >= IndexToInt(size)) {
    throw std::out_of_range(fmt::format(
        "AutoDiff offset {} must be strictly less than size {}", offset, size));
  }
  storage_ = StorageVec::MakeUnit(size, offset);
}

Partials::Partials(const Eigen::Ref<const VectorXd>& value) : coeff_{1.0} {
  const int size = IndexToInt(value.size());
  if (size == 0) {
    // Nothing else to do.
  } else if (size == 1) {
    coeff_ = value[0];
    storage_ = StorageVec::MakeUnit(1, 0);
  } else {
    storage_ = StorageVec::Allocate(size);
    mutable_storage_view() = value;
  }
}

void Partials::MatchSizeOf(const Partials& other) {
  // Partials with size() == 0 may be freely combined with any other size.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    if (other.size() == 1) {
      storage_ = StorageVec::MakeZero(1);
    } else {
      storage_ = StorageVec::Allocate(other.size());
      mutable_storage_view().setZero();
    }
    coeff_ = 1.0;
    return;
  }
  ThrowIfDifferentSize(other);
}

void Partials::SetZero() {
  if (storage_.size() > 0) {
    if (storage_.size() <= StorageVec::kMaxUnownedSize) {
      storage_ = StorageVec::MakeZero(storage_.size());
    } else {
      mutable_storage_view().setZero();
    }
  }
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
    ad::DerivativesMutableXpr partials = MakeMutableXpr();
    for (int i = 0; i < size(); ++i) {
      const double x = partials[i];
      if ((x < 0) || (x > 0)) {
        partials[i] = x * total_factor;
      }
    }
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
    ad::DerivativesMutableXpr partials = MakeMutableXpr();
    for (int i = 0; i < size(); ++i) {
      const double x = partials[i];
      if ((x < 0) || (x > 0)) {
        partials[i] = x * total_factor;
      }
    }
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

  // Handle the case of this and/or other being full but filled with zeros.
  // TODO(jwnimmer-tri) Should we also special case if either coeff_ == 0.0?
  if (other.storage_.data() == GetStaticZeroVector()) {
    return;
  }
  if (storage_.data() == GetStaticZeroVector()) {
    *this = other;
    Mul(scale);
    return;
  }

  // Unlikely: non-finite scale; zero partials in `other` must not be scaled.
  if (!std::isfinite(scale)) [[unlikely]] {
    ad::DerivativesMutableXpr this_data = MakeMutableXpr();
    const ad::DerivativesConstXpr other_data = other.make_const_xpr();
    for (int i = 0; i < size(); ++i) {
      const double this_datum = this_data[i];
      const double other_datum = other_data[i];
      if ((other_datum < 0) || (other_datum > 0)) {
        this_data[i] = this_datum + other_datum * scale;
      }
    }
    return;
  }

  // Special case: both storage vectors are unit vectors.
  if (!storage_.is_owned() && !other.storage_.is_owned()) {
    // If `other` is the same unit vector, the addition is cheap.
    const int this_offset = GetStaticUnitVectorOffset(storage_.data());
    const int other_offset = GetStaticUnitVectorOffset(other.storage_.data());
    if (this_offset == other_offset) {
      coeff_ += scale * other.coeff_;
      return;
    }

    // Otherwise, we need to allocate storage for a non-unit vector.
    storage_ = StorageVec::Allocate(size());
    double* new_data = storage_.mutable_data();
    for (int i = 0; i < size(); ++i) {
      new_data[i] = 0.0;
    }
    new_data[this_offset] = coeff_;
    new_data[other_offset] = scale * other.coeff_;
    coeff_ = 1.0;
    return;
  }

  // TODO(jwnimmer-tri) It's probably worth special casing one unit vector
  // combined with one full vector.

  // Common case: perform the linear combination.
  //
  // TODO(jwnimmer-tri) Profile whether it's worth avoiding the multiplies in
  // the special case(s) where one or both of the `coeff_` factors are exactly
  // 0.0 or 1.0. So far, it doesn't benchmark faster in any suites we have, but
  // maybe it will in the future.
  //
  // N.B. Using a single Eigen expression for the following line is crucial
  // for performance. It compiles this better than a hand-written for-loop.
  if (!storage_.is_owned()) {
    storage_.ConvertToOwned();
  }
  mutable_storage_view().noalias() =
      coeff_ * storage_view() + (scale * other.coeff_) * other.storage_view();
  coeff_ = 1.0;
}

DerivativesConstXpr Partials::make_const_xpr() const {
  return DerivativesConstXpr{coeff_, storage_.data(), size(), /* stride = */ 1};
}

DerivativesMutableXpr Partials::MakeMutableXpr() {
  double* data = nullptr;
  if (size() == 0) {
    // Nothing to do.
  } else if (size() == 1) {
    // We'll just store it directly in `coeff_`.
    data = &coeff_;
  } else {
    if (!storage_.is_owned()) {
      storage_.ConvertToOwned();
    }
    if (coeff_ != 1.0) {
      mutable_storage_view() *= coeff_;
      coeff_ = 1.0;
    }
    data = storage_.mutable_data();
    DRAKE_ASSERT(data != nullptr);
  }
  return DerivativesMutableXpr{this, data, size()};
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

DerivativesMutableXpr Partials::SetFrom(
    const Eigen::Ref<const VectorXd>& other) {
  *this = Partials(other);
  return MakeMutableXpr();
}

// We want sizeof(AutoDiff) to be <= 32 bytes. Setting aside the 8 bytes for
// the value, that leaves 24 bytes available for the Partials.
static_assert(sizeof(Partials) <= 24);

}  // namespace internal
}  // namespace ad
}  // namespace drake
