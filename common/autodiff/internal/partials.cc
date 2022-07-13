#include "drake/common/autodiff/internal/partials.h"

#include <array>
#include <cstdint>
#include <new>
#include <stdexcept>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/autodiff/internal/static_unit_vector.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace internal {

using autodiff::DerivativesConstXpr;
using autodiff::DerivativesMutableXpr;

CowVec CowVec::Allocate(int size) {
  DRAKE_DEMAND(size > 0);

  CowVec result;
  const size_t bytes = sizeof(UseCount) + (sizeof(double) * size);
  void* storage = std::malloc(bytes);
  if (storage == nullptr) {
    throw std::bad_alloc();
  }
  new (storage) UseCount(1);
  result.data_ = reinterpret_cast<double*>(
      static_cast<char*>(storage) + sizeof(UseCount));

  return result;
}

void CowVec::DecrementUseCount() {
  UseCount& counter = mutable_use_count();
  if (--counter == 0) {
    counter.~UseCount();
    std::free(&counter);
  }
}


namespace {

// Narrow an Eigen::Index to a plain int index, throwing when out-of-range.
int IndexToInt(Eigen::Index index) {
  if (index < 0) {
    throw std::out_of_range(fmt::format(
        "AutoDiffScalar derivatives size or offset {} is negative",
        index));
  }
  if (index > INT32_MAX) {
    throw std::out_of_range(fmt::format(
        "AutoDiffScalar derivatives size or offset {} is too large",
        index));
  }
  return static_cast<int>(index);
}

template <typename EigenVectorLike>
CowVec CopyIntoCowVec(const EigenVectorLike& value) {
  static_assert(EigenVectorLike::ColsAtCompileTime == 1, "Must be a Vector");
  const int size = IndexToInt(value.size());
  DRAKE_DEMAND(size > 0);
  CowVec result = CowVec::Allocate(size);
  for (int i = 0; i < size; ++i) {
    result.mutable_data()[i] = value[i];
  }
  return result;
}

}  // namespace

Partials::Partials(Eigen::Index size, Eigen::Index offset, double coeff)
    : magic_size_{IndexToInt(size)},
      unit_{IndexToInt(offset) + 1},
      coeff_{coeff} {
  if (offset >= size) {
    throw std::out_of_range(fmt::format(
        "AutoDiffScalar offset {} must be strictly less than size {}",
        offset, size));
  }
  if (coeff == 0.0) {
    SetZero();
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }
  // For really big unit derivatives, we can't use GetStaticUnitVector() for
  // the make_const_xpr() readback, so they'll need to live on the heap. If
  // this ends up being a problem, we can increase kMaxStaticVectorSize or
  // add a compressed (i.e., sparse) storage option to CowVec.
  if (size > kMaxStaticVectorSize) {
    *this = Partials{Eigen::VectorXd::Unit(size, offset)};
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
}

Partials::Partials(const Eigen::Ref<const Eigen::VectorXd>& value)
    : magic_size_{IndexToInt(value.size())} {
  if (magic_size_ == 0) {
    // These are redundant with the defaults but we'll repeat them for clarity.
    DRAKE_ASSERT(unit_ == 0);
    DRAKE_ASSERT(coeff_ == 0.0);
    DRAKE_ASSERT(storage_.data() == nullptr);
  } else if (magic_size_ == 1) {
    coeff_ = value[0];
    unit_ = 1;
    if (coeff_ == 0.0) {
      SetZero();
    }
    DRAKE_ASSERT(storage_.data() == nullptr);
  } else {
    DRAKE_ASSERT(unit_ == 0);
    coeff_ = 1.0;
    storage_ = CopyIntoCowVec(value);
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
}

Partials::Partials(const autodiff::DerivativesConstXpr& value)
    : magic_size_{IndexToInt(value.size())} {
  DRAKE_DEMAND(magic_size_ >= 2);
  DRAKE_ASSERT(unit_ == 0);
  coeff_ = 1.0;
  storage_ = CopyIntoCowVec(value);
  DRAKE_ASSERT_VOID(CheckInvariants());
}

// Abstraction function:
// - When `magic_size == 0`, the partials vector is empty (zero everywhere),
//   and may be combined with any other vector (i.e., it has no defined size).
// - When `magic_size < 0`:
//   - The partials vector is zero everywhere.
//   - The partials vector has a size of `-magic_size-1`.
// - When `magic_size > 0`:
//   - The partials vector has a size of `magic_size`.
//   - When `unit > 0`, the partials are zero everywhere except
//     for the `unit-1`th partial where it has the value of `coeff`.
//   - When `unit == 0` the partials value is `coeff * storage`
//     where the storage is a full-size dense vector on the heap.
//
// Representation invariant:
// - if magic_size <= 0:
//   - storage.data == null
// - if magic_size == 1:
//   - 0 < unit <= magic_size
//   - storage.data == null
// - if magic_size >= 2:
//   - 0 <= unit <= magic_size
//   - coeff != 0.0
//   - if unit > 0:
//     - storage.data == null
//   - if unit == 0:
//     - storage.data != null
//     - storage.data contains storage for exactly magic_size elements
void Partials::CheckInvariants() const {
  if (magic_size_ <= 0) {
    // When we are a known zero, we must not hold a reference any data.
    DRAKE_DEMAND(storage_.data() == nullptr);
  } else if (magic_size_ == 1) {
    // When we denote exactly one partial, it must always be stored inline.
    DRAKE_DEMAND(unit_ == 1);
    DRAKE_DEMAND(storage_.data() == nullptr);
  } else {
    // This is the general case (size >= 2).
    DRAKE_DEMAND(unit_ >= 0);
    DRAKE_DEMAND(unit_ <= magic_size_);
    DRAKE_DEMAND(coeff_ != 0.0);
    if (unit_ > 0) {
      // When we denote exactly one partial, it must always be stored inline.
      DRAKE_DEMAND(storage_.data() == nullptr);
    } else {
      // When we denote >1 partials, we must have allocated heap for them.
      DRAKE_DEMAND(storage_.data() != nullptr);
      DRAKE_DEMAND(storage_.get_use_count() >= 1);
    }
  }
}

void Partials::AddScaledImpl(double scale, const Partials& other) {
  DRAKE_ASSERT_VOID(CheckInvariants());
  DRAKE_ASSERT_VOID(other.CheckInvariants());

  // Our header file's inline functions only call into here when the addition
  // isn't a no-op.
  DRAKE_ASSERT(scale != 0.0);
  DRAKE_ASSERT(!is_known_zero());
  DRAKE_ASSERT(!other.is_known_zero());

  // If this check trips, then that means the user tried to mix AutoDiff partial
  // derivatives of non-unform sizes.
  if (size() != other.size()) {
    throw std::logic_error(fmt::format(
        "The size of AutoDiff partial derivative vectors must be uniform"
        " throughout the computation, but two different sizes ({} and {})"
        " were encountered at runtime. The dervatives were {} and {}.",
        size(), other.size(), make_const_xpr().eval().transpose(),
        other.make_const_xpr().eval().transpose()));
  }

  // From now on, size() and other.size() are the same (and strictly positive)
  // so we can abbreviate our name for that idea.
  DRAKE_ASSERT(magic_size_ > 0);
  DRAKE_ASSERT(magic_size_ == other.size());
  const int size = magic_size_;

  // The implementation that follows has many special cases for performance.
  //
  // The easiest way to follow along is to recognize that the case-analysis
  // branches first on the representation of `this` from lowest complexity
  // to highest complexity:
  //   Case 1 -- `this` is a scaled unit vector.
  //   Case 2 -- `this` is a scaled uniquely-owned heap vector.
  //   Case 3 -- `this` is a scaled shared-ownership heap vector.
  //
  // Then secondarily, we branch on the nature of `other`.

  // === Case 1 -- this is a scaled unit vector. ===

  if (is_unit()) {
    // If `other` is the same unit vector, the addition is cheap.
    if (unit_ == other.unit_) {
      coeff_ += scale * other.coeff_;
      if (coeff_ == 0.0) {
        SetZero();
      }
      return;
    }

    // Otherwise, we need to allocate storage for a non-unit vector.
    storage_ = CowVec::Allocate(size);
    double* new_data = storage_.mutable_data();

    // If this and other were both unit vectors, just set the two partials.
    if (other.is_unit()) {
      for (int i = 0; i < size; ++i) {
        new_data[i] = 0.0;
      }
      new_data[get_unit_index()] = coeff_;
      new_data[other.get_unit_index()] = scale * other.coeff_;
      coeff_ = 1.0;
      unit_ = 0;
      DRAKE_ASSERT_VOID(CheckInvariants());
      return;
    }

    // The `other` was a full vector, so we'll need to copy its data and then
    // add our unit. We'll inherit other's coeff so that we don't need to
    // multiply it through each vector element.
    const double new_coeff = scale * other.coeff_;
    const double this_relative_partial = coeff_ / new_coeff;
    const double* const other_data = other.storage_.data();
    for (int i = 0; i < size; ++i) {
      new_data[i] = other_data[i];
    }
    new_data[get_unit_index()] += this_relative_partial;
    coeff_ = new_coeff;
    unit_ = 0;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // === Case 2 -- `this` is a scaled uniquely-owned heap vector. ===

  // If we have exclusive storage, then we can mutate it in place.
  if (storage_.get_use_count() == 1) {
    double* this_data = storage_.mutable_data();

    // If `other` is a unit vector, the addition is cheap.
    if (other.is_unit()) {
      this_data[other.get_unit_index()] += (scale * other.coeff_) / coeff_;
      return;
    }

    // Otherwise, we'll need to fold in the other's data.
    // We use a linear combination and end up with a new coeff of 1.0.
    const double* const other_data = other.storage_.data();
    for (int i = 0; i < size; ++i) {
      this_data[i] = coeff_ * this_data[i]
          + (scale * other.coeff_) * other_data[i];
    }
    coeff_ = 1.0;
    unit_ = 0;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // === Case 3 -- `this` is a scaled shared-ownership heap vector. ===

  // It's not uncommon to multiply (x + x') by itself, which in autodiff looks
  // like x² + x*x' + x*x'. Recognizing that x' is the same across both addends
  // save us a copy.
  if (storage_.data() == other.storage_.data()) {
    DRAKE_DEMAND(!other.is_unit());
    coeff_ += scale * other.coeff_;
    if (coeff_ == 0.0) {
      SetZero();
    }
    return;
  }

  // We've run out of tricks; we have no choice now but to allocate fresh
  // storage to accumulate the result.
  CowVec new_storage = CowVec::Allocate(size);
  double* new_this_data = new_storage.mutable_data();
  const double* const original_this_data = storage_.data();

  // If `other` was a unit vector, we can copy our data and add the single unit.
  if (other.is_unit()) {
    for (int i = 0; i < size; ++i) {
      new_this_data[i] = original_this_data[i];
    }
    new_this_data[other.get_unit_index()] += scale * other.coeff_ / coeff_;
    storage_ = std::move(new_storage);
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // We've reached the most general case. Both partials are distinct, readonly
  // heap vectors.
  Eigen::Map<Eigen::VectorXd> output_vector(new_this_data, size);
  output_vector.noalias() =
      coeff_ * Eigen::Map<const Eigen::VectorXd>(storage_.data(), size)
      + (scale * other.coeff_) *
            Eigen::Map<const Eigen::VectorXd>(other.storage_.data(), size);
  storage_ = std::move(new_storage);
  coeff_ = 1.0;
  DRAKE_ASSERT_VOID(CheckInvariants());
}

DerivativesConstXpr Partials::make_const_xpr() const {
  const double* data = nullptr;
  int stride = 1;
  if (is_known_zero()) {
    // Instead of providing a pointer to an unboundedly large array of zeros,
    // we'll provide a pointer to a *single* zero and no-op our stride.
    // (We can't just point to &coeff_ because it might not be zero.)
    static const double kZero{0.0};
    stride = 0;
    data = &kZero;
  } else if (is_unit()) {
    data = GetStaticUnitVector(get_unit_index());
  } else {
    data = storage_.data();
  }
  return DerivativesConstXpr{coeff_, data, size(), stride};
}

DerivativesMutableXpr Partials::MakeMutableXpr() {
  double* xpr_data = nullptr;

  // Check if we need to copy-on-write.
  if (size() == 0) {
    // There's no need to allocate storage for an empty vector.
  } else if (size() == 1) {
    // There's no need to allocate storage for a 1-d vector.
    // We'll just store it directly in `coeff_`.
    if (magic_size_ < 0) {
      coeff_ = 0.0;
      magic_size_ = 1;
    }
    unit_ = 1;
    xpr_data = &coeff_;
    DRAKE_ASSERT_VOID(CheckInvariants());
  } else if ((magic_size_ > 0) && (unit_ == 0) && (coeff_ == 1.0) &&
             (storage_.get_use_count() == 1)) {
    // There's no need to allocate any storage because we already have
    // exclusively owned storage with a no-op coefficient.
    xpr_data = storage_.mutable_data();
  } else {
    // We need to allocate storage now.
    *this = Partials(make_const_xpr());
    DRAKE_ASSERT(unit_ == 0);
    DRAKE_ASSERT(coeff_ == 1.0);
    DRAKE_ASSERT(storage_.get_use_count() == 1);
    xpr_data = storage_.mutable_data();
  }

  return DerivativesMutableXpr{this, xpr_data, size()};
}

DerivativesMutableXpr Partials::SetFrom(
    const Eigen::Ref<const Eigen::VectorXd>& other) {
  *this = Partials(other);
  return MakeMutableXpr();
}

}  // namespace internal
}  // namespace drake
