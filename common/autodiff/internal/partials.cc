#include "drake/common/autodiff/internal/partials.h"

#include <array>
#include <cstdint>
#include <new>
#include <stdexcept>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_throw.h"

namespace drake {
namespace internal {

using autodiff::AutoDiffDerivativesConstXpr;
using autodiff::AutoDiffDerivativesMutableXpr;

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

}  //  namespace

CowVec CowVec::Allocate(int size) {
  DRAKE_DEMAND(size >= 0);

  CowVec result;
  if (size == 0) {
    return result;
  }

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

CowVec CowVec::Copy(const Eigen::Ref<const Eigen::VectorXd>& value) {
  const int size = IndexToInt(value.size());
  CowVec result = Allocate(size);
  for (int i = 0; i < size; ++i) {
    result.data_[i] = value[i];
  }
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

// The limit (inclusive) on the partials.size() for which we can use the
// compressed unit vector representation. For all compressed unit vectors,
// we need to have pointers to constant memory backing them for our read-
// only views. This constant determines how much such memory we allocate.
// The (singleton) heap allocation is 16x as large as this number.
constexpr int kMaxUnitSize = 1023;

// For a given unit vector offset, returns a pointer to static memory that
// contains that unit vector.  For example, `offset == 0` returns [1.0, 0.0,
// 0.0, etc...] and `offset == 2` returns `[0.0, 0.0, 1.0, 0.0, 0.0, etc...].
// The array contains kMaxUnitSize elements.
const double* GetStaticUnitVector(int offset) {
  DRAKE_DEMAND(offset >= 0);
  DRAKE_DEMAND(offset < kMaxUnitSize);
  // Create a global data array that looks like this:
  //
  //   [0 ... 0 1 0 ... 0]
  //            ^ There's a 1.0 in the middle.
  //
  // By offsetting into this array, we can return any unit vector up to
  // kMaxUnitSize.
  const constexpr int kMiddle = kMaxUnitSize - 1;
  struct Unit {
    Unit() {
      data = {};
      data[kMiddle] = 1.0;
    }
    std::array<double, kMiddle + kMaxUnitSize> data;
  };
  static const Unit* const storage = new Unit;  // This is never destroyed.
  return &(storage->data[kMiddle - offset]);
}

}  // namespace

Partials::Partials(Eigen::Index size, Eigen::Index offset)
    : size_{IndexToInt(size)},
      unit_{IndexToInt(offset) + 1},
      coeff_{1.0} {
  if (offset >= size) {
    throw std::out_of_range(fmt::format(
        "AutoDiffScalar offset {} must be strictly less than size {}",
        offset, size));
  }
  // For really big unit derivatives, we can't view_as_eigen() on the way out,
  // so they'll need to live on the heap.
  if (size > kMaxUnitSize) {
    *this = Partials{Eigen::VectorXd::Unit(size, offset)};
  }
}

Partials::Partials(const Eigen::Ref<const Eigen::VectorXd>& value)
    : size_{IndexToInt(value.size())},
      coeff_{size_ > 0 ? 1.0 : 0.0},
      storage_{CowVec::Copy(value)} {
  DRAKE_ASSERT_VOID(CheckInvariants());
}

void Partials::CheckInvariants() const {
  DRAKE_DEMAND(size_ >= 0);
  DRAKE_DEMAND(unit_ >= 0);
  if (size_ == 0) {
    DRAKE_DEMAND(storage_.data() == nullptr);
  } else {
    DRAKE_DEMAND(coeff_ != 0.0);
    DRAKE_DEMAND(unit_ <= size_);
    if (unit_ == 0) {
      DRAKE_DEMAND(storage_.data() != nullptr);
      DRAKE_DEMAND(storage_.get_use_count() >= 1);
    }
  }
}

void Partials::AddImpl(const Partials& other) {
  // TODO(jwnimmer-tri) Perhaps we could save some dummy multiplies (by 1.0)
  // with a slightly more careful implementation?
  AddScaledImpl(1.0, other);
}

void Partials::AddScaledImpl(double scale, const Partials& other) {
  DRAKE_ASSERT_VOID(CheckInvariants());
  DRAKE_ASSERT_VOID(other.CheckInvariants());

  // Our header file's inline functions only call into here when the addition
  // isn't a no-op.
  DRAKE_ASSERT(scale != 0.0);
  DRAKE_ASSERT(!other.empty());

  // Both `this` and `other` are of non-zero size. In that case, the sizes
  // must match. If this check trips, then that means the user tried to mix
  // AutoDiff partial derivatives of non-unform sizes.
  if ((size_ > 0) && (size_ != other.size_)) {
    throw std::logic_error(fmt::format(
        "The size of AutoDiff partial derivative vectors must be uniform"
        " throughout the computation, but two different sizes ({} and {})"
        " were encountered at runtime. The dervatives were {} and {}.",
        size_, other.size_, make_const_xpr().eval(),
        other.make_const_xpr().eval()));
  }

  // The implementation that follows has many special cases for performance.
  //
  // The easiest way to follow along is to recognize that the case-analysis
  // branches first on the representation of `this` from lowest complexity
  // to highest complexity:
  //   Case 0 -- `this` is a zero.
  //   Case 1 -- `this` is a scaled unit vector.
  //   Case 2 -- `this` is a scaled uniquely-owned heap vector.
  //   Case 3 -- `this` is a scaled shared-ownership heap vector.
  //
  // Then secondarily, we branch on the nature of `other`.

  // === Case 0 -- this is a zero. ===

  if (empty()) {
    // Borrow from `other` with no new allocations.
    *this = other;
    // N.B. Only _after_ copying can we apply the scale.
    coeff_ *= scale;
  }

  // === Case 1 -- this is a scaled unit vector. ===

  if (is_unit()) {
    // If `other` is the same unit vector, the addition is cheap.
    if (unit_ == other.unit_) {
      coeff_ += scale * other.coeff_;
      if (coeff_ == 0.0) {
        Clear();
      }
      return;
    }

    // Otherwise, we need to allocate storage for a non-unit vector.
    storage_ = CowVec::Allocate(size_);
    double* new_data = const_cast<double*>(storage_.data());

    // If this and other were both unit vectors, just set the two partials.
    if (other.is_unit()) {
      for (int i = 0; i < size_; ++i) {
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
    for (int i = 0; i < size_; ++i) {
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
    double* this_data = const_cast<double*>(storage_.data());

    // If `other` is a unit vector, the addition is cheap.
    if (other.is_unit()) {
      this_data[other.get_unit_index()] += scale * other.coeff_ / coeff_;
      return;
    }

    // Otherwise, we'll need to fold in the other's data.
    // We use a linear combination and end up with a new coeff of 1.0.
    const double* const other_data = other.storage_.data();
    for (int i = 0; i < size_; ++i) {
      this_data[i] = coeff_ * this_data[i]
          + scale * other.coeff_ * other_data[i];
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
    coeff_ += scale * other.coeff_;
    if (coeff_ == 0.0) {
      Clear();
    }
    return;
  }

  // We've run out of tricks; we have no choice now but to allocate fresh
  // storage to accumulate the result.
  CowVec new_storage = CowVec::Allocate(size_);
  double* new_this_data = const_cast<double*>(new_storage.data());
  const double* const original_this_data = storage_.data();

  // If `other` was a unit vector, we can copy our data and add the single unit.
  if (other.is_unit()) {
    for (int i = 0; i < size_; ++i) {
      new_this_data[i] = original_this_data[i];
    }
    new_this_data[other.get_unit_index()] = scale * other.coeff_ / coeff_;
    storage_ = std::move(new_storage);
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // We've reached the most general case. Both partials are distinct, readonly
  // heap vectors.
  Eigen::Map<Eigen::VectorXd> output_vector(new_this_data, size_);
  output_vector.noalias() =
      coeff_ * Eigen::Map<const Eigen::VectorXd>(storage_.data(), size_)
      + (scale * other.coeff_) *
            Eigen::Map<const Eigen::VectorXd>(other.storage_.data(), size_);
  storage_ = std::move(new_storage);
  coeff_ = 1.0;
  DRAKE_ASSERT_VOID(CheckInvariants());
}

AutoDiffDerivativesConstXpr Partials::make_const_xpr() const {
  const double* data = nullptr;
  if (size_ > 0) {
    if (unit_ > 0) {
      data = GetStaticUnitVector(unit_ - 1);
    } else {
      data = storage_.data();
    }
    DRAKE_ASSERT(data != nullptr);
  }
  return AutoDiffDerivativesConstXpr{coeff_, data, size_};
}

AutoDiffDerivativesMutableXpr Partials::MakeMutableXpr() {
  DRAKE_ASSERT_VOID(CheckInvariants());

  // If we have no data, then there's no preparation required.
  // If we do have data, then we might need to copy-on-write now.
  if (size_ == 0) {
    DRAKE_DEMAND(storage_.data() == nullptr);
  } else if ((unit_ == 0) && (coeff_ == 1.0) &&
             (storage_.get_use_count() == 1)) {
    // We don't need to copy it again; we already own the correct thing.
  } else {
    // The data is either not exclusively owned, or is not in canonical form.
    // We need to copy it now.
    // XXX is the Eigen::Ref making a (discared) copy here?
    *this = Partials(make_const_xpr());
    DRAKE_DEMAND(size_ > 0);
    DRAKE_DEMAND(storage_.data() != nullptr);
  }

  return AutoDiffDerivativesMutableXpr{this};
}

AutoDiffDerivativesMutableXpr Partials::SetFrom(
    const Eigen::Ref<const Eigen::VectorXd>& other) {
  *this = Partials(other);
  return MakeMutableXpr();
}

}  // namespace internal
}  // namespace drake
