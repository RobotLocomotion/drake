#pragma once

#include <algorithm>
#include <array>
#include <tuple>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/is_less_than_comparable.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy): If we add hash_append, change the documentation below
//  to say it can be used with std::unordered_map and std::unordered_set.

/// This class is similar to the drake::SortedPair class. However, this class
/// uses a triplet of homogeneous types. Both SortedPair and SortedTriplet
/// sort the values such that one value is less than or equal to the next one.
/// The SortedTriplet class can be used to generate keys for std::map (or
/// std::set) from triplets of objects. However, it cannot be used to
/// generate keys for std::unordered_map (or std::unordered_set).
///
/// @tparam T A template type that provides `operator<` and supports default
///           construction.
template<class T>
struct SortedTriplet {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SortedTriplet)
  static_assert(is_less_than_comparable<T>::value,
                "SortedTriplet can only be used with types that can be "
                "compared using the less-than operator (operator<).");

  /// The default constructor creates the three objects using their
  /// respective default constructors.
  SortedTriplet() = default;

  // TODO(DamrongGuoy): Add rvalue reference constructor, permits constructing
  //  with std::unique_ptr types, for example.

  /// Constructs a %SortedTriplet from three objects.
  SortedTriplet(const T& a, const T& b, const T& c) : objects_{a, b, c} {
    std::stable_sort(objects_.begin(), objects_.end());
  }

  // TODO(DamrongGuoy): Add type-converting copy constructor.

  // TODO(DamrongGuoy): Add set(a,b,c) to reset the stored objects.

  /// Gets the first (according to `operator<`) of the objects.
  const T& first() const { return objects_[0]; }

  /// Gets the second (according to `operator<`) of the objects.
  const T& second() const { return objects_[1]; }

  /// Gets the third (according to `operator<`) of the objects.
  const T& third() const { return objects_[2]; }

  // TODO(DamrongGuoy): Add Swap(t) to swap `this` and `t`.

  // TODO(DamrongGuoy): Add hash_append(HashAlgorithm&, SortedTriplet&) to
  //  implement the hash_append concept.

 private:
  // The three objects in the order of T::operator<.
  std::array<T, 3> objects_{};
};

/// Two triplets of the same type are equal iff their members are equal after
/// sorting.
template<class T>
inline bool operator==(const SortedTriplet<T>& x, const SortedTriplet<T>& y) {
  return !(x < y) && !(y < x);
}

/// Compares two triplets using lexicographic ordering.
template<class T>
inline bool operator<(const SortedTriplet<T>& x, const SortedTriplet<T>& y) {
  return std::tie(x.first(), x.second(), x.third()) <
         std::tie(y.first(), y.second(), y.third());
}

// TODO(DamrongGuoy): Provide operators !=, >, <=, >=.

// TODO(DamrongGuoy): Provide MakeSortedTriplet() a convenience wrapper for
//  creating a sorted triplet from three objects.

}  // namespace internal
}  // namespace geometry
}  // namespace drake

// TODO(DamrongGuoy): Implement std::swap().

// TODO(DamrongGuoy): Implement std::hash<SortedTriplet<T>>.

