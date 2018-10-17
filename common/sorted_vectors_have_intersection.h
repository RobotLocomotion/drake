#pragma once

#include <algorithm>
#include <type_traits>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {

/** Checks for the existence of a non-empty intersection between two sorted
`std::vector`'s.

@param[in]  a  First vector.
@param[in]  b  Second vector.
@tparam T The type of the elements in the input vectors @p a and @p b. This is
expected to be an integral type or a pointer type.
@returns `true` if there is a non-empty intersection between vectors;
`false` otherwise.

Elements are compared using `operator<` and the vectors must be sorted with
respect to the same operator.

This algorithm only works on `std::vector`'s to take advantage of their fast
and random access.

Entries can be repeated as long as they are sorted. For vector `a` of size `Na`
and vector `b` of size `Nb` the complexity is at most Order(Na + Nb).
The algorithm executes in constant time for vectors with disjoint entries.
An example of the worst case scenario is given below:
@verbatim
 a = (10, 20, 30)
 b = (15, 25, 35)
@endverbatim

In this case the algorithm needs to scan both vectors from start to end. */
template <typename T>
bool SortedVectorsHaveIntersection(const std::vector<T>& a,
                                   const std::vector<T>& b) {
  // Asserts that T is either an integral type or a pointer type.
  static_assert(std::is_integral<T>::value || std::is_pointer<T>::value,
                "Input vectors must hold integral types or pointers.");

  // Checks the precondition that the lists are sorted, only in debug builds.
  // This means O(n) performance in Debug builds but it could also catch some
  // very obscure errors.
  DRAKE_ASSERT(std::is_sorted(a.cbegin(), a.cend()) &&
               std::is_sorted(b.cbegin(), b.cend()));

  typename std::vector<T>::const_iterator ai = a.begin();
  typename std::vector<T>::const_iterator bi = b.begin();

  // Quick checks first:
  // Check if either of the ranges is empty:
  if (ai == a.end() || bi == b.end()) return false;

  // Check for non-overlapping ranges:
  if (a.front() > b.back() || b.front() > a.back()) return false;

  // Non-empty ranges with elements that overlap.
  while (true) {
    // Increments the range with the smaller first element.
    if (*ai < *bi) {
      if (++ai == a.end()) break;
    } else if (*bi < *ai) {
      if (++bi == b.end()) break;
    } else {
      // Found matching element.
      return true;
    }
  }
  // One of the ranges ran out of elements before a match was found.
  return false;
}

}  // namespace drake
