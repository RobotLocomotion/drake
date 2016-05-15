#pragma once

namespace drake {

/** @brief Checks for non-empty intersection of two sorted ranges.

@param  first1  Start of first range.
@param  last1   End of first range.
@param  first2  Start of second range.
@param  last2   End of second range.
@return `true` if non-empty intersection between ranges. `false` otherwise.

Elements are compared using operator< and the ranges must be sorted with
respect to the same operator.

This algorithm is based upon `std::set_intersection`. However, instead of
constructing the intersection this returns a yes/no result, which can be done
much faster.
Unlike the STL implementation, this algorithm does not offer the user the
possibility to provide a custom binary comparison function.
There is no execution policy as with the STL `std::set_intersection` to be
introduced in C++17.

This algorithm only works on sorted ranges.
For range 1 of size N and range 2 of size M the complexity is at most
Order(N+M). The algorithm executes in constant time for disjoint ranges.
An example of worst case is given below:
@verbatim
 range_1 = (10, 20, 30)
 range_2 = (15, 25, 35)
@endverbatim

In this case the algorithm needs to scan both arrays from start to end. **/
template<class InputIterator1, class InputIterator2>
bool HaveIntersection(InputIterator1 first1, InputIterator1 last1,
                      InputIterator2 first2, InputIterator2 last2) {
  // Quick checks first:
  // Check if any of the ranges is empty:
  if (first1 == last1 || first2 == last2) return false;

  // Check for non-overlapping ranges:
  if (*first1 > *(--InputIterator2(last2)) || *first2 > *(--InputIterator1(last1))) return false;

  // Non-empty ranges with elements that overlap.
  while (true) {
    // Increments the range with the smaller first element.
    if (*first1 < *first2) {
      ++first1;
      if (first1 == last1) break;
    } else if (*first2 < *first1) {
      ++first2;
      if (first2 == last2) break;
    } else {
      // Found matching element.
      return true;
    }
  }
  // One of the ranges run out of elements before a match was found.
  return false;
}

}  // namespace drake
