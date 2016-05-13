#pragma once

namespace drake {
namespace common {
namespace algorithms {

/**
 *  @brief Checks for non-empty intersection of two sorted ranges.
 *
 *  @param  first1  Start of first range.
 *  @param  last1   End of first range.
 *  @param  first2  Start of second range.
 *  @param  last2   End of second range.
 *  @return `true` if non-empty intersection between ranges. `false` otherwise.
 *
 *  Elements are compared using operator< and the ranges must be sorted with
 *  respect to the same operator.
 *
 *  This algorithm is based upon std::set_intersection. However, instead of
 *  computing the intersection, which is a more expensive operation, this
 *  algorithm returns as soon as a common element is found and the range of
 *  common elements is not constructed.
 *
 *
 *
 *  This algorithm only works on sorted ranges.
 *  For range 1 of size N and range 2 of size M the worst case is Order(N+M).
 *  An example of worst case is given below:
 *  @verbatim
    range_1 = (10, 20, 30)
    range_2 = (15, 25, 35)
    @endverbatim
 *
 *
 *  In this case the algorithm needs to scan both arrays from start to end.
 */
template<class InputIterator1, class InputIterator2>
bool have_intersection(InputIterator1 first1, InputIterator1 last1,
                       InputIterator2 first2, InputIterator2 last2) {
  while (first1 != last1 && first2 != last2) {
    if (*first1 < *first2)
      ++first1;
    else if (*first2 < *first1)
      ++first2;
    else
      return true;
  }
  return false;
}

}  // namespace algorithms
}  // namespace common
}  // namespace drake
