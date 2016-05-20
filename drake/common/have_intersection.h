#pragma once

#include <vector>

namespace drake {

/** Checks for non-empty intersection of two sorted `std::vector`'s.

@param  a  First vector.
@param  b  Second vector.
@return `true` if non-empty intersection between vectors. `false` otherwise.

Elements are compared using operator< and the vectors must be sorted with
respect to the same operator.

This algorithm only works on std::vector's to take advantage of their fast and
random access.

This algorithm only works on sorted vectors. Entries can be repeated as long as
they are sorted.
For vector a of size Na and vector b of size Nb the complexity is at most
Order(Na+Nb). The algorithm executes in constant time for vectors with disjoint
entries.
An example of worst case is given below:
@verbatim
 a = (10, 20, 30)
 b = (15, 25, 35)
@endverbatim

In this case the algorithm needs to scan both vectors from start to end. **/
template<typename T>
bool SortedVectorsHaveIntersection(const std::vector<T> &a,
                                   const std::vector<T> &b) {
  typename std::vector<T>::const_iterator ai = a.begin();
  typename std::vector<T>::const_iterator bi = b.begin();
  // Quick checks first:
  // Check if either of the ranges is empty:
  if (ai == a.end() || bi == b.end()) return false;

  // Check for non-overlapping ranges:
  if (a.front() > b.back() ||
      b.front() > a.back()) return false;

  // Non-empty ranges with elements that overlap.
  while (true) {
    // Increments the range with the smaller first element.
    if (*ai < *bi) {
      ++ai;
      if (ai == a.end()) break;
    } else if (*bi < *ai) {
      ++bi;
      if (bi == b.end()) break;
    } else {
      // Found matching element.
      return true;
    }
  }
  // One of the ranges run out of elements before a match was found.
  return false;
}

}  // namespace drake
