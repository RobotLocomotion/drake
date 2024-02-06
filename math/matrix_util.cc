// For now, this is an empty .cc file that only serves to confirm
// matrix_util.h is a stand-alone header.

#include "drake/math/matrix_util.h"
namespace drake {
namespace math {

int SymmetricMatrixIndexToLowerTriangularLinearIndex(int i, int j, int n) {
  DRAKE_THROW_UNLESS(i < n && j < n && i >= j);
  // TODO(Alexandre.Amice) make this way more efficient.
  int ctr = 0;
  for (int r = 0; r < n; ++r) {
    for (int c = r; c < n; ++c) {
      if (r == i && c == j) {
        return ctr;
      }
      ctr++;
    }
  }
  DRAKE_UNREACHABLE();
}

std::pair<int, int> LowerTriangularLinearIndexToSymmetricMatrixIndex(int l,
                                                                     int n) {
  DRAKE_THROW_UNLESS(l < 0.5 * n * (n + 1));
  // TODO(Alexandre.Amice) make this way more efficient.
  int ctr = 0;
  for (int i = 0; i < n; ++i) {
    for (int j = i; j < n; ++j) {
      if (ctr == l) {
        return {i, j};
      }
      ctr++;
    }
  }
  DRAKE_UNREACHABLE();
}
}  // namespace math
}  // namespace drake