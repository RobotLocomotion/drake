#include "drake/math/binomial_coefficient.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace math {

int BinomialCoefficient(int n, int k) {
  // Adapted from https://stackoverflow.com/a/55422097/9510020.
  DRAKE_DEMAND(k >= 0);

  if (k > n) {
    return 0;
  }
  if (k == 0) {
    return 1;
  }

  int next = n - k + 1;
  int current;
  for (int i = 1; i < k; ++i) {
    current = next;
    next = current * (n - k + 1 + i) / (i + 1);
  }

  return next;
}

}  // namespace math
}  // namespace drake
