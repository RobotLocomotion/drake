#include "drake/common/test_utilities/limit_malloc.h"

int main(void) {
  drake::test::LimitMalloc guard;
  // Deliberately cause LimitMalloc to report failure. Yes, we leak memory
  // here, but it doesn't much matter; the material fact to the test is that a
  // disallowed heap allocation happens.
  new int;
  return 0;
}
