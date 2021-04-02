#include "drake/common/test_utilities/limit_malloc.h"

int main(void) {
  drake::test::LimitMalloc guard;
  new int;  // Deliberately cause LimitMalloc to report failure.
  return 0;
}
