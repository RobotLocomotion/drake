#include "drake/common/drake_assert.h"

namespace { struct NonBoolConvertible{}; }

int main(int argc, const char* argv[]) {
// The build system toggles this guard.  The program should compile
// and pass when undefined, and should fail to compile when defined.
#ifdef DRAKE_ASSERT_TEST_COMPILE_ERROR
  DRAKE_ASSERT(NonBoolConvertible());
#endif

  return 0;
}
