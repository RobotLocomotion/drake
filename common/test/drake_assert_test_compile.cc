/* clang-format off to disable clang-format-includes */
#include "drake/common/drake_assert.h"
/* clang-format on */

#include "drake/common/unused.h"

namespace {
struct NonBoolConvertible{};
void VoidFunction() {}
int NonVoidFunction() { return 0; }
}

int main(int argc, const char* argv[]) {
  DRAKE_ASSERT(true);
  DRAKE_ASSERT_VOID(VoidFunction());
  drake::unused(VoidFunction);

  // The build system toggles these guards.  The program should compile
  // and pass when undefined, and should fail to compile when defined.
#ifdef DRAKE_ASSERT_TEST_COMPILE_ERROR1
  DRAKE_ASSERT(NonBoolConvertible());
#endif

#ifdef DRAKE_ASSERT_TEST_COMPILE_ERROR2
  DRAKE_ASSERT_VOID(NonVoidFunction());
#endif
  drake::unused(NonVoidFunction);

  return 0;
}
