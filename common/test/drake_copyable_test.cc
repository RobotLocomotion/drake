#include "drake/common/drake_copyable.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

// When a class has a user-defined destructor, the implicit generation of the
// copy constructor and copy-assignment operator is deprecated as of C++11.
//
// Under Drake's Clang configuration, relying on the deprecated operator
// produces an error under -Werror=deprecated.
//
// Therefore, if DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN were failed to default
// those operations, then this unit test would no longer compile.
class Example {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Example);
  Example() = default;
  ~Example() {}
};

GTEST_TEST(DrakeCopyableTest, CopyConstruct) {
  Example foo;
  Example bar(foo);
}

GTEST_TEST(DrakeCopyableTest, CopyAssign) {
  Example foo, bar;
  bar = foo;
}

}  // namespace
}  // namespace drake
