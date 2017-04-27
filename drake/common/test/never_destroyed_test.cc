#include "drake/common/never_destroyed.h"

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace {

namespace {
class Boom : public std::exception { };
struct DtorGoesBoom {
  ~DtorGoesBoom() noexcept(false) { throw Boom(); }
};
}

// Confirm that we see booms by default.
GTEST_TEST(NeverDestroyedTest, BoomTest) {
  try {
    { DtorGoesBoom foo; }
    GTEST_FAIL();
  } catch (const Boom&) {
    ASSERT_TRUE(true);
  }
}

// Confirm that our wrapper stops the booms.
GTEST_TEST(NeverDestroyedTest, NoBoomTest) {
  try {
    { never_destroyed<DtorGoesBoom> foo; }
    ASSERT_TRUE(true);
  } catch (const Boom& e) {
    GTEST_FAIL();
  }
}

}  // namespace
}  // namespace drake
