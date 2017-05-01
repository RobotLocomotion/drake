#include "drake/common/test/is_memcpy_movable.h"

#include <utility>

#include <gtest/gtest.h>

namespace drake {
namespace test {
namespace {

using std::move;

// An example class that is not memcpy-movable.
class NotMemcpyMovable {
 public:
  NotMemcpyMovable() : self_{this} {};
  NotMemcpyMovable(const NotMemcpyMovable&) : self_{this} {};
  NotMemcpyMovable(NotMemcpyMovable&&) : self_{this} {}

  void operator=(const NotMemcpyMovable&) {}
  void operator=(NotMemcpyMovable&&) {}

  bool check_invariant() const { return this == self_; }

 private:
  NotMemcpyMovable* self_{};
};

struct InvariantsHold {
  // It checks if both of v1 and v2 satisfy invariants.
  bool operator()(const NotMemcpyMovable& v1,
                  const NotMemcpyMovable& v2) const {
    return v1.check_invariant() && v2.check_invariant();
  }
};

GTEST_TEST(IsMemcpyMovableTest, NotMemcpyMovableTest) {
  const NotMemcpyMovable v;
  EXPECT_FALSE(IsMemcpyMovable(v, InvariantsHold()));
  EXPECT_FALSE(IsMemcpyMovable(move(v), InvariantsHold()));
}

}  // namespace
}  // namespace test
}  // namespace drake
