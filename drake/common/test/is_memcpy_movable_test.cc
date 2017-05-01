#include "drake/common/test/is_memcpy_movable.h"

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

namespace drake {
namespace test {
namespace {

using std::make_shared;
using std::move;
using std::shared_ptr;
using std::string;

// An example class that is memcpy-movable.
class MemcpyMovable {
 public:
  MemcpyMovable(int id, string name)
      : id_{id}, name_{make_shared<string>(move(name))} {}
  MemcpyMovable(const MemcpyMovable& m) : MemcpyMovable{m.id_, *m.name_} {}
  MemcpyMovable(MemcpyMovable&& m) = default;
  bool operator==(const MemcpyMovable& m) const {
    return id_ == m.id_ && *name_ == *m.name_;
  }
  ~MemcpyMovable() = default;

  struct EqualTo {
    bool operator()(const MemcpyMovable& v1, const MemcpyMovable& v2) const {
      return v1 == v2;
    }
  };

 private:
  const int id_{};
  const shared_ptr<string> name_;
};

// An example class that is not memcpy-movable.
class NotMemcpyMovable {
 public:
  NotMemcpyMovable() : self_{this} {};
  NotMemcpyMovable(const NotMemcpyMovable&) : self_{this} {};
  NotMemcpyMovable(NotMemcpyMovable&&) : self_{this} {}

  void operator=(const NotMemcpyMovable&) {}
  void operator=(NotMemcpyMovable&&) {}

  bool check_invariant() const { return this == self_; }

  // It checks if both of v1 and v2 satisfy the invariant.
  struct InvariantsHold {
    bool operator()(const NotMemcpyMovable& v1,
                    const NotMemcpyMovable& v2) const {
      return v1.check_invariant() && v2.check_invariant();
    }
  };

 private:
  NotMemcpyMovable* self_{};
};

GTEST_TEST(IsMemcpyMovableTest, MemcpyMovableTestInt) {
  const int v{3};
  EXPECT_TRUE(IsMemcpyMovable(v));
}

GTEST_TEST(IsMemcpyMovableTest, MemcpyMovableTestDouble) {
  const double v{3.14};
  EXPECT_TRUE(IsMemcpyMovable(v));
}

GTEST_TEST(IsMemcpyMovableTest, MemcpyMovableTestStringPtr) {
  const string* const v{new string{"3.14"}};
  EXPECT_TRUE(IsMemcpyMovable(v));
  delete v;
}

GTEST_TEST(IsMemcpyMovableTest, MemcpyMovableTestClass) {
  const MemcpyMovable v{5, "MemcpyMovable"};
  EXPECT_TRUE(IsMemcpyMovable(v, MemcpyMovable::EqualTo()));
  EXPECT_TRUE(IsMemcpyMovable(move(v), MemcpyMovable::EqualTo()));
}

GTEST_TEST(IsMemcpyMovableTest, NotMemcpyMovableTest) {
  const NotMemcpyMovable v{};
  EXPECT_FALSE(IsMemcpyMovable(v, NotMemcpyMovable::InvariantsHold()));
  EXPECT_FALSE(IsMemcpyMovable(move(v), NotMemcpyMovable::InvariantsHold()));
}

}  // namespace
}  // namespace test
}  // namespace drake
