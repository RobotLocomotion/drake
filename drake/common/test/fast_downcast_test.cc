#include "drake/common/fast_downcast.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace {

class Base {
 public:
  virtual ~Base() {}
  virtual int value() const = 0;
};

class DerivedFromBase : public Base {
 public:
  explicit DerivedFromBase(int a) : i_(a) {}
  int value() const final {
    return i_;
  }
 private:
  int i_;
};

class AnotherDerivedFromBase : public Base {
 public:
  explicit AnotherDerivedFromBase(int a) : i_(a) {}
  int value() const final {
    return 2 * i_;
  }
 private:
  int i_;
};

class NotDerivedFromBase {};

// Mostly, tests that we can compile.
GTEST_TEST(FastDownCastSucceeds, DowncastMutablePointer) {
  DerivedFromBase original(2);  // Instantiates a derived class.
  Base* base = &original;  // Somewhere in the process we get a Base pointer.
  // Attempt to retrieve the derived class as a pointer.
  DerivedFromBase* derived = fast_downcast<DerivedFromBase*>(base);
  EXPECT_EQ(derived->value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastConstReference) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a Base reference.
  const Base& base = original;
  // Attempt to retrieve the derived class as a reference.
  const DerivedFromBase& derived = fast_downcast<const DerivedFromBase&>(base);
  EXPECT_EQ(derived.value(), 2);
}

// The following tests fail to downcast.
#ifndef NDEBUG
GTEST_TEST(FastDownCastFails, DowncastMutablePointer) {
  // Instantiates a non-derived class.
  AnotherDerivedFromBase original(3);
  // Somewhere in the process we get a pointer to the wrong Base.
  Base* base = &original;
  // Attempt to retrieve the derived class as a pointer.
  EXPECT_EQ(fast_downcast<DerivedFromBase*>(base), nullptr);
}
#endif

}  // namespace
}  // namespace drake
