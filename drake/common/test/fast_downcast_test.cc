#include "drake/common/fast_downcast.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace {

// An abstract class used for testing.
class Base {
 public:
  virtual ~Base() {}
  virtual int value() const = 0;
};

// A derived class from Base.
class DerivedFromBase : public Base {
 public:
  explicit DerivedFromBase(int a) : i_(a) {}
  int value() const final {
    return i_;
  }
 private:
  int i_;
};

// Another derived class from Base.
class AnotherDerivedFromBase : public Base {
 public:
  explicit AnotherDerivedFromBase(int a) : i_(a) {}
  int value() const final {
    return 2 * i_;
  }
 private:
  int i_;
};

// Mostly, tests that we can compile.
GTEST_TEST(FastDownCastSucceeds, DowncastMutablePointer) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a mutable Base pointer.
  Base* base = &original;
  // Attempts to retrieve the derived class as a pointer.
  DerivedFromBase* derived = fast_downcast<DerivedFromBase*>(base);
  EXPECT_EQ(derived->value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastConstPointer) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a const Base pointer.
  const Base* base = &original;
  // Attempts to retrieve the derived class as a pointer.
  const DerivedFromBase* derived = fast_downcast<const DerivedFromBase*>(base);
  EXPECT_EQ(derived->value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastMutableReference) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a mutable Base reference.
  Base& base = original;
  // Attempts to retrieve the derived class as a mutable reference.
  DerivedFromBase& derived = fast_downcast<DerivedFromBase&>(base);
  EXPECT_EQ(derived.value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastConstReference) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a const Base reference.
  const Base& base = original;
  // Attempts to retrieve the derived class as a reference.
  const DerivedFromBase& derived = fast_downcast<const DerivedFromBase&>(base);
  EXPECT_EQ(derived.value(), 2);
}

// The following tests fail to downcast because the argument class type B is not
// a base class of the requested type D.
// As with dynamic_cast, expect fast_downcast<D*>(B*) (argument is a pointer)
// to return nullptr and expect fast_downcast<T&>(B&) (argument is a reference)
// to throw a std::bad_cast exception.
#ifndef NDEBUG
GTEST_TEST(FastDownCastFails, DowncastMutablePointer) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a pointer to the Base class.
  Base* base = &original;
  // Attempts to retrieve another type of derived class.
  EXPECT_EQ(fast_downcast<AnotherDerivedFromBase*>(base), nullptr);
}

GTEST_TEST(FastDownCastFails, DowncastConstPointer) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a const pointer to the Base class.
  const Base* base = &original;
  // Attempts to retrieve another type of derived class.
  EXPECT_EQ(fast_downcast<const AnotherDerivedFromBase*>(base), nullptr);
}

GTEST_TEST(FastDownCastFails, DowncastMutableReference) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a mutable reference to the Base class.
  Base& base = original;
  // Attempts to retrieve another type of derived class.
  EXPECT_THROW(fast_downcast<AnotherDerivedFromBase&>(base), std::bad_cast);
}

GTEST_TEST(FastDownCastFails, DowncastConstReference) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a const reference to the Base class.
  const Base& base = original;
  // Attempts to retrieve another type of derived class.
  EXPECT_THROW(fast_downcast<const AnotherDerivedFromBase&>(base),
      std::bad_cast);
}
#endif

}  // namespace
}  // namespace drake
