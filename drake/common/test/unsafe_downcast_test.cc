#include "drake/common/unsafe_downcast.h"

#include <gtest/gtest.h>

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
  DerivedFromBase* derived = unsafe_downcast<DerivedFromBase *>(base);
  EXPECT_EQ(derived->value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastConstPointer) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a const Base pointer.
  const Base* base = &original;
  // Attempts to retrieve the derived class as a pointer.
  const DerivedFromBase* derived =
      unsafe_downcast<const DerivedFromBase *>(base);
  EXPECT_EQ(derived->value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastMutableReference) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a mutable Base reference.
  Base& base = original;
  // Attempts to retrieve the derived class as a mutable reference.
  DerivedFromBase& derived = unsafe_downcast<DerivedFromBase &>(base);
  EXPECT_EQ(derived.value(), 2);
}

GTEST_TEST(FastDownCastSucceeds, DowncastConstReference) {
  // Instantiates a derived class.
  DerivedFromBase original(2);
  // Somewhere in the process we get a const Base reference.
  const Base& base = original;
  // Attempts to retrieve the derived class as a reference.
  const DerivedFromBase& derived =
      unsafe_downcast<const DerivedFromBase &>(base);
  EXPECT_EQ(derived.value(), 2);
}

// The following tests fail to downcast because the argument class type B is not
// a base class of the requested type D.
// If DRAKE_UNSAFE_DOWNCAST_IS_SAFE is defined, then unsafe_downcast will behave
// as a dynamic_cast and then we can check if the downcast was successful;
// As with dynamic_cast, expect unsafe_downcast<D*>(B*) (argument is a pointer)
// to return nullptr and expect unsafe_downcast<T&>(B&) (argument is a
// reference) to throw a std::bad_cast exception.
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
GTEST_TEST(FastDownCastFails, DowncastMutablePointer) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a pointer to the Base class.
  Base* base = &original;
  // Attempts to retrieve another type of derived class.
  EXPECT_EQ(unsafe_downcast<AnotherDerivedFromBase *>(base), nullptr);
}

GTEST_TEST(FastDownCastFails, DowncastConstPointer) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a const pointer to the Base class.
  const Base* base = &original;
  // Attempts to retrieve another type of derived class.
  EXPECT_EQ(unsafe_downcast<const AnotherDerivedFromBase *>(base), nullptr);
}
#endif

// Under unsuccessful downcasting of references, unsafe_downcast behaves as
// dynamic_cast (i.e. throws as std::bad_cast) if
// DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE defined, and it behaves as static_cast
// (no exception thrown) if DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE is not defined.
GTEST_TEST(FastDownCastFails, DowncastMutableReference) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a mutable reference to the Base class.
  Base& base = original;
  // Attempts to retrieve another type of derived class.
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
  EXPECT_THROW(unsafe_downcast<AnotherDerivedFromBase &>(base), std::bad_cast);
#else
  EXPECT_NO_THROW(unsafe_downcast<AnotherDerivedFromBase &>(base));
#endif
}

GTEST_TEST(FastDownCastFails, DowncastConstReference) {
  // Instantiates derived class.
  DerivedFromBase original(3);
  // Somewhere in the process we get a const reference to the Base class.
  const Base& base = original;
  // Attempts to retrieve another type of derived class.
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
  EXPECT_THROW(unsafe_downcast<const AnotherDerivedFromBase &>(base),
               std::bad_cast);
#else
  EXPECT_NO_THROW(unsafe_downcast<const AnotherDerivedFromBase &>(base));
#endif
}

}  // namespace
}  // namespace drake
