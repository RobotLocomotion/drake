#include "drake/common/pointer_cast.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace {

struct Base {
  virtual ~Base() = default;
};

struct Derived : public Base {};

struct MoreDerived : public Derived {};

// === Tests for static_pointer_cast<> ===

// N.B. We can't test failed downcast (types mismatch), because that is
// undefined behavior.

// Downcasting works.
GTEST_TEST(StaticPointerCastTest, Downcast) {
  std::unique_ptr<Base> u = std::make_unique<Derived>();
  std::unique_ptr<Derived> t = static_pointer_cast<Derived>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Casting to the identical type works.
GTEST_TEST(StaticPointerCastTest, SameType) {
  std::unique_ptr<Base> u = std::make_unique<Base>();
  std::unique_ptr<Base> t = static_pointer_cast<Base>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Upcasting works.
GTEST_TEST(StaticPointerCastTest, Upcast) {
  std::unique_ptr<Derived> u = std::make_unique<Derived>();
  std::unique_ptr<Base> t = static_pointer_cast<Base>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Downcasting while preserving const works.
GTEST_TEST(StaticPointerCastTest, DowncastConst) {
  std::unique_ptr<const Base> u = std::make_unique<const Derived>();
  std::unique_ptr<const Derived> t =
      static_pointer_cast<const Derived>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// === Tests for dynamic_pointer_cast<> ===

// Downcasting works.
GTEST_TEST(DynamicPointerCastTest, Downcast) {
  std::unique_ptr<Base> u = std::make_unique<Derived>();
  std::unique_ptr<Derived> t = dynamic_pointer_cast<Derived>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Failed downcast (type mismatch) leaves the original object intact.
GTEST_TEST(DynamicPointerCastTest, FailedDowncast) {
  std::unique_ptr<Base> u = std::make_unique<Derived>();
  std::unique_ptr<Derived> t = dynamic_pointer_cast<MoreDerived>(std::move(u));
  EXPECT_EQ(!!u, true);
  EXPECT_EQ(!!t, false);
}

// Casting to the identical type works.
GTEST_TEST(DynamicPointerCastTest, SameType) {
  std::unique_ptr<Base> u = std::make_unique<Base>();
  std::unique_ptr<Base> t = dynamic_pointer_cast<Base>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Upcasting works.
GTEST_TEST(DynamicPointerCastTest, Upcast) {
  std::unique_ptr<Derived> u = std::make_unique<Derived>();
  std::unique_ptr<Base> t = dynamic_pointer_cast<Base>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Downcasting while preserving const works.
GTEST_TEST(DynamicPointerCastTest, DowncastConst) {
  std::unique_ptr<const Base> u = std::make_unique<const Derived>();
  std::unique_ptr<const Derived> t =
      dynamic_pointer_cast<const Derived>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// === Tests for dynamic_pointer_cast_or_throw<> on unique_ptr ===

// Downcasting works.
GTEST_TEST(DynamicPointerCastOrThrowTest, Downcast) {
  std::unique_ptr<Base> u = std::make_unique<Derived>();
  std::unique_ptr<Derived> t =
      dynamic_pointer_cast_or_throw<Derived>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Failed cast of nullptr.
GTEST_TEST(DynamicPointerCastOrThrowTest, FailedCastFromNullptr) {
  std::unique_ptr<Base> u;
  DRAKE_EXPECT_THROWS_MESSAGE(
      dynamic_pointer_cast_or_throw<MoreDerived>(std::move(u)),
      "Cannot cast a unique_ptr<drake.*Base> containing nullptr "
      "to unique_ptr<drake.*MoreDerived>.");
  EXPECT_EQ(!!u, false);
}

// Failed cast (type mismatch) leaves the original object intact.
GTEST_TEST(DynamicPointerCastOrThrowTest, FailedDowncast) {
  std::unique_ptr<Base> u = std::make_unique<Derived>();
  DRAKE_EXPECT_THROWS_MESSAGE(
      dynamic_pointer_cast_or_throw<MoreDerived>(std::move(u)),
      "Cannot cast a unique_ptr<drake.*Base> containing an object of type "
      "drake.*Derived to unique_ptr<drake.*MoreDerived>.");
  EXPECT_EQ(!!u, true);
}

// Casting to the identical type works.
GTEST_TEST(DynamicPointerCastOrThrowTest, SameType) {
  std::unique_ptr<Base> u = std::make_unique<Base>();
  std::unique_ptr<Base> t = dynamic_pointer_cast_or_throw<Base>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Upcasting works.
GTEST_TEST(DynamicPointerCastOrThrowTest, Upcast) {
  std::unique_ptr<Derived> u = std::make_unique<Derived>();
  std::unique_ptr<Base> t = dynamic_pointer_cast_or_throw<Base>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// Downcasting while preserving const works.
GTEST_TEST(DynamicPointerCastOrThrowTest, DowncastConst) {
  std::unique_ptr<const Base> u = std::make_unique<const Derived>();
  std::unique_ptr<const Derived> t =
      dynamic_pointer_cast_or_throw<const Derived>(std::move(u));
  EXPECT_EQ(!!u, false);
  EXPECT_EQ(!!t, true);
}

// === Tests for dynamic_pointer_cast_or_throw<> on a raw pointer ===

// Downcasting works.
GTEST_TEST(BareDynamicPointerCastOrThrowTest, Downcast) {
  Derived derived;
  Base* u = &derived;
  Derived* t = dynamic_pointer_cast_or_throw<Derived>(u);
  EXPECT_TRUE(t != nullptr);
  EXPECT_TRUE(t == u);
}

// Failed cast of nullptr.
GTEST_TEST(BareDynamicPointerCastOrThrowTest, FailedCastFromNullptr) {
  Base* u = nullptr;
  DRAKE_EXPECT_THROWS_MESSAGE(dynamic_pointer_cast_or_throw<MoreDerived>(u),
                              "Cannot cast a nullptr drake.*Base. "
                              "to drake.*MoreDerived..");
}

// Failed cast (type mismatch).
GTEST_TEST(BareDynamicPointerCastOrThrowTest, FailedDowncast) {
  Derived derived;
  Base* u = &derived;
  DRAKE_EXPECT_THROWS_MESSAGE(
      dynamic_pointer_cast_or_throw<MoreDerived>(u),
      "Cannot cast a drake.*Base. pointing to an object of type "
      "drake.*Derived to drake.*MoreDerived..");
}

}  // namespace
}  // namespace drake
