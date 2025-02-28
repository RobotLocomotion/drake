#include "drake/systems/framework/bus_value.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

GTEST_TEST(BusValueTest, Empty) {
  const BusValue dut;
  EXPECT_EQ(dut.Find("foo"), nullptr);
  EXPECT_EQ(dut.begin(), dut.end());
}

GTEST_TEST(BusValueTest, Operations) {
  BusValue dut;

  dut.Set("foo", Value<std::string>("foo_value"));
  dut.Set("bar", Value<std::string>("bar_value"));
  EXPECT_EQ(dut.Find("foo")->template get_value<std::string>(), "foo_value");
  EXPECT_EQ(dut.Find("bar")->template get_value<std::string>(), "bar_value");
  EXPECT_EQ(dut.Find("quux"), nullptr);
  int i = 0;
  for (const auto&& [name, abstract] : dut) {
    EXPECT_LT(i, 2);
    const std::string expected = (i == 0) ? "foo" : "bar";
    EXPECT_EQ(name, expected);
    EXPECT_EQ(abstract.template get_value<std::string>(), expected + "_value");
    ++i;
  }

  dut.Clear();
  EXPECT_EQ(dut.Find("foo"), nullptr);
  EXPECT_EQ(dut.Find("bar"), nullptr);
  EXPECT_EQ(dut.begin(), dut.end());

  dut.Set("bar", Value<std::string>("bar_value"));
  dut.Set("quux", Value<std::string>("quux_value"));
  EXPECT_EQ(dut.Find("foo"), nullptr);
  EXPECT_EQ(dut.Find("bar")->template get_value<std::string>(), "bar_value");
  EXPECT_EQ(dut.Find("quux")->template get_value<std::string>(), "quux_value");
  i = 0;
  for (const auto&& [name, abstract] : dut) {
    EXPECT_LT(i, 2);
    const std::string expected = (i == 0) ? "bar" : "quux";
    EXPECT_EQ(name, expected);
    EXPECT_EQ(abstract.template get_value<std::string>(), expected + "_value");
    ++i;
  }

  dut.Clear();
  EXPECT_EQ(dut.Find("foo"), nullptr);
  EXPECT_EQ(dut.Find("bar"), nullptr);
  EXPECT_EQ(dut.Find("quux"), nullptr);
  EXPECT_EQ(dut.begin(), dut.end());
}

GTEST_TEST(BusValueTest, DispreferredIterator) {
  BusValue dut;
  dut.Set("foo", Value<std::string>("foo_value"));
  dut.Set("bar", Value<std::string>("bar_value"));
  auto iter = dut.begin();
  iter++;  // GSG says not to do this, but we need to test this overload.
  EXPECT_EQ((*iter).first, "bar");
}

GTEST_TEST(BusValueTest, Copy) {
  BusValue dut;
  dut.Set("foo", Value<std::string>("foo_value"));
  dut.Set("bar", Value<std::string>("bar_value"));

  BusValue copy(dut);
  dut.Clear();
  EXPECT_EQ(copy.Find("foo")->template get_value<std::string>(), "foo_value");
  EXPECT_EQ(copy.Find("bar")->template get_value<std::string>(), "bar_value");

  BusValue empty(dut);
  EXPECT_EQ(empty.Find("foo"), nullptr);
  EXPECT_EQ(empty.Find("bar"), nullptr);
  EXPECT_EQ(empty.begin(), empty.end());
}

GTEST_TEST(BusValueTest, CopyMoveAssignExist) {
  // Given test coverage already provided by the Copy test immediately above,
  // we'll rely our how copyable_unique_ptr works to conclude that the only
  // extra testing we need for the other standard methods are to make sure they
  // exist at all.

  BusValue dut;
  const BusValue ctor_copy{dut};
  const BusValue ctor_move{std::move(dut)};
  BusValue ctor_copy_assign;
  ctor_copy_assign = dut;
  BusValue ctor_move_assign;
  ctor_move_assign = std::move(dut);
}

std::optional<std::string> AsString(const AbstractValue* abstract) {
  if (abstract == nullptr) {
    return std::nullopt;
  }
  return abstract->template get_value<std::string>();
}

std::string AsString(const AbstractValue& abstract) {
  return abstract.template get_value<std::string>();
}

GTEST_TEST(BusValueTest, Reindexing) {
  BusValue dut;

  // Trigger the capacity overflow condition inside Set().
  for (int i = 0; i < 100; ++i) {
    const std::string x = std::to_string(i);
    dut.Set(x, Value<std::string>(x + "_value"));
  }

  // Check that everything survived.
  auto iter = dut.begin();
  for (int i = 0; i < 100; ++i) {
    SCOPED_TRACE(fmt::format("i = {}", i));
    EXPECT_NE(iter, dut.end());
    const auto& [name, abstract] = *iter;
    const std::string x = std::to_string(i);
    EXPECT_EQ(name, x);
    EXPECT_EQ(AsString(abstract), x + "_value");
    EXPECT_EQ(AsString(dut.Find(x)), x + "_value");
    ++iter;
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
