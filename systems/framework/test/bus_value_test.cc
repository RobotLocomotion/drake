#include "drake/systems/framework/bus_value.h"

#include <algorithm>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace systems {
namespace {

// N.B. In several of the tests below, we assume a specific iteration order to
// make the tests simpler to write and understand, even though the API does not
// promise one. If the implementation changes, we'll need to revisit the tests.

std::optional<std::string> AsString(const AbstractValue* abstract) {
  if (abstract == nullptr) {
    return std::nullopt;
  }
  return abstract->template get_value<std::string>();
}

std::string AsString(const AbstractValue& abstract) {
  return abstract.template get_value<std::string>();
}

GTEST_TEST(BusValueTest, Empty) {
  const BusValue dut;
  EXPECT_EQ(AsString(dut.Find("foo")), std::nullopt);
  EXPECT_EQ(dut.begin(), dut.end());

  BusValue dut_mutable;
  EXPECT_NO_THROW(dut_mutable.Clear());
}

GTEST_TEST(BusValueTest, Operations) {
  BusValue dut;

  dut.Set("foo", Value<std::string>("foo_value"));
  dut.Set("bar", Value<std::string>("bar_value"));
  EXPECT_EQ(AsString(dut.Find("foo")), "foo_value");
  EXPECT_EQ(AsString(dut.Find("bar")), "bar_value");
  EXPECT_EQ(AsString(dut.Find("quux")), std::nullopt);
  int i = 0;
  for (const auto&& [name, abstract] : dut) {
    const std::string expected = (i == 0) ? "foo" : "bar";
    EXPECT_EQ(name, expected);
    EXPECT_EQ(abstract.template get_value<std::string>(), expected + "_value");
    ++i;
  }
  EXPECT_EQ(i, 2);

  dut.Clear();
  EXPECT_EQ(AsString(dut.Find("foo")), std::nullopt);
  EXPECT_EQ(AsString(dut.Find("bar")), std::nullopt);
  EXPECT_EQ(dut.begin(), dut.end());

  dut.Set("bar", Value<std::string>("bar_value"));
  dut.Set("quux", Value<std::string>("quux_value"));
  EXPECT_EQ(AsString(dut.Find("foo")), std::nullopt);
  EXPECT_EQ(AsString(dut.Find("bar")), "bar_value");
  EXPECT_EQ(AsString(dut.Find("quux")), "quux_value");
  i = 0;
  for (const auto&& [name, abstract] : dut) {
    EXPECT_LT(i, 2);
    const std::string expected = (i == 0) ? "bar" : "quux";
    EXPECT_EQ(name, expected);
    EXPECT_EQ(AsString(abstract), expected + "_value");
    ++i;
  }

  dut.Clear();
  EXPECT_EQ(AsString(dut.Find("foo")), std::nullopt);
  EXPECT_EQ(AsString(dut.Find("bar")), std::nullopt);
  EXPECT_EQ(AsString(dut.Find("quux")), std::nullopt);
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
  EXPECT_EQ(AsString(copy.Find("foo")), "foo_value");
  EXPECT_EQ(AsString(copy.Find("bar")), "bar_value");

  BusValue empty(dut);
  EXPECT_EQ(AsString(empty.Find("foo")), std::nullopt);
  EXPECT_EQ(AsString(empty.Find("bar")), std::nullopt);
  EXPECT_EQ(empty.begin(), empty.end());
}

GTEST_TEST(BusValueTest, CopyMoveAssignExist) {
  // Given test coverage already provided by the Copy test immediately above,
  // we'll rely on how copyable_unique_ptr works to conclude that the only extra
  // testing we need for the other standard methods are to make sure they exist
  // at all.

  BusValue dut;
  const BusValue ctor_copy{dut};
  const BusValue ctor_move{std::move(dut)};
  BusValue ctor_copy_assign;
  ctor_copy_assign = dut;
  BusValue ctor_move_assign;
  ctor_move_assign = std::move(dut);
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

// This test demonstrates our current behavior for mismatched types. In the
// future it would be okay to soften what's here, but at least this shows us
// the starting point and will help us notice when things change.
GTEST_TEST(BusValueTest, MismatchedTypes) {
  const Value<int> indigo{22};
  const Value<std::string> sierra{"sierra"};

  // The current type of "x" will be "int".
  BusValue dut;
  dut.Set("x", indigo);

  // Even after clearing, the presumed type remains.
  dut.Clear();
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Set("x", sierra), ".*cast.*int.*string.*");

  // A default-constructed object is empty (and so, resets the presumed type).
  dut = BusValue{};
  EXPECT_NO_THROW(dut.Set("x", sierra));

  // Start fresh, set the presumed type to "int" again.
  dut = BusValue{};
  dut.Set("x", indigo);
  dut.Clear();

  // The presumed type follows the move; the donor dut is truly empty now.
  BusValue moved_to{std::move(dut)};
  EXPECT_NO_THROW(dut.Set("x", sierra));
  DRAKE_EXPECT_THROWS_MESSAGE(moved_to.Set("x", sierra),
                              ".*cast.*int.*string.*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
