#include "drake/multibody/tree/scoped_name.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(ScopedNameTest, DefaultCtor) {
  const ScopedName dut;
  EXPECT_EQ(dut.get_namespace(), "");
  EXPECT_EQ(dut.get_element(), "");
  EXPECT_EQ(dut.get_full(), "");
  EXPECT_EQ(dut.to_string(), "");
}

GTEST_TEST(ScopedNameTest, PlainCtor) {
  const ScopedName dut("foo", "bar");
  EXPECT_EQ(dut.get_namespace(), "foo");
  EXPECT_EQ(dut.get_element(), "bar");
  EXPECT_EQ(dut.get_full(), "foo::bar");
  EXPECT_EQ(dut.to_string(), "foo::bar");
}

GTEST_TEST(ScopedNameTest, PlainCtorBad) {
  EXPECT_THROW(ScopedName("foo", ""), std::exception);
  EXPECT_THROW(ScopedName("foo", "::bar"), std::exception);
  EXPECT_THROW(ScopedName("::foo", "bar"), std::exception);
  EXPECT_THROW(ScopedName("foo::", "bar"), std::exception);
}

GTEST_TEST(ScopedNameTest, Make) {
  const std::optional<ScopedName> dut = ScopedName::Make("foo", "bar");
  ASSERT_TRUE(dut.has_value());
  EXPECT_EQ(dut->get_namespace(), "foo");
  EXPECT_EQ(dut->get_element(), "bar");
  EXPECT_EQ(dut->get_full(), "foo::bar");
  EXPECT_EQ(dut->to_string(), "foo::bar");
}

GTEST_TEST(ScopedNameTest, MakeBad) {
  EXPECT_EQ(ScopedName::Make("foo", ""), std::nullopt);
  EXPECT_EQ(ScopedName::Make("foo", "::bar"), std::nullopt);
  EXPECT_EQ(ScopedName::Make("::foo", "bar"), std::nullopt);
  EXPECT_EQ(ScopedName::Make("foo::", "bar"), std::nullopt);
}

GTEST_TEST(ScopedNameTest, Move) {
  ScopedName donor("foo", "bar");
  const ScopedName recipient(std::move(donor));

  EXPECT_EQ(donor.get_namespace(), "");
  EXPECT_EQ(donor.get_element(), "");
  EXPECT_EQ(donor.get_full(), "");
  EXPECT_EQ(donor.to_string(), "");

  EXPECT_EQ(recipient.get_namespace(), "foo");
  EXPECT_EQ(recipient.get_element(), "bar");
  EXPECT_EQ(recipient.get_full(), "foo::bar");
  EXPECT_EQ(recipient.to_string(), "foo::bar");
}

GTEST_TEST(ScopedNameTest, Join) {
  std::vector<std::array<std::string, 4>> tests{
      {"", "bar", "", "bar"},
      {"foo", "bar", "foo", "bar"},
      {"foo::quux", "bar", "foo::quux", "bar"},
      {"foo", "quux::bar", "foo::quux", "bar"},
      {"::foo::::", "::quux::bar::::", "foo::quux", "bar"},
  };
  for (const auto& [name1, name2, expected_namespace, expected_element] :
       tests) {
    SCOPED_TRACE(fmt::format("with names = ({}, {})", name1, name2));
    auto dut = ScopedName::Join(name1, name2);
    EXPECT_EQ(dut.get_namespace(), expected_namespace);
    EXPECT_EQ(dut.get_element(), expected_element);
  }
}

GTEST_TEST(ScopedNameTest, Parse) {
  std::vector<std::array<std::string, 3>> tests{
      {"bar", "", "bar"},
      {"foo::bar", "foo", "bar"},
      {"foo::quux::bar", "foo::quux", "bar"},
      {"::foo::::quux::::bar::", "foo::quux", "bar"},
  };
  for (const auto& [input, expected_namespace, expected_element] : tests) {
    SCOPED_TRACE(fmt::format("with input = {}", input));
    auto dut = ScopedName::Parse(input);
    EXPECT_EQ(dut.get_namespace(), expected_namespace);
    EXPECT_EQ(dut.get_element(), expected_element);
  }
}

GTEST_TEST(ScopedNameTest, Setters) {
  ScopedName dut("foo", "bar");
  EXPECT_EQ(dut.get_full(), "foo::bar");

  dut.set_namespace("");
  EXPECT_EQ(dut.get_full(), "bar");
  dut.set_namespace("quux");
  EXPECT_EQ(dut.get_full(), "quux::bar");

  dut.set_element("baz");
  EXPECT_EQ(dut.get_full(), "quux::baz");

  dut.set_namespace("foo::bar");
  EXPECT_EQ(dut.get_full(), "foo::bar::baz");
}

GTEST_TEST(ScopedNameTest, SettersBad) {
  ScopedName dut("foo", "bar");
  EXPECT_THROW(dut.set_namespace("::foo"), std::exception);
  EXPECT_THROW(dut.set_namespace("foo::"), std::exception);
  EXPECT_THROW(dut.set_element("::bar"), std::exception);
  EXPECT_THROW(dut.set_element(""), std::exception);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
