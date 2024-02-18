#include <gtest/gtest.h>

#include "drake/common/string_map.h"
#include "drake/common/string_set.h"
#include "drake/common/string_unordered_map.h"
#include "drake/common/string_unordered_set.h"

namespace drake {
namespace {

GTEST_TEST(StringContainerTest, string_set) {
  string_set dut;
  dut.insert("hello");
  dut.insert("hello");
  const int count = 1;
  EXPECT_EQ(dut.count("hello"), count);
}

GTEST_TEST(StringContainerTest, string_multiset) {
  string_multiset dut;
  dut.insert("hello");
  dut.insert("hello");
  const int count = 2;
  EXPECT_EQ(dut.count("hello"), count);
}

GTEST_TEST(StringContainerTest, string_unordered_set) {
  string_unordered_set dut;
  dut.insert("hello");
  dut.insert("hello");
  const int count = 1;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
  // Demonstrate that the container is unordered.
  EXPECT_GT(dut.bucket_count(), 0);
}

GTEST_TEST(StringContainerTest, string_unordered_multiset) {
  string_unordered_multiset dut;
  dut.insert("hello");
  dut.insert("hello");
  const int count = 2;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
  // Demonstrate that the container is unordered.
  EXPECT_GT(dut.bucket_count(), 0);
}

GTEST_TEST(StringContainerTest, string_map) {
  string_map<int> dut;
  dut.emplace("hello", 1);
  dut.emplace("hello", 2);
  const int count = 1;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
}

GTEST_TEST(StringContainerTest, string_multimap) {
  string_multimap<int> dut;
  dut.emplace("hello", 1);
  dut.emplace("hello", 2);
  const int count = 2;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
}

GTEST_TEST(StringContainerTest, string_unordered_map) {
  string_unordered_map<int> dut;
  dut.emplace("hello", 1);
  dut.emplace("hello", 2);
  const int count = 1;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
  // Demonstrate that the container is unordered.
  EXPECT_GT(dut.bucket_count(), 0);
}

GTEST_TEST(StringContainerTest, string_unordered_multimap) {
  string_unordered_multimap<int> dut;
  dut.emplace("hello", 1);
  dut.emplace("hello", 2);
  const int count = 2;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
  // Demonstrate that the container is unordered.
  EXPECT_GT(dut.bucket_count(), 0);
}

}  // namespace
}  // namespace drake
