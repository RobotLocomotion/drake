#include <string>

#include <gtest/gtest.h>

#include "drake/common/string_map.h"
#include "drake/common/string_set.h"
#include "drake/common/string_unordered_map.h"
#include "drake/common/string_unordered_set.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace {

GTEST_TEST(StringContainerTest, string_set) {
  string_set dut;
  dut.insert("hello");
  dut.insert("hello");
  const int count = 1;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
}

GTEST_TEST(StringContainerTest, string_multiset) {
  string_multiset dut;
  dut.insert("hello");
  dut.insert("hello");
  const int count = 2;
  EXPECT_EQ(dut.count("hello"), count);
  EXPECT_EQ(dut.count(std::string{"hello"}), count);
  EXPECT_EQ(dut.count(std::string_view{"hello"}), count);
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

GTEST_TEST(StringContainerTest, NoMalloc) {
  string_set alpha;
  string_multiset bravo;
  string_unordered_set charlie;
  string_unordered_multiset delta;
  string_map<int> echo;
  string_multimap<int> foxtrot;
  string_unordered_map<int> gulf;
  string_unordered_multimap<int> hotel;

  alpha.insert("hello");
  bravo.insert("hello");
  charlie.insert("hello");
  delta.insert("hello");
  echo.emplace("hello", 1);
  foxtrot.emplace("hello", 1);
  gulf.emplace("hello", 1);
  hotel.emplace("hello", 1);

  {
    drake::test::LimitMalloc guard;

    alpha.contains("hello");
    bravo.contains("hello");
    charlie.contains("hello");
    delta.contains("hello");
    echo.contains("hello");
    foxtrot.contains("hello");
    gulf.contains("hello");
    hotel.contains("hello");

    alpha.contains(std::string_view{"world"});
    bravo.contains(std::string_view{"world"});
    charlie.contains(std::string_view{"world"});
    delta.contains(std::string_view{"world"});
    echo.contains(std::string_view{"world"});
    foxtrot.contains(std::string_view{"world"});
    gulf.contains(std::string_view{"world"});
    hotel.contains(std::string_view{"world"});
  }
}

}  // namespace
}  // namespace drake
