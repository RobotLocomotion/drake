#include "drake/common/drake_assert.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {
struct AbortRecord {
  std::string condition;
  std::string func;
  std::string file;
  int line;
};
std::vector<AbortRecord> g_records;
}  // namespace

// We provide a new definition of Abort for unit testing.
namespace drake {
void Abort(const char* condition, const char* func, const char* file, int line) {
  AbortRecord record{
    condition ? condition : "",
    func ? func : "",
    file ? file : "",
    line};
  g_records.push_back(record);
}
}  // namespace drake

namespace {

GTEST_TEST(DrakeAssertTest, AbortTest) {
  EXPECT_EQ(g_records.size(), 0);

  int expected_line{};
  expected_line = __LINE__ + 1;
  DRAKE_ABORT();
  EXPECT_EQ(g_records.size(), 1);
  EXPECT_EQ(g_records.front().condition, "");
  EXPECT_EQ(g_records.front().func, "TestBody");
  EXPECT_EQ(g_records.front().file.empty(), false);
  EXPECT_EQ(g_records.front().line, expected_line);
  g_records.clear();

  expected_line = __LINE__ + 1;
  DRAKE_ABORT_UNLESS(false);
  EXPECT_EQ(g_records.size(), 1);
  EXPECT_EQ(g_records.front().condition, "false");
  EXPECT_EQ(g_records.front().func, "TestBody");
  EXPECT_EQ(g_records.front().file.empty(), false);
  EXPECT_EQ(g_records.front().line, expected_line);
  g_records.clear();

  DRAKE_ABORT_UNLESS(true);
  EXPECT_EQ(g_records.size(), 0);
  g_records.clear();
}

struct BoolConvertible {
  operator bool() { return true; }
};

GTEST_TEST(DrakeAssertTest, SyntaxTest) {
  // These should compile.
  DRAKE_ASSERT((2 + 2) == 4);
  DRAKE_ASSERT(BoolConvertible());
}

// Only run this test if assertions are armed.
#ifndef DRAKE_ASSERT_IS_VOID
GTEST_TEST(DrakeAssertTest, AssertFalseTest) {
  EXPECT_EQ(g_records.size(), 0);

  int expected_line{};
  expected_line = __LINE__ + 1;
  DRAKE_ASSERT((2 + 2) == 5);
  EXPECT_EQ(g_records.size(), 1);
  EXPECT_EQ(g_records.front().condition, "(2 + 2) == 5");
  EXPECT_EQ(g_records.front().func, "TestBody");
  EXPECT_EQ(g_records.front().file.empty(), false);
  EXPECT_EQ(g_records.front().line, expected_line);
  g_records.clear();
}
#endif  //  DRAKE_ASSERT_IS_VOID

}  // namespace
