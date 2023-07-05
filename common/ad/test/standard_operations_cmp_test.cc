#include <sstream>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

#define DRAKE_CHECK_CMP(cmp)                             \
  EXPECT_EQ(0 cmp 1, AutoDiffDut{0} cmp AutoDiffDut{1}); \
  EXPECT_EQ(1 cmp 0, AutoDiffDut{1} cmp AutoDiffDut{0}); \
  EXPECT_EQ(1 cmp 1, AutoDiffDut{1} cmp AutoDiffDut{1}); \
  EXPECT_EQ(0 cmp 1, AutoDiffDut{0} cmp 1); /* NOLINT */ \
  EXPECT_EQ(1 cmp 0, AutoDiffDut{1} cmp 0); /* NOLINT */ \
  EXPECT_EQ(1 cmp 1, AutoDiffDut{1} cmp 1); /* NOLINT */ \
  EXPECT_EQ(0 cmp 1, 0 cmp AutoDiffDut{1});              \
  EXPECT_EQ(1 cmp 0, 1 cmp AutoDiffDut{0});              \
  EXPECT_EQ(1 cmp 1, 1 cmp AutoDiffDut{1})

TEST_F(StandardOperationsTest, CmpLt) {
  DRAKE_CHECK_CMP(<);  // NOLINT
}

TEST_F(StandardOperationsTest, CmpLe) {
  DRAKE_CHECK_CMP(<=);
}

TEST_F(StandardOperationsTest, CmpGt) {
  DRAKE_CHECK_CMP(>);  // NOLINT
}

TEST_F(StandardOperationsTest, CmpGe) {
  DRAKE_CHECK_CMP(>=);
}

TEST_F(StandardOperationsTest, CmpEq) {
  DRAKE_CHECK_CMP(==);
}

TEST_F(StandardOperationsTest, CmpNe) {
  DRAKE_CHECK_CMP(!=);
}

#undef DRAKE_CHECK_CMP

}  // namespace
}  // namespace test
}  // namespace drake
