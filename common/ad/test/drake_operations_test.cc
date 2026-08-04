#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace ad {
namespace {

using Eigen::Vector3d;

class DrakeOperationsTest : public ::testing::Test {
 protected:
  const AutoDiff w_{0.5};
  const AutoDiff x_{0.4, Vector3d::LinSpaced(0.0, 2.0)};
  const AutoDiff y_{0.3, Vector3d::LinSpaced(-1.0, 1.0)};
  const AutoDiff z_{0.2, 3, 1};
};

TEST_F(DrakeOperationsTest, ExtractDouble) {
  EXPECT_EQ(ExtractDoubleOrThrow(w_), 0.5);
  EXPECT_EQ(ExtractDoubleOrThrow(x_), 0.4);
  EXPECT_EQ(ExtractDoubleOrThrow(y_), 0.3);
  EXPECT_EQ(ExtractDoubleOrThrow(z_), 0.2);
}

TEST_F(DrakeOperationsTest, IfThenElse) {
  EXPECT_EQ(if_then_else(true, w_, x_), w_);
  EXPECT_EQ(if_then_else(false, w_, x_), x_);
  EXPECT_EQ(if_then_else(true, x_, y_), x_);
  EXPECT_EQ(if_then_else(false, x_, y_), y_);
  EXPECT_EQ(if_then_else(true, y_, z_), y_);
  EXPECT_EQ(if_then_else(false, y_, z_), z_);
}

TEST_F(DrakeOperationsTest, Cond) {
  EXPECT_EQ(cond(true, w_, true, x_, true, y_, z_), w_);
  EXPECT_EQ(cond(false, w_, true, x_, true, y_, z_), x_);
  EXPECT_EQ(cond(false, w_, false, x_, true, y_, z_), y_);
  EXPECT_EQ(cond(false, w_, false, x_, false, y_, z_), z_);
}

}  // namespace
}  // namespace ad
}  // namespace drake
