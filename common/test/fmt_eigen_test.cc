#include "drake/common/fmt_eigen.h"

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace drake {
namespace {

// TODO(jwnimer-tri) The expected values below are what Eigen prints for us by
// default. In the future, we might decide to use some different formatting.
// In that case, it's fine to update the test goals here to reflect those
// updated defaults.

GTEST_TEST(FmtEigenTest, RowVector3d) {
  const Eigen::RowVector3d value{1.1, 2.2, 3.3};
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), "1.1 2.2 3.3");
}

GTEST_TEST(FmtEigenTest, Vector3d) {
  const Eigen::Vector3d value{1.1, 2.2, 3.3};
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), "1.1\n2.2\n3.3");
}

GTEST_TEST(FmtEigenTest, EmptyMatrix) {
  const Eigen::MatrixXd value;
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), "");
}

GTEST_TEST(FmtEigenTest, Matrix3d) {
  Eigen::Matrix3d value;
  value << 1.1, 1.2, 1.3,
           2.1, 2.2, 2.3,
           3.1, 3.2, 3.3;
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)),
            "1.1 1.2 1.3\n"
            "2.1 2.2 2.3\n"
            "3.1 3.2 3.3");
}

GTEST_TEST(FmtEigenTest, Matrix3dNeedsPadding) {
  Eigen::Matrix3d value;
  value << 10.1, 1.2, 1.3,
           2.1, 2.2, 2.3,
           3.1, 3.2, 3.3;
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)),
            "10.1  1.2  1.3\n"
            " 2.1  2.2  2.3\n"
            " 3.1  3.2  3.3");
}

GTEST_TEST(FmtEigenTest, Matrix3i) {
  Eigen::Matrix3i value;
  value << 11, 12, 13,
           21, 22, 23,
           31, 32, 33;
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)),
            "11 12 13\n"
            "21 22 23\n"
            "31 32 33");
}

}  // namespace
}  // namespace drake
