#include "drake/common/fmt_eigen.h"

#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace {

/* The implementation of fmt_eigen reserves 20 characters to format each scalar
in an Eigen::Matrix. If the formatted size exceeds that, everything still works,
but it will cause a re-allocation (using exponential growth). This scalar type
allows probing that in a LimitMalloc test case, because its formated size is 30
characters per scalar. */
struct BigToken {
  static std::string_view to_string() {
    return "0123456789"
           "0123456789"
           "0123456789";
  }
};

}  // namespace
}  // namespace drake

DRAKE_FORMATTER_AS(, drake, BigToken, x, BigToken::to_string())

namespace drake {
namespace {

GTEST_TEST(FmtEigenTest, DocumentationExamples) {
  const Eigen::RowVector3d x{M_PI, M_SQRT2, M_E};
  EXPECT_EQ(fmt::format("{}", fmt_eigen(x)),
            " 3.141592653589793 1.4142135623730951  2.718281828459045");
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(x)), "3.14 1.41 2.72");
  EXPECT_EQ(fmt::format("{x::e}", fmt::arg("x", fmt_eigen(x))),
            "3.141593e+00 1.414214e+00 2.718282e+00");
}

GTEST_TEST(FmtEigenTest, Precision) {
  // This constant needs the maximum number of digits to round-trip correctly.
  const double scalar = 0.12345678901234566;
  const Eigen::VectorXd value = Eigen::VectorXd::Constant(1, scalar);

  // Sanity check the scalar formatting without fmt_eigen.
  const std::string_view baseline{"0.12345678901234566"};
  ASSERT_EQ(fmt::to_string(scalar), baseline);

  // Default format.
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(value)), "0.12");
}

GTEST_TEST(FmtEigenTest, RowVector3d) {
  const Eigen::RowVector3d value{1.1, 2.2, 3.3};

  // Default format.
  const std::string_view baseline{"1.1 2.2 3.3"};
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(value)), "1.10 2.20 3.30");
}

GTEST_TEST(FmtEigenTest, Vector3d) {
  const Eigen::Vector3d value{1.1, 2.2, 3.3};

  // Default format.
  std::string_view baseline{"1.1\n2.2\n3.3"};
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(value)), "1.10\n2.20\n3.30");
}

GTEST_TEST(FmtEigenTest, EmptyMatrix) {
  const Eigen::MatrixXd value;

  // Default format.
  const std::string_view baseline;
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // Scalar format string modifiers are accepeted, but no-ops.
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(value)), baseline);
}

GTEST_TEST(FmtEigenTest, Matrix3d) {
  Eigen::Matrix3d value;
  // clang-format off
  value << 1.1, 1.2, 1.3,
           2.1, 2.2, 2.3,
           3.1, 3.2, 3.3;
  // clang-format on

  // Default format.
  const std::string_view baseline{
      "1.1 1.2 1.3\n"
      "2.1 2.2 2.3\n"
      "3.1 3.2 3.3"};
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(value)),
            "1.10 1.20 1.30\n"
            "2.10 2.20 2.30\n"
            "3.10 3.20 3.30");
}

GTEST_TEST(FmtEigenTest, Matrix3dNeedsPadding) {
  Eigen::Matrix3d value;
  // clang-format off
  value << 10.1, 1.2, 1.3,
           2.1, 2.2, 2.3,
           3.1, 3.2, 3.3;
  // clang-format on

  // Default format.
  const std::string_view baseline{
      "10.1  1.2  1.3\n"
      " 2.1  2.2  2.3\n"
      " 3.1  3.2  3.3"};
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("{::.2f}", fmt_eigen(value)),
            "10.10  1.20  1.30\n"
            " 2.10  2.20  2.30\n"
            " 3.10  3.20  3.30");
}

GTEST_TEST(FmtEigenTest, Matrix3i) {
  Eigen::Matrix3i value;
  // clang-format off
  value << 11, 12, 13,
           21, 22, 23,
           31, 32, 33;
  // clang-format on

  // Default format.
  const std::string_view baseline{
      "11 12 13\n"
      "21 22 23\n"
      "31 32 33"};
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("value =\n"
                        "{0}\n"
                        "value as hex =\n"
                        "{0::x}",
                        fmt_eigen(value)),
            "value =\n" + std::string{baseline} + "\n" +
                "value as hex =\n"
                " b  c  d\n"
                "15 16 17\n"
                "1f 20 21");
}

GTEST_TEST(FmtEigenTest, MatrixString) {
  Eigen::MatrixX<std::string> value(2, 3);
  // clang-format off
  value << "hello", "world", "!",
           "goodbye", "cruel", "world";
  // clang-format on

  // Default format.
  const std::string_view baseline{
      "  hello   world       !\n"
      "goodbye   cruel   world"};
  EXPECT_EQ(fmt::to_string(fmt_eigen(value)), baseline);
  EXPECT_EQ(fmt::format("{}", fmt_eigen(value)), baseline);

  // With Scalar format string modifiers.
  EXPECT_EQ(fmt::format("{::?}", fmt_eigen(value)),
            "  \"hello\"   \"world\"       \"!\"\n"
            "\"goodbye\"   \"cruel\"   \"world\"");
}

GTEST_TEST(FmtEigenTest, AllocationsNominal) {
  Eigen::RowVectorXd value(6);
  value << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
  std::string formatted;
  {
    // We expect exactly 5 allocations:
    // • 3 vector reservations in the header,
    // • 1 string return value of FormatMatrix,
    // • 1 string copy within fmt.
    test::LimitMalloc guard({.max_num_allocations = 5});
    formatted = fmt::to_string(fmt_eigen(value));
  }
  EXPECT_EQ(formatted, "1.1 2.2 3.3 4.4 5.5 6.6");
}

GTEST_TEST(FmtEigenTest, AllocationsOverflow) {
  const Eigen::RowVector3<BigToken> value;
  std::string formatted;
  {
    // We expect exactly 6 allocations:
    // • 3 vector reservations in the header,
    //   ◦ 1 vector reallocation (element_buffer reservation was insufficient)
    // • 1 string return value of FormatMatrix,
    // • 1 string copy within fmt.
    test::LimitMalloc guard({.max_num_allocations = 6});
    formatted = fmt::to_string(fmt_eigen(value));
  }
  EXPECT_EQ(formatted, std::string{BigToken::to_string()} + " " +
                           std::string{BigToken::to_string()} + " " +
                           std::string{BigToken::to_string()});
}

// Regression against the set of supported Scalar types.
GTEST_TEST(FmtEigenTest, StringifyErrorDetailValue) {
  using internal::StringifyErrorDetailValue;
  {
    Eigen::VectorX<double> v(1);
    v << 1.5;
    EXPECT_NO_THROW(StringifyErrorDetailValue(fmt_eigen(v)));
  }
  {
    Eigen::VectorX<float> v(1);
    v << 1.5f;
    EXPECT_NO_THROW(StringifyErrorDetailValue(fmt_eigen(v)));
  }
  {
    Eigen::VectorX<int> v(1);
    v << 15;
    EXPECT_NO_THROW(StringifyErrorDetailValue(fmt_eigen(v)));
  }
  {
    Eigen::VectorX<std::string> v(1);
    v << "1.5";
    EXPECT_NO_THROW(StringifyErrorDetailValue(fmt_eigen(v)));
  }
}

}  // namespace
}  // namespace drake
