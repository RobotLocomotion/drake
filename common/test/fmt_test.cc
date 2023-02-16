#include "drake/common/fmt.h"

#include <ostream>

#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

// This namespace contains example code (i.e., what a user would write) for
// formatting a class (or struct) using the "format as" helper.
namespace sample {

// A simple, non-templated struct.
struct Int {
  int i{};
};

// A template with 1 type argument.
template <typename T>
struct Wrap {
  T t{};
};

// A template with 2 type arguments.
template <typename T, typename U>
struct Pair {
  T t{};
  U u{};
};

}  // namespace sample

// Tell fmt how to format the sample types.
DRAKE_FORMATTER_AS(, sample, Int, x, x.i)
DRAKE_FORMATTER_AS(typename T, sample, Wrap<T>, x, x.t)
DRAKE_FORMATTER_AS(typename... Ts, sample, Pair<Ts...>, x, std::pair(x.t, x.u))

namespace drake {
namespace {

// Spot check the for "format as" formatter.
GTEST_TEST(FmtTest, FormatAsFormatter) {
  const sample::Int plain{1};
  EXPECT_EQ(fmt::format("{}", plain), "1");
  EXPECT_EQ(fmt::format("{:3}", plain), "  1");
  EXPECT_EQ(fmt::format("{:<3}", plain), "1  ");

  const sample::Wrap<double> real{1.1234567e6};
  EXPECT_EQ(fmt::format("{}", real), "1123456.7");
  EXPECT_EQ(fmt::format("{:.3G}", real), "1.12E+06");

  // N.B. The fmt:formatter for std::pair comes from <fmt/ranges.h>.
  const sample::Pair<int, int> pear{1, 2};
  EXPECT_EQ(fmt::format("{}", pear), "(1, 2)");
}

// The googletest infrastructure uses fmt's formatters.
GTEST_TEST(FmtTest, TestPrinter) {
  const sample::Int plain{1};
  EXPECT_EQ(testing::PrintToString(plain), "1");

  const sample::Wrap<double> real{1.1};
  EXPECT_EQ(testing::PrintToString(real), "1.1");

  const sample::Pair<int, int> pear{1, 2};
  EXPECT_EQ(testing::PrintToString(pear), "(1, 2)");
}

}  // namespace
}  // namespace drake
