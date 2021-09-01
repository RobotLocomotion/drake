#include "drake/geometry/rgba.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {

GTEST_TEST(RgbaTest, Basic) {
  const double r = 0.75;
  const double g = 0.5;
  const double b = 0.25;
  const double a = 1.;
  Rgba color(r, g, b);
  EXPECT_EQ(color.r(), r);
  EXPECT_EQ(color.g(), g);
  EXPECT_EQ(color.b(), b);
  EXPECT_EQ(color.a(), a);
  EXPECT_TRUE(color == Rgba(r, g, b, a));
  EXPECT_TRUE(color != Rgba(r, g, b, 0.));
  const double kEps = 1e-8;
  const Rgba color_delta(color.r() + kEps, color.g() - kEps, color.b() + kEps,
                         color.a() - kEps);
  EXPECT_TRUE(color.AlmostEqual(color_delta, 1.001 * kEps));
  EXPECT_FALSE(color.AlmostEqual(color_delta, 0.999 * kEps));
  color.set(1., 1., 1., 0.);
  EXPECT_EQ(color, Rgba(1., 1., 1., 0.));
}

GTEST_TEST(RgbaTest, Errors) {
  const Rgba original(0.1, 0.2, 0.3, 0.4);

  auto expect_error = [original](double ri, double gi, double bi, double ai) {
    const std::string expected_message = fmt::format(
        "All values must be within the range \\[0, 1\\]. Values provided: "
        "\\(r={}, g={}, b={}, a={}\\)", ri, gi, bi, ai);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Rgba(ri, gi, bi, ai),
        std::runtime_error,
        expected_message);
    // Check for transaction integrity.
    Rgba color = original;
    DRAKE_EXPECT_THROWS_MESSAGE(
        color.set(ri, gi, bi, ai),
        std::runtime_error,
        expected_message);
    EXPECT_EQ(color, original);
  };

  const double bad_low = -0.1;
  const double bad_high = 1.1;
  const double r = 0.75;
  const double g = 0.5;
  const double b = 0.25;
  const double a = 1.;

  expect_error(bad_low, g, b, a);
  expect_error(bad_high, g, b, a);
  expect_error(r, bad_low, b, a);
  expect_error(r, bad_high, b, a);
  expect_error(r, g, bad_low, a);
  expect_error(r, g, bad_high, a);
  expect_error(r, g, b, bad_low);
  expect_error(r, g, b, bad_high);
}

}  // namespace geometry
}  // namespace drake
