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
  color.set(1., 1., 1., 0.);
  EXPECT_EQ(color, Rgba(1., 1., 1., 0.));
}

GTEST_TEST(RgbaTest, Errors) {
  // Ensure that errors do not corrupt the values.
  const Rgba original(0.75, 0.5, 0.25, 1.);
  Rgba color = original;
  DRAKE_EXPECT_THROWS_MESSAGE(
      color.set(1.2, 0., 0.),
      std::runtime_error,
      "All values must be within the range \\[0, 1\\]. Values provided: "
      "\\(r=1.2, g=0.0, b=0.0, a=1.0\\)");
  EXPECT_EQ(color, original);
  EXPECT_THROW(color.set(-1.2, 0., 0.), std::runtime_error);
  EXPECT_EQ(color, original);
  // Check constructors.
  EXPECT_THROW(Rgba(0., 1.2, 0.), std::runtime_error);
  EXPECT_THROW(Rgba(0, 0, 0, 1.2), std::runtime_error);
}

}  // namespace geometry
}  // namespace drake
