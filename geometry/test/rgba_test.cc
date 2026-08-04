#include "drake/geometry/rgba.h"

#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;

GTEST_TEST(RgbaTest, Default) {
  Rgba defaulted;
  Rgba opaque_white(1, 1, 1, 1);
  EXPECT_EQ(defaulted, opaque_white);
}

GTEST_TEST(RgbaTest, Basic) {
  const double r = 0.75;
  const double g = 0.5;
  const double b = 0.25;
  const double a = 1.0;
  Rgba color(r, g, b);
  EXPECT_EQ(color.r(), r);
  EXPECT_EQ(color.g(), g);
  EXPECT_EQ(color.b(), b);
  EXPECT_EQ(color.a(), a);
  EXPECT_EQ(color.rgba()[0], r);
  EXPECT_EQ(color.rgba()[1], g);
  EXPECT_EQ(color.rgba()[2], b);
  EXPECT_EQ(color.rgba()[3], a);
  EXPECT_TRUE(color == Rgba(r, g, b, a));
  EXPECT_TRUE(color != Rgba(r, g, b, 0.0));
  const double kEps = 1e-8;
  const Rgba color_delta(color.r() + kEps, color.g() - kEps, color.b() + kEps,
                         color.a() - kEps);
  EXPECT_TRUE(color.AlmostEqual(color_delta, 1.001 * kEps));
  EXPECT_FALSE(color.AlmostEqual(color_delta, 0.999 * kEps));
  color.set(1.0, 1.0, 1.0, 0.0);
  EXPECT_EQ(color, Rgba(1.0, 1.0, 1.0, 0.0));
  color.set(Vector4d{0.1, 0.2, 0.3, 0.4});
  EXPECT_EQ(color, Rgba(0.1, 0.2, 0.3, 0.4));
  color.set(Vector3d{0.5, 0.6, 0.7});
  EXPECT_EQ(color, Rgba(0.5, 0.6, 0.7, 1.0));
}

GTEST_TEST(RgbaTest, Update) {
  Rgba color(0.1, 0.2, 0.3, 0.4);
  color.update();
  EXPECT_EQ(color.rgba(), Vector4d(0.1, 0.2, 0.3, 0.4));
  color.update(0.6, 0.7, 0.8);
  EXPECT_EQ(color.rgba(), Vector4d(0.6, 0.7, 0.8, 0.4));
  color.update({}, {}, {}, 0.9);
  EXPECT_EQ(color.rgba(), Vector4d(0.6, 0.7, 0.8, 0.9));
}

GTEST_TEST(RgbaTest, Errors) {
  const Rgba original(0.1, 0.2, 0.3, 0.4);

  auto expect_error = [original](double ri, double gi, double bi, double ai) {
    const std::string expected_message = fmt::format(
        "Rgba values must be within the range \\[0, 1\\]. Values provided: "
        "\\(r={}, g={}, b={}, a={}\\)",
        ri, gi, bi, ai);
    DRAKE_EXPECT_THROWS_MESSAGE(Rgba(ri, gi, bi, ai), expected_message);
    // Check for transaction integrity.
    Rgba color = original;
    DRAKE_EXPECT_THROWS_MESSAGE(color.set(ri, gi, bi, ai), expected_message);
    EXPECT_EQ(color, original);
  };

  const double bad_low = -0.1;
  const double bad_high = 1.1;
  const double r = 0.75;
  const double g = 0.5;
  const double b = 0.25;
  const double a = 1.0;

  expect_error(bad_low, g, b, a);
  expect_error(bad_high, g, b, a);
  expect_error(r, bad_low, b, a);
  expect_error(r, bad_high, b, a);
  expect_error(r, g, bad_low, a);
  expect_error(r, g, bad_high, a);
  expect_error(r, g, b, bad_low);
  expect_error(r, g, b, bad_high);

  expect_error(r, g, b, std::numeric_limits<double>::quiet_NaN());
}

GTEST_TEST(RgbaTest, Product) {
  const Rgba a(0.25, 0.5, 0.75, 0.875);
  const Rgba b(0.75, 0.5, 0.25, 0.875);
  const Rgba c(a.r() * b.r(), a.g() * b.g(), a.b() * b.b(), a.a() * b.a());
  EXPECT_EQ(a * b, c);

  const Rgba a_color_scaled(a.r() * 1.1, a.g() * 1.1, a.b() * 1.1, a.a());
  EXPECT_EQ(a.scale_rgb(1.1), a_color_scaled);

  // Rgba channel values saturate at 1.
  EXPECT_EQ(a.scale_rgb(10), Rgba(1, 1, 1, a.a()));
}

GTEST_TEST(RgbaTest, ToString) {
  const Rgba a(0.25, 0.5, 0.75, 0.875);
  EXPECT_EQ(a.to_string(), "(0.25, 0.5, 0.75, 0.875)");
  EXPECT_EQ(fmt::to_string(a), "(0.25, 0.5, 0.75, 0.875)");
}

/** Confirm that this can be serialized appropriately. */
GTEST_TEST(RgbaTest, Serialization) {
  {
    // Serializing.
    const Rgba rgba(0.75, 0.25, 1.0, 0.5);
    const std::string y = yaml::SaveYamlString(rgba);
    EXPECT_EQ(y, "rgba: [0.75, 0.25, 1.0, 0.5]\n");
  }

  {
    // Deserialize: full specification.
    const Rgba rgba = yaml::LoadYamlString<Rgba>("rgba: [0.1, 0.2, 0.3, 0.4]");
    EXPECT_EQ(rgba, Rgba(0.1, 0.2, 0.3, 0.4));
  }

  {
    // Deserialize: alpha isn't necessary; alpha = 1 is provided.
    const Rgba rgba = yaml::LoadYamlString<Rgba>("rgba: [0.1, 0.2, 0.3]");
    EXPECT_EQ(rgba, Rgba(0.1, 0.2, 0.3, 1));
  }

  // Too-large YAML arrays are detected by the YAML parser using the MaxRows
  // template parameter of our Eigen vector declaratation; it never even calls
  // our Serialize() function in this case.
  DRAKE_EXPECT_THROWS_MESSAGE(
      yaml::LoadYamlString<Rgba>("rgba: [1, 1, 1, 1, 1]"),
      ".*maximum size is 4.*");

  // Too-small YAML arrays are detected by our Serialize() function.
  DRAKE_EXPECT_THROWS_MESSAGE(yaml::LoadYamlString<Rgba>("rgba: [1, 1]"),
                              "Rgba must contain either 3 or 4.+");

  // Out-of-range YAML values are detected by our Serialize() function.
  DRAKE_EXPECT_THROWS_MESSAGE(yaml::LoadYamlString<Rgba>("rgba: [1, 1, 2]"),
                              ".*must be within the range.+");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
