#include "drake/perception/point_cloud_flags.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace perception {

namespace pcf = pc_flags;

namespace {

GTEST_TEST(PointCloudFlagsTest, ConstExpr) {
  // @note This is demonstrating that `DescriptorType` can be a literal type
  // (suitable for being a `constexpr`). However, this is not necessary
  // to ensure this contract, as the compiler will throw an error if a
  // `constexpr` is attempted with a non-literal-type class (as of C++14).
  // TODO(eric.cousineau): Replace with the successor of is_literal_type.
  // @ref https://stackoverflow.com/a/40352351/7829525
  EXPECT_TRUE(std::is_literal_type_v<pcf::DescriptorType>);
}

GTEST_TEST(PointCloudFlagsTest, Formatting) {
  // Check human-friendly formatting.
  {
    std::ostringstream os;
    os << (pcf::kXYZs | pcf::kDescriptorCurvature);
    EXPECT_EQ("(kXYZs | kDescriptorCurvature)", os.str());
  }
  // Ensure it works for multiple types.
  {
    std::ostringstream os;
    os << (pcf::kXYZs | pcf::kRGBs | pcf::kDescriptorCurvature);
    EXPECT_EQ("(kXYZs | kRGBs | kDescriptorCurvature)", os.str());
  }
}

GTEST_TEST(PointCloudFlagsTest, Basic) {
  // Check basics.
  pcf::Fields lhs = pcf::kXYZs;
  pcf::Fields rhs = pcf::kXYZs;
  EXPECT_EQ(lhs, rhs);
  EXPECT_TRUE(lhs.contains(pcf::kXYZs));
  EXPECT_FALSE(lhs.contains(pcf::kDescriptorCurvature));
  lhs |= pcf::kDescriptorFPFH;
  EXPECT_NE(lhs, rhs);
  EXPECT_TRUE(lhs.contains(pcf::kDescriptorFPFH));

  // TODO(eric.cousineau): Add check for intersection when there is more
  // than one `BaseField`.
  EXPECT_EQ(lhs & pcf::kXYZs, pcf::kXYZs);
  EXPECT_EQ(lhs & pcf::kDescriptorFPFH, pcf::kDescriptorFPFH);

  // Check implicit conversion.
  EXPECT_EQ(pcf::Fields(pcf::kXYZs), pcf::kXYZs);
  EXPECT_EQ(pcf::Fields(pcf::kNone), pcf::Fields(pcf::kDescriptorNone));

  // Check negatives.
  EXPECT_THROW(pcf::Fields(-100), std::runtime_error);
  EXPECT_THROW(pcf::Fields(100), std::runtime_error);

  // Check combinations with `None` (effectively zero) values.
  DRAKE_EXPECT_NO_THROW(pcf::kDescriptorNone | pcf::kDescriptorCurvature);
  DRAKE_EXPECT_NO_THROW(pcf::kNone | pcf::kDescriptorCurvature);

  // Cannot have two descriptors.
  EXPECT_THROW(pcf::kDescriptorFPFH | pcf::kDescriptorCurvature,
               std::runtime_error);
}

}  // namespace
}  // namespace perception
}  // namespace drake
