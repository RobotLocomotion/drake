#include "drake/perception/point_cloud_flags.h"

// TODO(2026-06-01): Remove sstream header when `Formatting` is removed.
#include <sstream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace perception {

namespace pcf = pc_flags;

namespace {

// TODO(2026-06-01): Delete test `Formatting`.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
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
#pragma GCC diagnostic pop

GTEST_TEST(PointCloudFlagsTest, DescriptorTypeToStringFmtFormatter) {
  EXPECT_EQ(fmt::to_string(pcf::kDescriptorNone), "kDescriptorNone");
}

GTEST_TEST(PointCloudFlagsTest, FieldsToStringFmtFormatter) {
  EXPECT_EQ(fmt::to_string(pcf::Fields(pcf::kDescriptorNone)), "()");
  EXPECT_EQ(fmt::to_string(pcf::kXYZs | pcf::kDescriptorNone), "(kXYZs)");
  EXPECT_EQ(fmt::to_string(pcf::kXYZs | pcf::kNormals | pcf::kRGBs |
                           pcf::kDescriptorCurvature),
            "(kXYZs | kNormals | kRGBs | kDescriptorCurvature)");
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
