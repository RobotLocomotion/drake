#include "drake/perception/point_cloud.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

using Eigen::Matrix3Xf;
using Eigen::Matrix4Xf;

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;


namespace drake {
namespace perception {
namespace {

// Provides a helper mechanism to (a) check default types and (b) compare
// matrices, handling the special case of unsigned values.
template <typename T>
struct check_helper {
  template <typename XprType>
  static bool IsDefault(const XprType& xpr) {
    return xpr.array().isNaN().all();
  }

  template <typename XprTypeA, typename XprTypeB>
  static AssertionResult Compare(const XprTypeA& a, const XprTypeB& b) {
    return CompareMatrices(a, b);
  }
};

template <>
struct check_helper<uint8_t> {
  template <typename XprType>
  static bool IsDefault(const XprType& xpr) {
    return (xpr.array() == 0).all();
  }

  template <typename XprTypeA, typename XprTypeB>
  static AssertionResult Compare(const XprTypeA& a, const XprTypeB& b) {
    if ((a.array() == b.array()).all()) {
      return AssertionSuccess();
    } else {
      return AssertionFailure();
    }
  }
};

GTEST_TEST(PointCloudTest, Basic) {
  const int count = 5;

  auto CheckFields = [count](auto values_expected, pc_flags::Fields fields,
                             auto get_mutable_values, auto get_values,
                             auto get_mutable_value, auto get_value) {
    PointCloud cloud(count, fields);
    EXPECT_EQ(count, cloud.size());
    typedef decltype(values_expected) XprType;
    typedef typename XprType::Scalar T;

    // Shim normal comparison to use unsigned-type friendly comparators.
    auto CompareMatrices = [](const auto& a, const auto& b) {
      return check_helper<T>::Compare(a, b);
    };

    // Expect the values to be default-initialized.
    EXPECT_TRUE(check_helper<T>::IsDefault(get_mutable_values(cloud)));

    // Set values using the mutable accessor.
    get_mutable_values(cloud) = values_expected;

    EXPECT_TRUE(CompareMatrices(values_expected, get_mutable_values(cloud)));
    EXPECT_TRUE(CompareMatrices(values_expected, get_values(cloud)));

    // Check element-by-element.
    for (int i = 0; i < count; ++i) {
      const auto value_expected = values_expected.col(i);
      EXPECT_TRUE(CompareMatrices(value_expected, get_value(cloud, i)));
      EXPECT_TRUE(CompareMatrices(value_expected, get_mutable_value(cloud, i)));
    }

    // Check copy and move constructors.
    {
      PointCloud cloud_copy(cloud);
      EXPECT_TRUE(
          CompareMatrices(values_expected, get_values(cloud_copy)));

      PointCloud cloud_move(std::move(cloud_copy));
      EXPECT_TRUE(
          CompareMatrices(values_expected, get_values(cloud_move)));
      // Ensure the original cloud was emptied out.
      EXPECT_EQ(0, cloud_copy.size());
    }

    // Check copy and move assignment.
    {
      PointCloud cloud_copy(0, fields);
      cloud_copy = cloud;
      EXPECT_TRUE(
          CompareMatrices(values_expected, get_values(cloud_copy)));

      PointCloud cloud_move(0, fields);
      cloud_move = std::move(cloud_copy);
      EXPECT_TRUE(
          CompareMatrices(values_expected, get_values(cloud_move)));
      // Ensure the original cloud was emptied out.
      EXPECT_EQ(0, cloud_copy.size());
    }

    // Add item which should be default-initialized.
    int last = cloud.size();
    cloud.Expand(1);
    EXPECT_EQ(count + 1, cloud.size());
    // Check default-initialized.
    EXPECT_TRUE(check_helper<T>::IsDefault(get_value(cloud, last)));
    // Ensure that we preserve the values.
    EXPECT_TRUE(
        CompareMatrices(values_expected,
                        get_values(cloud).middleCols(0, last)));

    // Resize to a size smaller.
    int small_size = 3;
    cloud.resize(small_size);
    EXPECT_EQ(small_size, cloud.size());
    EXPECT_TRUE(
        CompareMatrices(values_expected.middleCols(0, small_size),
                        get_values(cloud)));

    // Resize to a size larger.
    int large_size = 6;
    cloud.resize(large_size);
    EXPECT_EQ(large_size, cloud.size());
    EXPECT_TRUE(
        CompareMatrices(values_expected.middleCols(0, small_size),
                        get_values(cloud).middleCols(0, small_size)));
    EXPECT_TRUE(
        check_helper<T>::IsDefault(
            get_values(cloud).middleCols(small_size, large_size - small_size)));
  };

  // Points.
  Matrix3Xf xyzs_expected(3, count);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(xyzs_expected, pc_flags::kXYZs,
              [](PointCloud& cloud) { return cloud.mutable_xyzs(); },
              [](PointCloud& cloud) { return cloud.xyzs(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_xyz(i); },
              [](PointCloud& cloud, int i) { return cloud.xyz(i); });

  // Normals.
  Matrix3Xf normals_expected(3, count);
  normals_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  CheckFields(normals_expected, pc_flags::kNormals,
              [](PointCloud& cloud) { return cloud.mutable_normals(); },
              [](PointCloud& cloud) { return cloud.normals(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_normal(i); },
              [](PointCloud& cloud, int i) { return cloud.normal(i); });

  // RGBs.
  Matrix3X<uint8_t> rgbs_expected(3, count);
  rgbs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 255,
    4, 5, 6,
    40, 50, 60;
  CheckFields(rgbs_expected, pc_flags::kRGBs,
            [](PointCloud& cloud) { return cloud.mutable_rgbs(); },
            [](PointCloud& cloud) { return cloud.rgbs(); },
            [](PointCloud& cloud, int i) { return cloud.mutable_rgb(i); },
            [](PointCloud& cloud, int i) { return cloud.rgb(i); });

  // Descriptors (Curvature).
  Eigen::RowVectorXf descriptors_expected(count);
  descriptors_expected <<
    1, 2, 3, 4, 5;
  CheckFields(descriptors_expected, pc_flags::kDescriptorCurvature,
              [](PointCloud& cloud) { return cloud.mutable_descriptors(); },
              [](PointCloud& cloud) { return cloud.descriptors(); },
              [](PointCloud& cloud, int i) {
                return cloud.mutable_descriptor(i);
              },
              [](PointCloud& cloud, int i) { return cloud.descriptor(i); });
}

GTEST_TEST(PointCloudTest, Fields) {
  // Check zero-size.
  {
    PointCloud cloud(0, pc_flags::kXYZs);
    EXPECT_EQ(0, cloud.size());
  }

  // Check basic requirements.
  {
    PointCloud cloud(1, pc_flags::kXYZs);
    EXPECT_TRUE(cloud.has_xyzs());
    EXPECT_TRUE(cloud.HasFields(pc_flags::kXYZs));
    DRAKE_EXPECT_NO_THROW(cloud.RequireFields(pc_flags::kXYZs));
    EXPECT_FALSE(cloud.HasFields(pc_flags::kDescriptorFPFH));
    EXPECT_THROW(cloud.RequireFields(pc_flags::kDescriptorFPFH),
                 std::runtime_error);
  }

  // Check with exact fields.
  {
    auto fields = pc_flags::kXYZs | pc_flags::kNormals | pc_flags::kRGBs |
                  pc_flags::kDescriptorCurvature;
    PointCloud cloud(1, fields);
    EXPECT_TRUE(cloud.HasExactFields(fields));
    DRAKE_EXPECT_NO_THROW(cloud.RequireExactFields(fields));
    EXPECT_FALSE(cloud.HasExactFields(pc_flags::kXYZs));
    EXPECT_THROW(cloud.RequireExactFields(pc_flags::kXYZs),
                 std::runtime_error);
  }

  // Check invalid fields.
  {
    EXPECT_THROW(PointCloud(1, 0), std::runtime_error);
    const int invalid_flag_value = pc_flags::internal::kMaxBitInUse << 1;
    EXPECT_THROW(PointCloud(1, invalid_flag_value), std::runtime_error);
    EXPECT_THROW(PointCloud(1, -invalid_flag_value), std::runtime_error);
  }

  // Check with descriptors.
  {
    PointCloud cloud(1, pc_flags::kDescriptorCurvature);
    EXPECT_FALSE(cloud.has_xyzs());
    EXPECT_TRUE(cloud.has_descriptors());
    EXPECT_TRUE(cloud.has_descriptors(pc_flags::kDescriptorCurvature));
    EXPECT_FALSE(cloud.has_descriptors(pc_flags::kDescriptorFPFH));

    // Negative tests for `has_descriptors`.
    PointCloud simple_cloud(1, pc_flags::kXYZs);
    EXPECT_FALSE(simple_cloud.has_descriptors());
    EXPECT_FALSE(simple_cloud.has_descriptors(pc_flags::kDescriptorCurvature));

    // Negative tests for construction.
    EXPECT_THROW(PointCloud(1, pc_flags::kNone),
                     std::runtime_error);
    EXPECT_THROW(PointCloud(1, pc_flags::kDescriptorNone),
                 std::runtime_error);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
