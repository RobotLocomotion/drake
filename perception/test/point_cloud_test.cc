#include "drake/perception/point_cloud.h"

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

using Eigen::Matrix3Xf;
using Eigen::Matrix4Xf;
using Eigen::Vector3f;

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

  {  // Crop
    PointCloud cloud(count, pc_flags::kXYZs | pc_flags::kNormals |
                                pc_flags::kRGBs |
                                pc_flags::kDescriptorCurvature);
    cloud.mutable_xyzs() = xyzs_expected;
    cloud.mutable_rgbs() = rgbs_expected;
    cloud.mutable_normals() = normals_expected;
    cloud.mutable_descriptors() = descriptors_expected;

    PointCloud cropped =
        cloud.Crop(Eigen::Vector3f{4, 5, 6}, Eigen::Vector3f{10, 20, 30});
    EXPECT_EQ(cropped.size(), 2);
    std::vector<int> indices{1, 3};

    for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
      EXPECT_TRUE(CompareMatrices(cropped.xyzs().col(i),
                                  xyzs_expected.col(indices[i])));
      EXPECT_TRUE((cropped.rgbs().col(i).array() ==
                   rgbs_expected.col(indices[i]).array())
                      .all());
      EXPECT_TRUE(CompareMatrices(cropped.normals().col(i),
                                  normals_expected.col(indices[i])));
      EXPECT_TRUE(CompareMatrices(cropped.descriptors().col(i),
                                  descriptors_expected.col(indices[i])));
    }
  }
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

GTEST_TEST(PointCloud, Concatenate) {
  auto fields = pc_flags::kXYZs | pc_flags::kNormals | pc_flags::kRGBs |
                pc_flags::kDescriptorCurvature;

  std::vector<PointCloud> clouds;
  // Populate three point clouds with arbitrary values.
  clouds.push_back(PointCloud(2, fields, true));
  clouds.push_back(PointCloud(3, fields, true));
  clouds.push_back(PointCloud(1, fields, true));

  clouds[0].mutable_xyzs() << 1, 2, 3, 4, 5, 6;
  clouds[0].mutable_rgbs() << 10, 20, 30, 40, 50, 60;
  clouds[0].mutable_normals() << 3, 4, -1, 12, 4, 32;
  clouds[0].mutable_descriptors() << 8, 2;

  clouds[1].mutable_xyzs() << 421, 1, 1, 4, 4, 2, 141, 1, 63;
  clouds[1].mutable_rgbs() << 57, 1, 42, 25, 25, 10, 12, 63, 192;
  clouds[1].mutable_normals() << -2, 1, 532, 7, 2, 6, 36, 84, 42;
  clouds[1].mutable_descriptors() << 10, 52, 1;

  clouds[2].mutable_xyzs() << 71, 2, 1;
  clouds[2].mutable_rgbs() << 53, 24, 172;
  clouds[2].mutable_normals() << 91, 4, 44;
  clouds[2].mutable_descriptors() << 91;

  PointCloud merged = Concatenate(clouds);
  EXPECT_EQ(merged.size(), 6);

  Eigen::Matrix3Xf xyzs_expected(3, 6);
  xyzs_expected << clouds[0].xyzs(), clouds[1].xyzs(), clouds[2].xyzs();
  EXPECT_TRUE(CompareMatrices(merged.xyzs(), xyzs_expected));

  Matrix3X<uint8_t> rgbs_expected(3, 6);
  rgbs_expected << clouds[0].rgbs(), clouds[1].rgbs(), clouds[2].rgbs();
  EXPECT_TRUE((merged.rgbs().array() == rgbs_expected.array()).all());

  Eigen::Matrix3Xf normals_expected(3, 6);
  normals_expected << clouds[0].normals(), clouds[1].normals(),
      clouds[2].normals();
  EXPECT_TRUE(CompareMatrices(merged.normals(), normals_expected));

  Eigen::RowVectorXf descriptors_expected(6);
  descriptors_expected << clouds[0].descriptors(), clouds[1].descriptors(),
      clouds[2].descriptors();
  EXPECT_TRUE(CompareMatrices(merged.descriptors(), descriptors_expected));
}

GTEST_TEST(PointCloudTest, VoxelizedDownSample) {
  const auto fields = pc_flags::kXYZs | pc_flags::kNormals |
                          pc_flags::kRGBs | pc_flags::kDescriptorCurvature;
  constexpr int num_points{6};
  PointCloud cloud(num_points, fields);
  // Place points inside the cube with corners at (±1, ±1, ±1).
  // clang-format off
  cloud.mutable_xyzs().transpose() <<
    -1, -1, -1,  // lower_xyz
    0.1, 0.2, 0.3,
    0.24, 0.1, 0.25,
    -.63, 0.25, .64,
    -.57, 0.73, .92,
    -.39, 0.2, .18;
  // clang-format on

  // Populate the other fields with random values.
  std::srand(1234);
  cloud.mutable_normals().setRandom();
  cloud.mutable_rgbs().setRandom();
  cloud.mutable_descriptors().setRandom();

  // Down-sample so that each occupied octant returns one point.
  PointCloud down_sampled = cloud.VoxelizedDownSample(1.0);
  EXPECT_EQ(down_sampled.size(), 3);

  auto CheckHasPointAveragedFrom =
      [&cloud, &down_sampled](const std::vector<int>& indices) {
        // Use doubles for accumulators.
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        Eigen::Vector3d rgb = Eigen::Vector3d::Zero();
        Vector1<double> descriptor = Vector1<double>::Zero();
        for (int i : indices) {
          xyz += cloud.xyz(i).cast<double>();
          normal += cloud.normal(i).cast<double>();
          rgb += cloud.rgb(i).cast<double>();
          descriptor += cloud.descriptor(i).cast<double>();
        }
        xyz /= indices.size();
        normal /= indices.size();
        rgb /= indices.size();
        descriptor /= indices.size();

        // Just do a linear search for the matching point.
        // TODO(russt): Could replace this with a nearest neighbor query if/when
        // it is provided.
        double kTol = 1e-8;
        bool found_match = false;
        int i = 0;
        for (; i < down_sampled.size(); ++i) {
          if (CompareMatrices(down_sampled.xyz(i), xyz.cast<float>(), kTol)) {
            found_match = true;
            break;
          }
        }
        ASSERT_TRUE(found_match);

        EXPECT_TRUE(CompareMatrices(down_sampled.normal(i),
                                    normal.cast<float>(), kTol));
        EXPECT_EQ(down_sampled.rgb(i), rgb.cast<u_int8_t>());
        EXPECT_TRUE(CompareMatrices(down_sampled.descriptor(i),
                                    descriptor.cast<float>(), kTol));
      };

  CheckHasPointAveragedFrom({0});
  CheckHasPointAveragedFrom({1, 2});
  CheckHasPointAveragedFrom({3, 4, 5});

  // Check that Infs/NaNs are ignored.
  cloud.mutable_xyz(3)[0] = std::numeric_limits<float>::quiet_NaN();
  cloud.mutable_xyz(4)[1] = std::numeric_limits<float>::infinity();

  down_sampled = cloud.VoxelizedDownSample(1.0);
  EXPECT_EQ(down_sampled.size(), 3);
  CheckHasPointAveragedFrom({5});
}

// Checks that normal has unit magnitude and that normal == expected up to a
// sign flip.
void CheckNormal(const Eigen::Ref<const Eigen::Vector3f>& normal,
                 const Eigen::Ref<const Eigen::Vector3f>& expected,
                 double kTolerance) {
  EXPECT_NEAR(normal.norm(), 1.0, kTolerance)
      << "Normal " << normal << " does not have unit length.";
  EXPECT_NEAR(std::abs(normal.dot(expected)), 1.0, kTolerance)
      << "normal.dot(expected) = " << normal.dot(expected);
}

// Test that point clouds that consistent of three points (defining a plane)
// get estimated normals that match their closed-form solution.
GTEST_TEST(PointCloudTest, EstimateNormalsPlane) {
  PointCloud cloud(3);

  cloud.mutable_xyzs().transpose() << 0, 0, 0, 0, 0, 1, 0, 1, 1;

  EXPECT_FALSE(cloud.has_normals());
  cloud.EstimateNormals(10, 3);
  EXPECT_TRUE(cloud.has_normals());

  double kTol = 1e-6;
  for (int i = 0; i < 3; ++i) {
    CheckNormal(cloud.normal(i), Vector3f{1, 0, 0}, kTol);
  }

  cloud.mutable_xyzs().transpose() << 0, 0, 0, 1, 0, 0, 1, 0, 1;

  cloud.EstimateNormals(10, 3);
  for (int i = 0; i < 3; ++i) {
    CheckNormal(cloud.normal(i), Vector3f{0, 1, 0}, kTol);
  }

  cloud.mutable_xyzs().transpose() << 0, 0, 0, 1, 0, 0, 0, 1, 1;
  cloud.EstimateNormals(10, 3);
  for (int i = 0; i < 3; ++i) {
    CheckNormal(cloud.normal(i),
                Vector3f{0, 1.0 / std::sqrt(2.0), -1.0 / std::sqrt(2.0)}, kTol);
  }
}

// Test that points uniformly randomly distributed on the surface of the sphere
// get normals close to the true normal of the sphere.
GTEST_TEST(PointCloudTest, EstimateNormalsSphere) {
  const int kSize{10000};
  PointCloud cloud(kSize);

  // Sample from a Gaussian and project it onto the sphere.
  RandomGenerator generator(1234);
  std::normal_distribution<double> distribution(0, 1.0);
  for (int i = 0; i < 3 * kSize; ++i) {
    cloud.mutable_xyzs().data()[i] = distribution(generator);
  }
  for (int i = 0; i < kSize; ++i) {
    cloud.mutable_xyz(i).normalize();
  }

  cloud.EstimateNormals(0.1, 30);

  double kTol = 1e-3;  // This will be loose unless kSize gets very large.
  for (int i = 0; i < kSize; ++i) {
    CheckNormal(cloud.normal(i), cloud.xyz(i), kTol);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
