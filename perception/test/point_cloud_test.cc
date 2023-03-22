#include "drake/perception/point_cloud.h"

#include <stdexcept>

#include <common_robotics_utilities/openmp_helpers.hpp>
#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

using Eigen::Matrix3Xf;
using Eigen::RowVectorXf;
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

void CompareClouds(const PointCloud& cloud_1, const PointCloud& cloud_2) {
  EXPECT_TRUE(cloud_1.fields() == cloud_2.fields());

  pc_flags::Fields fields_to_compare = cloud_1.fields();
  if (fields_to_compare.contains(pc_flags::kXYZs)) {
    EXPECT_TRUE(CompareMatrices(cloud_1.xyzs(), cloud_2.xyzs()));
  }
  if (fields_to_compare.contains(pc_flags::kRGBs)) {
    EXPECT_TRUE((cloud_1.rgbs().array() == cloud_2.rgbs().array()).all());
  }
  if (fields_to_compare.contains(pc_flags::kNormals)) {
    EXPECT_TRUE(CompareMatrices(cloud_1.normals(), cloud_2.normals()));
  }
  if (fields_to_compare.has_descriptor()) {
    EXPECT_TRUE(CompareMatrices(cloud_1.descriptors(), cloud_2.descriptors()));
  }
}

GTEST_TEST(PointCloudTest, TestExpectedNumThreads) {
#if defined(_OPENMP)
  constexpr bool has_openmp = true;
#else
  constexpr bool has_openmp = false;
#endif

  const int num_omp_threads =
      common_robotics_utilities::openmp_helpers::GetNumOmpThreads();

  if (has_openmp && ENABLE_PARALLEL_OPS) {
    // The build file specifies OMP_NUM_THREADS=2 for the parallel test.
    EXPECT_EQ(num_omp_threads, 2);
  } else {
    EXPECT_EQ(num_omp_threads, 1);
  }
}

GTEST_TEST(PointCloudTest, Basic) {
  const int count = 5;

  // Declare default values for each field to facilitate testing.
  Matrix3Xf xyzs_expected(3, count);
  xyzs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  Matrix3Xf normals_expected(3, count);
  normals_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  Matrix3X<uint8_t> rgbs_expected(3, count);
  rgbs_expected.transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 255,
    4, 5, 6,
    40, 50, 60;
  RowVectorXf descriptors_expected(count);
  descriptors_expected << 1, 2, 3, 4, 5;

  // Create a point cloud with default values.
  auto CreatePointCloud = [&](pc_flags::Fields fields) {
    PointCloud cloud(count, fields);
    if (fields.contains(pc_flags::kXYZs)) {
      cloud.mutable_xyzs() = xyzs_expected;
    }
    if (fields.contains(pc_flags::kNormals)) {
      cloud.mutable_rgbs() = rgbs_expected;
    }
    if (fields.contains(pc_flags::kRGBs)) {
      cloud.mutable_normals() = normals_expected;
    }
    if (fields.has_descriptor()) {
      cloud.mutable_descriptors() = descriptors_expected;
    }
    return cloud;
  };

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

  // XYZs.
  CheckFields(xyzs_expected, pc_flags::kXYZs,
              [](PointCloud& cloud) { return cloud.mutable_xyzs(); },
              [](PointCloud& cloud) { return cloud.xyzs(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_xyz(i); },
              [](PointCloud& cloud, int i) { return cloud.xyz(i); });

  // Normals.
  CheckFields(normals_expected, pc_flags::kNormals,
              [](PointCloud& cloud) { return cloud.mutable_normals(); },
              [](PointCloud& cloud) { return cloud.normals(); },
              [](PointCloud& cloud, int i) { return cloud.mutable_normal(i); },
              [](PointCloud& cloud, int i) { return cloud.normal(i); });

  // RGBs.
  CheckFields(rgbs_expected, pc_flags::kRGBs,
            [](PointCloud& cloud) { return cloud.mutable_rgbs(); },
            [](PointCloud& cloud) { return cloud.rgbs(); },
            [](PointCloud& cloud, int i) { return cloud.mutable_rgb(i); },
            [](PointCloud& cloud, int i) { return cloud.rgb(i); });

  // Descriptors (Curvature).
  CheckFields(descriptors_expected, pc_flags::kDescriptorCurvature,
              [](PointCloud& cloud) { return cloud.mutable_descriptors(); },
              [](PointCloud& cloud) { return cloud.descriptors(); },
              [](PointCloud& cloud, int i) {
                return cloud.mutable_descriptor(i);
              },
              [](PointCloud& cloud, int i) { return cloud.descriptor(i); });

  // Test operator= between two clouds with different fields.
  {
    pc_flags::Fields xyz_rgb_normals =
        (pc_flags::kXYZs | pc_flags::kNormals | pc_flags::kRGBs);
    pc_flags::Fields all_fields =
        (xyz_rgb_normals | pc_flags::kDescriptorCurvature);

    const std::vector<std::pair<pc_flags::Fields, pc_flags::Fields>>
        fields_pairs{{pc_flags::kXYZs, xyz_rgb_normals},
                     {xyz_rgb_normals, pc_flags::kXYZs},
                     {pc_flags::kXYZs, all_fields},
                     {all_fields, pc_flags::kXYZs}};

    for (const auto& [fields_to, fields_from] : fields_pairs) {
      PointCloud cloud_to = CreatePointCloud(fields_to);
      PointCloud cloud_from = CreatePointCloud(fields_from);

      cloud_to = cloud_from;
      CompareClouds(cloud_to, cloud_from);

      PointCloud cloud_move_to = CreatePointCloud(fields_to);
      cloud_move_to = std::move(cloud_from);
      CompareClouds(cloud_to, cloud_move_to);
    }
  }

  // Test `SetFields()` and its `skip_initialize` flag.
  {
    PointCloud cloud = CreatePointCloud(pc_flags::kXYZs);
    EXPECT_TRUE(cloud.HasFields(pc_flags::kXYZs));
    EXPECT_FALSE(cloud.HasFields(pc_flags::kNormals));

    // Expand the fields.
    cloud.SetFields((pc_flags::kXYZs | pc_flags::kNormals | pc_flags::kRGBs |
                     pc_flags::kDescriptorCurvature),
                    false);
    EXPECT_TRUE(cloud.HasFields(pc_flags::kXYZs));
    EXPECT_TRUE(cloud.HasFields(pc_flags::kNormals));
    // Check the retained container is unchanged.
    EXPECT_TRUE(CompareMatrices(cloud.xyzs(), xyzs_expected));
    // Check the new containers are default-initialized.
    EXPECT_TRUE((cloud.rgbs().array() == PointCloud::kDefaultColor).all());
    EXPECT_TRUE((cloud.normals().array().isNaN()).all());
    EXPECT_TRUE((cloud.descriptors().array().isNaN()).all());

    // Set some values for the descriptor container.
    cloud.mutable_descriptors() = descriptors_expected;
    // Shrink the fields with a different descriptor. Also, skip initialization.
    cloud.SetFields((pc_flags::kRGBs | pc_flags::kDescriptorFPFH), true);
    EXPECT_TRUE(cloud.HasFields(pc_flags::kRGBs));
    EXPECT_FALSE(cloud.HasFields(pc_flags::kXYZs));
    // The rows size should be consistent with the new FPFH descriptor.
    EXPECT_EQ(cloud.descriptors().rows(), 33);
    EXPECT_FALSE((cloud.descriptors().array().isNaN()).all());
  }

  // Test point cloud cropping.
  {
    PointCloud cloud =
        CreatePointCloud(pc_flags::kXYZs | pc_flags::kNormals |
                         pc_flags::kRGBs | pc_flags::kDescriptorCurvature);
    PointCloud cropped = cloud.Crop(Vector3f{4, 5, 6}, Vector3f{10, 20, 30});
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
    EXPECT_THROW(PointCloud(1, pc_flags::kNone), std::runtime_error);
    EXPECT_THROW(PointCloud(1, pc_flags::kDescriptorNone),
                 std::runtime_error);
  }
}

GTEST_TEST(PointCloud, Concatenate) {
  // Run with a few subsets of the possible fields.
  std::vector<pc_flags::Fields> fields;
  fields.push_back(pc_flags::kXYZs);
  fields.push_back(pc_flags::kRGBs);
  fields.push_back(pc_flags::kNormals);
  fields.push_back(pc_flags::kDescriptorCurvature);
  fields.push_back(pc_flags::kXYZs | pc_flags::kRGBs | pc_flags::kNormals |
                   pc_flags::kDescriptorCurvature);

  for (const auto& f : fields) {
    std::vector<PointCloud> clouds;
    // Populate three point clouds with arbitrary values.
    clouds.push_back(PointCloud(2, f, true));
    clouds.push_back(PointCloud(3, f, true));
    clouds.push_back(PointCloud(1, f, true));

    if (f.contains(pc_flags::kXYZs)) {
      clouds[0].mutable_xyzs() << 1, 2, 3, 4, 5, 6;
      clouds[1].mutable_xyzs() << 421, 1, 1, 4, 4, 2, 141, 1, 63;
      clouds[2].mutable_xyzs() << 71, 2, 1;
    }

    if (f.contains(pc_flags::kRGBs)) {
      clouds[0].mutable_rgbs() << 10, 20, 30, 40, 50, 60;
      clouds[1].mutable_rgbs() << 57, 1, 42, 25, 25, 10, 12, 63, 192;
      clouds[2].mutable_rgbs() << 53, 24, 172;
    }

    if (f.contains(pc_flags::kNormals)) {
      clouds[0].mutable_normals() << 3, 4, -1, 12, 4, 32;
      clouds[1].mutable_normals() << -2, 1, 532, 7, 2, 6, 36, 84, 42;
      clouds[2].mutable_normals() << 91, 4, 44;
    }

    if (f.has_descriptor()) {
      clouds[0].mutable_descriptors() << 8, 2;
      clouds[1].mutable_descriptors() << 10, 52, 1;
      clouds[2].mutable_descriptors() << 91;
    }

    PointCloud merged = Concatenate(clouds);
    EXPECT_EQ(merged.size(), 6);

    if (f.contains(pc_flags::kXYZs)) {
      Matrix3Xf xyzs_expected(3, 6);
      xyzs_expected << clouds[0].xyzs(), clouds[1].xyzs(), clouds[2].xyzs();
      EXPECT_TRUE(CompareMatrices(merged.xyzs(), xyzs_expected));
    }

    if (f.contains(pc_flags::kRGBs)) {
      Matrix3X<uint8_t> rgbs_expected(3, 6);
      rgbs_expected << clouds[0].rgbs(), clouds[1].rgbs(), clouds[2].rgbs();
      EXPECT_TRUE((merged.rgbs().array() == rgbs_expected.array()).all());
    }

    if (f.contains(pc_flags::kNormals)) {
      Matrix3Xf normals_expected(3, 6);
      normals_expected << clouds[0].normals(), clouds[1].normals(),
          clouds[2].normals();
      EXPECT_TRUE(CompareMatrices(merged.normals(), normals_expected));
    }

    if (f.has_descriptor()) {
      RowVectorXf descriptors_expected(6);
      descriptors_expected << clouds[0].descriptors(), clouds[1].descriptors(),
          clouds[2].descriptors();
      EXPECT_TRUE(CompareMatrices(merged.descriptors(), descriptors_expected));
    }
  }
}

GTEST_TEST(PointCloudTest, FlipNormals) {
  const auto fields = pc_flags::kXYZs | pc_flags::kNormals;
  constexpr int num_points{7};
  PointCloud cloud(num_points, fields);

  constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
  // Place the xyzs in a square on the xy axis, plus a test for nan and inf.
  // clang-format off
  cloud.mutable_xyzs().transpose() <<
    0,    0,    0,
    0.1,  0,    0,
    0,    0.1,  0,
    0.1,  0.1,  0,
    0,    kNan, 0,
    0,    0.1,  0,  // will have a kNan in the normal
    0,    0,    0;  // will have an orthogonal normal
  // clang-format on

  // Original normals have (almost) arbitrary xy values, but an intentional mix
  // of positive and negative z values.  The z values are taken to be larger
  // than the x,y values so that the normals are vertical enough that the z
  // component determines if they should flip.
  Matrix3Xf original_normals(3, num_points);
  // clang-format off
  original_normals.transpose() <<
    0.12, 0.32,  1.0,
    -0.2, 0.24,  2.3,
    -3,   0.24, -2.5,
    .25,  0.47, -2.4,
    0.12, 0.32,  1.0,
    kNan, 0.47, -2.4,
    1,    1,     0;
  // clang-format on

  auto CheckNormal = [&cloud, &original_normals](int index,
                                                 bool expect_flipped) {
    if (expect_flipped) {
      EXPECT_TRUE(
          CompareMatrices(cloud.normal(index), -original_normals.col(index)));
    } else {
      EXPECT_TRUE(
          CompareMatrices(cloud.normal(index), original_normals.col(index)));
    }
  };

  // Orient toward positive z.
  cloud.mutable_normals() = original_normals;
  cloud.FlipNormalsTowardPoint(Vector3f{0, 0, 1});

  CheckNormal(0, false);
  CheckNormal(1, false);
  CheckNormal(2, true);
  CheckNormal(3, true);
  CheckNormal(4, false);  // NaN xyz doesn't explode.
  CheckNormal(5, false);  // NaN normal doesn't explode.
  CheckNormal(6, false);  // orthogonal normals don't flip.

  // Orient toward negative z.
  cloud.mutable_normals() = original_normals;
  cloud.FlipNormalsTowardPoint(Vector3f{0, 0, -1});

  CheckNormal(0, true);
  CheckNormal(1, true);
  CheckNormal(2, false);
  CheckNormal(3, false);
  CheckNormal(4, false);  // NaN xyz doesn't explode.
  CheckNormal(5, false);  // NaN normal doesn't explode.
  CheckNormal(6, false);  // orthogonal normals don't flip.
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
  PointCloud down_sampled = cloud.VoxelizedDownSample(1.0, ENABLE_PARALLEL_OPS);
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
        normal.normalize();
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
        EXPECT_NEAR(down_sampled.normal(i).norm(), 1.0, 1e-6);
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

  // Check that voxels with only NaN normals still return NaN.
  cloud.mutable_normal(0)[1] = std::numeric_limits<float>::quiet_NaN();

  down_sampled = cloud.VoxelizedDownSample(1.0, ENABLE_PARALLEL_OPS);
  EXPECT_EQ(down_sampled.size(), 3);
  CheckHasPointAveragedFrom({5});

  // Check the NaN normal obtained from cloud index 0.
  bool found_match_for_cloud_0 = false;
  for (int i = 0; i < down_sampled.size(); ++i) {
    if (down_sampled.xyz(i) == cloud.xyz(0)) {
      // Confirm that the normal is now [nan, nan, nan].
      EXPECT_TRUE(down_sampled.normal(i).array().isNaN().all());
      found_match_for_cloud_0 = true;
    }
  }
  EXPECT_TRUE(found_match_for_cloud_0);
}

// Checks that normal has unit magnitude and that normal == expected up to a
// sign flip.
void CheckNormal(const Eigen::Ref<const Vector3f>& normal,
                 const Eigen::Ref<const Vector3f>& expected, double tolerance) {
  EXPECT_NEAR(normal.norm(), 1.0, tolerance)
      << fmt::format("Normal {} does not have unit length.", fmt_eigen(normal));
  EXPECT_NEAR(std::abs(normal.dot(expected)), 1.0, tolerance)
      << "normal.dot(expected) = " << normal.dot(expected);
}

// Test that point clouds that consist of three points (defining a plane)
// get estimated normals that match their closed-form solution.
GTEST_TEST(PointCloudTest, EstimateNormalsPlane) {
  PointCloud cloud(3);

  cloud.mutable_xyzs().transpose() << 0, 0, 0, 0, 0, 1, 0, 1, 1;

  EXPECT_FALSE(cloud.has_normals());
  cloud.EstimateNormals(10, 3, ENABLE_PARALLEL_OPS);
  EXPECT_TRUE(cloud.has_normals());

  double kTol = 1e-6;
  for (int i = 0; i < 3; ++i) {
    CheckNormal(cloud.normal(i), Vector3f{1, 0, 0}, kTol);
  }

  cloud.mutable_xyzs().transpose() << 0, 0, 0, 1, 0, 0, 1, 0, 1;

  cloud.EstimateNormals(10, 3, ENABLE_PARALLEL_OPS);
  for (int i = 0; i < 3; ++i) {
    CheckNormal(cloud.normal(i), Vector3f{0, 1, 0}, kTol);
  }

  cloud.mutable_xyzs().transpose() << 0, 0, 0, 1, 0, 0, 0, 1, 1;
  cloud.EstimateNormals(10, 3, ENABLE_PARALLEL_OPS);
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

  cloud.EstimateNormals(0.1, 30, ENABLE_PARALLEL_OPS);

  double kTol = 1e-3;  // This will be loose unless kSize gets very large.
  for (int i = 0; i < kSize; ++i) {
    CheckNormal(cloud.normal(i), cloud.xyz(i), kTol);
  }
}

// Tests that you can compute normals with just two neighbors; the normal is
// ambiguous, but the computed normal will at least be orthogonal.
GTEST_TEST(PointCloudTest, EstimateNormalsTwoPoints) {
  PointCloud cloud(2);
  // clang-format off
  cloud.mutable_xyzs().transpose() <<
    0, 0, 0,
    1, 1, 0;
  // clang-format on
  cloud.EstimateNormals(10, 3, ENABLE_PARALLEL_OPS);

  double kTolerance = 1e-6;
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(cloud.normal(i).norm(), 1.0, kTolerance);
    EXPECT_NEAR(cloud.normal(i).dot(cloud.xyz(1) - cloud.xyz(0)), 0.0,
                kTolerance);
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
