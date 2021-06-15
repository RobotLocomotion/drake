#include "drake/perception/depth_image_to_point_cloud.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/camera_info.h"

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::Image;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageTraits;
using drake::systems::sensors::PixelType;
using Eigen::Vector3d;
using Eigen::Vector3f;

namespace drake {
namespace perception {
namespace {

constexpr float kFloatInf = std::numeric_limits<float>::infinity();
constexpr float kFloatNaN = std::numeric_limits<float>::quiet_NaN();

// We will run all test cases eight times -- the cartesian product of using both
// pixel types:
//  - uint16_t
//  - float
// both API flavors:
//  - System input & output ports
//  - static methods
// and with and without color images
struct System16 {
  static constexpr PixelType kPixelType = PixelType::kDepth16U;
  static constexpr bool kUseSystem = true;
  static constexpr pc_flags::BaseFieldT kFields = pc_flags::kXYZs;
};
struct Static16 {
  static constexpr PixelType kPixelType = PixelType::kDepth16U;
  static constexpr bool kUseSystem = false;
  static constexpr pc_flags::BaseFieldT kFields = pc_flags::kXYZs;
};
struct System32 {
  static constexpr PixelType kPixelType = PixelType::kDepth32F;
  static constexpr bool kUseSystem = true;
  static constexpr pc_flags::BaseFieldT kFields = pc_flags::kXYZs;
};
struct Static32 {
  static constexpr PixelType kPixelType = PixelType::kDepth32F;
  static constexpr bool kUseSystem = false;
  static constexpr pc_flags::BaseFieldT kFields = pc_flags::kXYZs;
};
struct System16Color {
  static constexpr PixelType kPixelType = PixelType::kDepth16U;
  static constexpr bool kUseSystem = true;
  static constexpr pc_flags::BaseFieldT kFields =
      pc_flags::kXYZs | pc_flags::kRGBs;
};
struct Static16Color {
  static constexpr PixelType kPixelType = PixelType::kDepth16U;
  static constexpr bool kUseSystem = false;
  static constexpr pc_flags::BaseFieldT kFields =
      pc_flags::kXYZs | pc_flags::kRGBs;
};
struct System32Color {
  static constexpr PixelType kPixelType = PixelType::kDepth32F;
  static constexpr bool kUseSystem = true;
  static constexpr pc_flags::BaseFieldT kFields =
      pc_flags::kXYZs | pc_flags::kRGBs;
};
struct Static32Color {
  static constexpr PixelType kPixelType = PixelType::kDepth32F;
  static constexpr bool kUseSystem = false;
  static constexpr pc_flags::BaseFieldT kFields =
      pc_flags::kXYZs | pc_flags::kRGBs;
};
using AllConfigs =
    ::testing::Types<System16, Static16, System32, Static32, System16Color,
                     Static16Color, System32Color, Static32Color>;

// Fixture that runs all test cases for all combinations.
template <typename Config>
class DepthImageToPointCloudTest : public ::testing::Test {
 public:
  static constexpr PixelType kConfiguredPixelType = Config::kPixelType;
  static constexpr bool kUseSystem = Config::kUseSystem;
  static constexpr pc_flags::BaseFieldT kFields = Config::kFields;
  using ConfiguredImage = Image<kConfiguredPixelType>;
  using ConfiguredImageTraits = ImageTraits<kConfiguredPixelType>;
  using Pixel = typename ConfiguredImageTraits::ChannelType;

 protected:
  static ConfiguredImage MakeDepthImage(const MatrixX<Pixel>& matrix) {
    // Convert the input Matrix into an Image.
    ConfiguredImage image(matrix.rows(), matrix.cols());
    for (int v = 0; v < image.height(); ++v) {
      for (int u = 0; u < image.width(); ++u) {
        *image.at(u, v) = matrix(u, v);
      }
    }
    return image;
  }

  static ImageRgba8U MakeRgbaImage(const int width, const int height,
                                   const int red_value, const int green_value,
                                   const int blue_value) {
    // Create an image where every pixel is the same color.  Note that the alpha
    // channel is not used since PointClouds don't yet support alphas.
    ImageRgba8U image(width, height);
    for (int v = 0; v < image.height(); ++v) {
      for (int u = 0; u < image.width(); ++u) {
        image.at(u, v)[0] = static_cast<uint8_t>(red_value);
        image.at(u, v)[1] = static_cast<uint8_t>(green_value);
        image.at(u, v)[2] = static_cast<uint8_t>(blue_value);
      }
    }
    return image;
  }

  static bool CompareClouds(const PointCloud& result_cloud,
                            const PointCloud& expected_cloud,
                            const float xyz_tol = 1e-16) {
    bool equal = true;

    equal = equal && CompareMatrices(result_cloud.xyzs(), expected_cloud.xyzs(),
                                     xyz_tol);

    equal = equal && (result_cloud.has_rgbs() == expected_cloud.has_rgbs());

    if (result_cloud.has_rgbs()) {
      equal = equal && (result_cloud.rgbs() == expected_cloud.rgbs());
    }

    return equal;
  }

  static PointCloud DoConvert(
      const CameraInfo& camera_info,
      const std::optional<RigidTransformd>& camera_pose,
      const MatrixX<Pixel>& depth_image_matrix,
      const std::optional<systems::sensors::ImageRgba8U>& color_image,
      const std::optional<float>& scale) {
    const auto depth_image = MakeDepthImage(depth_image_matrix);

    // Call the DUT to convert Image to PointCloud.

    if (kUseSystem) {
      PointCloud result(0, kFields);
      const DepthImageToPointCloud dut(camera_info, kConfiguredPixelType,
                                       scale.value_or(1.0), kFields);
      auto context = dut.CreateDefaultContext();
      dut.get_input_port(0).FixValue(context.get(), depth_image);
      if (kFields & pc_flags::kRGBs) {
        dut.get_input_port(1).FixValue(context.get(), color_image.value());
      }
      if (camera_pose) {
        dut.get_input_port(2).FixValue(context.get(), *camera_pose);
      }
      return dut.get_output_port(0).Eval<PointCloud>(*context);
    } else {
      PointCloud result(0, kFields);
      if (kFields & pc_flags::kRGBs) {
        DepthImageToPointCloud::Convert(camera_info, camera_pose, depth_image,
                                        color_image, scale, &result);
      } else {
        DepthImageToPointCloud::Convert(camera_info, camera_pose, depth_image,
                                        std::nullopt, scale, &result);
      }
      return result;
    }
  }

  static void DoConvert(
      const CameraInfo& camera_info,
      const std::optional<RigidTransformd>& camera_pose,
      const MatrixX<Pixel>& depth_image_matrix,
      const std::optional<systems::sensors::ImageRgba8U>& color_image,
      const std::optional<float>& scale, AbstractValue* cloud) {
    const auto depth_image = MakeDepthImage(depth_image_matrix);

    // Call the DUT to convert Image to PointCloud.
    if (kUseSystem) {
      const DepthImageToPointCloud dut(camera_info, kConfiguredPixelType,
                                       scale.value_or(1.0), kFields);
      auto context = dut.CreateDefaultContext();
      dut.get_input_port(0).FixValue(context.get(), depth_image);
      if (kFields & pc_flags::kRGBs) {
        dut.get_input_port(1).FixValue(context.get(), color_image.value());
      }
      if (camera_pose) {
        dut.get_input_port(2).FixValue(context.get(), *camera_pose);
      }
      dut.get_output_port(0).Calc(*context, cloud);
    } else {
      if (kFields & pc_flags::kRGBs) {
        DepthImageToPointCloud::Convert(
            camera_info, camera_pose, depth_image, color_image, scale,
            &(cloud->get_mutable_value<PointCloud>()));
      } else {
        DepthImageToPointCloud::Convert(
            camera_info, camera_pose, depth_image, std::nullopt, scale,
            &(cloud->get_mutable_value<PointCloud>()));
      }
    }
  }

  const CameraInfo single_pixel_camera_{1, 1, 1.0, 1.0, 0.5, 0.5};
  const RigidTransformd random_transform_{RollPitchYawd(0.1, -0.2, 0.3),
                                          Vector3d(1.1, -1.2, 1.3)};
  const RigidTransformd z_translation_{Vector3d(0.0, 0.0, 1.3)};
};
TYPED_TEST_SUITE(DepthImageToPointCloudTest, AllConfigs);

// Verifies computed point cloud when pixel values are valid.
TYPED_TEST(DepthImageToPointCloudTest, Basic) {
  using TestFixturePixel = typename TestFixture::Pixel;

  static constexpr int kImageWidth = 60;
  static constexpr int kImageHeight = 40;
  static constexpr float kFocal = 500.0f;

  // Camera settings.
  const CameraInfo camera(kImageWidth, kImageHeight, kFocal, kFocal,
                          kImageWidth * 0.5, kImageHeight * 0.5);

  // A constant-value depth image.
  constexpr TestFixturePixel kDepthValue = 1;
  const auto& depth_image = MatrixX<TestFixturePixel>::Constant(
                                kImageWidth, kImageHeight, kDepthValue)
                                .eval();

  // A constant-value RGB image.
  constexpr uint8_t kRedValue = 0;
  constexpr uint8_t kGreenValue = 127;
  constexpr uint8_t kBlueValue = 255;
  const auto& color_image = this->MakeRgbaImage(
      kImageWidth, kImageHeight, kRedValue, kGreenValue, kBlueValue);

  // The expected resulting point cloud.
  PointCloud expected_cloud(kImageWidth * kImageHeight, TestFixture::kFields);
  for (int v = 0; v < depth_image.cols(); ++v) {
    for (int u = 0; u < depth_image.rows(); ++u) {
      const int i = v * depth_image.rows() + u;
      const double expected_x = kDepthValue * (u - kImageWidth * 0.5) / kFocal;
      const double expected_y = kDepthValue * (v - kImageHeight * 0.5) / kFocal;
      expected_cloud.mutable_xyzs().col(i) =
          Vector3f(expected_x, expected_y, kDepthValue);

      if (TestFixture::kFields & pc_flags::kRGBs) {
        expected_cloud.mutable_rgbs().col(i) =
            Vector3<uint8_t>(kRedValue, kGreenValue, kBlueValue);
      }
    }
  }

  // This tolerance was determined empirically for Drake's supported platforms.
  constexpr float kDistanceTolerance = 1e-8;

  PointCloud result(0, TestFixture::kFields);

  // Without a pose offset nor a scale factor.
  result = this->DoConvert(camera, std::nullopt, depth_image, color_image,
                           std::nullopt);
  EXPECT_TRUE(
      TestFixture::CompareClouds(result, expected_cloud, kDistanceTolerance));

  // Now with scale factor.
  result = this->DoConvert(camera, std::nullopt, depth_image, color_image,
                           0.001);
  if (TestFixture::kFields & pc_flags::kRGBs) {
    EXPECT_EQ(result.rgbs(), expected_cloud.rgbs());
  }
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud.xyzs() * 0.001,
                              kDistanceTolerance));

  // Now with a pose offset -- just check the z values.
  const auto& pose = this->z_translation_;
  const auto& expected_z_row = expected_cloud.xyzs().row(2).array().eval();
  result = this->DoConvert(camera, pose, depth_image, color_image,
                           std::nullopt);
  if (TestFixture::kFields & pc_flags::kRGBs) {
    EXPECT_EQ(result.rgbs(), expected_cloud.rgbs());
  }
  EXPECT_TRUE(CompareMatrices(
      result.xyzs().row(2), (expected_z_row + pose.translation().z()).matrix(),
      kDistanceTolerance));

  // With both scale and pose offset -- just check the z values.
  result = this->DoConvert(camera, pose, depth_image, color_image, 0.001);
  if (TestFixture::kFields & pc_flags::kRGBs) {
    EXPECT_EQ(result.rgbs(), expected_cloud.rgbs());
  }
  EXPECT_TRUE(CompareMatrices(
      result.xyzs().row(2),
      (expected_z_row * 0.001 + pose.translation().z()).matrix(),
      kDistanceTolerance));
}

// Verifies computed point cloud when pixel value is NaN.
TYPED_TEST(DepthImageToPointCloudTest, NanValue) {
  using TestFixturePixel = typename TestFixture::Pixel;

  // This test only applies for 32F images; there is no NaN for 16U images.
  constexpr bool is_meaningful_nan =
      !std::is_same_v<TestFixturePixel, uint16_t>;
  if (!is_meaningful_nan) {
    return;
  }
  // Use the ?: operator to avoid a -Woverflow warning.  (The exception case is
  // never reached, but mitigates the warning.)
  const TestFixturePixel single_pixel =
      is_meaningful_nan ? kFloatNaN : throw std::runtime_error("");

  // Test all combinations of {without pose, with pose} x
  // {without scale, with scale}.
  const auto& camera = this->single_pixel_camera_;
  const auto& pose = this->random_transform_;
  const auto& depth_image =
      Vector1<TestFixturePixel>::Constant(single_pixel).eval();

  // A single-pixel RGB image.
  const uint8_t kRedValue = 234;
  const uint8_t kGreenValue = 0;
  const uint8_t kBlueValue = 58;
  const auto& color_image =
      this->MakeRgbaImage(1, 1, kRedValue, kGreenValue, kBlueValue);

  PointCloud expected_cloud(1, TestFixture::kFields);
  expected_cloud.mutable_xyzs() = Vector3f::Constant(kFloatNaN).eval();
  if (TestFixture::kFields & pc_flags::kRGBs) {
    expected_cloud.mutable_rgbs() =
        Vector3<uint8_t>(kRedValue, kGreenValue, kBlueValue);
  }

  PointCloud result(0, TestFixture::kFields);

  result = this->DoConvert(camera, std::nullopt, depth_image, color_image,
                           std::nullopt);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, std::nullopt, depth_image, color_image, 0.1);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, pose, depth_image, color_image,
                           std::nullopt);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, pose, depth_image, color_image, 0.1);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));
}

// Verifies computed point cloud when pixel value is kTooClose or kTooFar.
TYPED_TEST(DepthImageToPointCloudTest, TooNearFar) {
  using TestFixturePixel = typename TestFixture::Pixel;
  using Traits = typename TestFixture::ConfiguredImageTraits;
  const auto& camera = this->single_pixel_camera_;
  const auto& pose = this->random_transform_;
  const auto& depth_image_near =
      Vector1<TestFixturePixel>::Constant(Traits::kTooClose).eval();
  const auto& depth_image_far =
      Vector1<TestFixturePixel>::Constant(Traits::kTooFar).eval();

  // A single-pixel RGB image.
  const uint8_t kRedValue = 200;
  const uint8_t kGreenValue = 100;
  const uint8_t kBlueValue = 50;
  const auto& color_image =
      this->MakeRgbaImage(1, 1, kRedValue, kGreenValue, kBlueValue);

  PointCloud expected_cloud(1, TestFixture::kFields);
  expected_cloud.mutable_xyzs() = Vector3f::Constant(kFloatInf).eval();
  if (TestFixture::kFields & pc_flags::kRGBs) {
    expected_cloud.mutable_rgbs() =
        Vector3<uint8_t>(kRedValue, kGreenValue, kBlueValue);
  }

  PointCloud result(0, TestFixture::kFields);

  // Test all combinations of {without pose, with pose} x {near, far} x
  // {without scale, with scale}.
  result =
      this->DoConvert(camera, std::nullopt, depth_image_near, color_image,
                      std::nullopt);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, std::nullopt, depth_image_near, color_image,
                           0.1);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result =
      this->DoConvert(camera, std::nullopt, depth_image_far, color_image,
                      std::nullopt);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, std::nullopt, depth_image_far, color_image,
                           0.1);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result =
      this->DoConvert(camera, pose, depth_image_near, color_image,
                      std::nullopt);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, pose, depth_image_near, color_image, 0.1);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, pose, depth_image_far, color_image,
                           std::nullopt);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));

  result = this->DoConvert(camera, pose, depth_image_far, color_image, 0.1);
  EXPECT_TRUE(TestFixture::CompareClouds(result, expected_cloud));
}

// Verifies the System method CalcOutput resets invalid storage.
TYPED_TEST(DepthImageToPointCloudTest, ResetStorage) {
  using TestFixturePixel = typename TestFixture::Pixel;
  const auto& camera = this->single_pixel_camera_;
  const auto& depth_image = Vector1<TestFixturePixel>::Constant(1).eval();
  std::unique_ptr<AbstractValue> abstract_value;
  const PointCloud* cloud{};

  // A single-pixel RGB image.
  const uint8_t kRedValue = 200;
  const uint8_t kGreenValue = 100;
  const uint8_t kBlueValue = 50;
  Matrix3X<uint8_t> expected_colors(3, 1);
  expected_colors << kRedValue, kGreenValue, kBlueValue;
  const auto& color_image =
      this->MakeRgbaImage(1, 1, kRedValue, kGreenValue, kBlueValue);

  PointCloud expected_cloud(1, TestFixture::kFields);
  expected_cloud.mutable_xyzs() = Vector3f(-0.5, -0.5, 1);
  if (TestFixture::kFields & pc_flags::kRGBs) {
    expected_cloud.mutable_rgbs() =
        Vector3<uint8_t>(kRedValue, kGreenValue, kBlueValue);
  }

  // N.B. A local variable is made to avoid linker errors.
  const pc_flags::Fields fields = TestFixture::kFields;

  // Starting from default storage, check the point cloud result.
  abstract_value = std::make_unique<Value<PointCloud>>(0, fields);
  this->DoConvert(camera, std::nullopt, depth_image, color_image, std::nullopt,
                  abstract_value.get());
  cloud = &(abstract_value->get_value<PointCloud>());
  EXPECT_EQ(cloud->fields(), fields);
  EXPECT_TRUE(TestFixture::CompareClouds(*cloud, expected_cloud));

  // If the storage was the wrong size, then Calc should be able to resize it.
  abstract_value = std::make_unique<Value<PointCloud>>(22, fields);
  this->DoConvert(camera, std::nullopt, depth_image, color_image, std::nullopt,
                  abstract_value.get());
  cloud = &(abstract_value->get_value<PointCloud>());
  EXPECT_EQ(cloud->fields(), fields);
  EXPECT_TRUE(TestFixture::CompareClouds(*cloud, expected_cloud));

  // If the storage had additional channels, we can't recover when its a System.
  if (TestFixture::kUseSystem) {
    // If the output has the wrong set of channels, currently the DUT throws an
    // exception because the PointCloud offers no way to change channels after
    // construction.  (Ideally, the dut would produce the expected output by
    // resetting the set of channels, in which case this test should change.)
    abstract_value = std::make_unique<Value<PointCloud>>(
        1, pc_flags::kXYZs | pc_flags::kNormals);
    EXPECT_THROW(this->DoConvert(camera, std::nullopt, depth_image, color_image,
                                 std::nullopt, abstract_value.get()),
                 std::exception);
  } else {
    abstract_value = std::make_unique<Value<PointCloud>>(1, fields);
    this->DoConvert(camera, std::nullopt, depth_image, color_image,
                    std::nullopt, abstract_value.get());
    cloud = &(abstract_value->get_value<PointCloud>());
    EXPECT_EQ(cloud->fields(), fields);
    EXPECT_TRUE(TestFixture::CompareClouds(*cloud, expected_cloud));
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
