#include "drake/perception/depth_image_to_point_cloud.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/camera_info.h"

using Eigen::Vector3d;
using Eigen::Vector3f;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::systems::AbstractValue;
using drake::systems::Value;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::Image;
using drake::systems::sensors::ImageTraits;
using drake::systems::sensors::PixelType;

namespace drake {
namespace perception {
namespace {

constexpr float kFloatInf = std::numeric_limits<float>::infinity();
constexpr float kFloatNaN = std::numeric_limits<float>::quiet_NaN();

// We will run all test cases four times -- the cartesian product of using both
// pixel types:
//  - uint16_t
//  - float
// and both API flavors:
//  - System input & output ports
//  - static methods
struct System16 {
  static constexpr PixelType kPixelType = PixelType::kDepth16U;
  static constexpr bool kUseSytem = true;
};
struct Static16 {
  static constexpr PixelType kPixelType = PixelType::kDepth16U;
  static constexpr bool kUseSytem = false;
};
struct System32 {
  static constexpr PixelType kPixelType = PixelType::kDepth32F;
  static constexpr bool kUseSytem = true;
};
struct Static32 {
  static constexpr PixelType kPixelType = PixelType::kDepth32F;
  static constexpr bool kUseSytem = false;
};
using AllConfigs = ::testing::Types<System16, Static16, System32, Static32>;

// Fixture that runs all test cases for all combinations.
template <typename Config>
class DepthImageToPointCloudTest : public ::testing::Test {
 public:
  static constexpr PixelType kConfiguredPixelType = Config::kPixelType;
  using ConfiguredImage = Image<kConfiguredPixelType>;
  using ConfiguredImageTraits = ImageTraits<kConfiguredPixelType>;
  using Pixel = typename ConfiguredImageTraits::ChannelType;

 protected:
  PointCloud DoConvert(
      const CameraInfo& camera_info,
      const optional<RigidTransformd>& camera_pose,
      const MatrixX<Pixel>& matrix,
      const optional<float>& scale) {
    // Convert the input Matrix into an Image.
    ConfiguredImage image(matrix.rows(), matrix.cols());
    for (int v = 0; v < image.height(); ++v) {
      for (int u = 0; u < image.width(); ++u) {
        *image.at(u, v) = matrix(u, v);
      }
    }
    const Value<ConfiguredImage> image_as_value(image);

    // Call the DUT to convert Image to PointCloud.
    PointCloud result;
    if (Config::kUseSytem) {
      const DepthImageToPointCloud dut(
          camera_info, kConfiguredPixelType, scale.value_or(1.0));
      auto context = dut.CreateDefaultContext();
      context->FixInputPort(0, image_as_value);
      if (camera_pose) {
        const Value<RigidTransformd> camera_pose_as_value(*camera_pose);
        context->FixInputPort(1, camera_pose_as_value);
      }
      result = dut.get_output_port(0).Eval<PointCloud>(*context);
    } else {
      DepthImageToPointCloud::Convert(
          camera_info, camera_pose, image, scale, &result);
    }
    return result;
  }

  const CameraInfo single_pixel_camera_{1, 1, 1.0};
  const RigidTransformd random_transform_{
    RollPitchYawd(0.1, -0.2, 0.3), Vector3d(1.1, -1.2, 1.3)};
  const RigidTransformd z_translation_{Vector3d(0.0, 0.0, 1.3)};
};
TYPED_TEST_CASE(DepthImageToPointCloudTest, AllConfigs);

// Verifies computed point cloud when pixel values are valid.
TYPED_TEST(DepthImageToPointCloudTest, Basic) {
  using Pixel = typename TestFixture::Pixel;

  static constexpr int kDepthWidth = 60;
  static constexpr int kDepthHeight = 40;
  static constexpr float kFocal = 500.0f;

  // Camera settings.
  const CameraInfo camera(
      kDepthWidth, kDepthHeight, kFocal, kFocal,
      kDepthWidth * 0.5, kDepthHeight * 0.5);

  // A constant-value depth image.
  constexpr Pixel kDepthValue = 1;
  const auto& image = MatrixX<Pixel>::Constant(
      kDepthWidth, kDepthHeight, kDepthValue).eval();

  // The expected resulting point cloud.
  PointCloud expected_cloud(kDepthWidth * kDepthHeight);
  for (int v = 0; v < image.cols(); ++v) {
    for (int u = 0; u < image.rows(); ++u) {
      const int i = v * image.rows() + u;
      const double expected_x = kDepthValue * (u - kDepthWidth * 0.5) / kFocal;
      const double expected_y = kDepthValue * (v - kDepthHeight * 0.5) / kFocal;
      expected_cloud.mutable_xyzs().col(i) = Vector3f(
          expected_x, expected_y, kDepthValue);
    }
  }

  // This tolerance was determined empirically for Drake's supported platforms.
  constexpr float kDistanceTolerance = 1e-8;
  PointCloud result;

  // Without a pose offset nor a scale factor.
  result = this->DoConvert(camera, nullopt, image, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud.xyzs(),
                              kDistanceTolerance));

  // Now with scale factor.
  result = this->DoConvert(camera, nullopt, image, 0.001);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud.xyzs() * 0.001,
                              kDistanceTolerance));

  // Now with a pose offset -- just check the z values.
  const auto& pose = this->z_translation_;
  const auto& expected_z_row = expected_cloud.xyzs().row(2).array().eval();
  result = this->DoConvert(camera, pose, image, nullopt);
  EXPECT_TRUE(CompareMatrices(
      result.xyzs().row(2),
      (expected_z_row + pose.translation().z()).matrix(),
      kDistanceTolerance));

  // With both scale and pose offset -- just check the z values.
  result = this->DoConvert(camera, pose, image, 0.001);
  EXPECT_TRUE(CompareMatrices(
      result.xyzs().row(2),
      (expected_z_row * 0.001 + pose.translation().z()).matrix(),
      kDistanceTolerance));
}

// Verifies computed point cloud when pixel value is NaN.
TYPED_TEST(DepthImageToPointCloudTest, NanValue) {
  using Pixel = typename TestFixture::Pixel;

  // This test only applies for 32F images; there is no NaN for 16U images.
  constexpr bool is_meaningful_nan = !std::is_same<Pixel, uint16_t>::value;
  if (!is_meaningful_nan) {
    return;
  }
  // Use the ?: operator to avoid a -Woverflow warning.  (The exception case is
  // never reached, but mitigates the warning.)
  const Pixel single_pixel = is_meaningful_nan ? kFloatNaN
      : throw std::runtime_error("");

  // Test all combinations of {without pose, with pose} x
  // {without scale, with scale}.
  const auto& camera = this->single_pixel_camera_;
  const auto& pose = this->random_transform_;
  const auto& image = Vector1<Pixel>::Constant(single_pixel).eval();
  const auto& expected_cloud = Vector3f::Constant(kFloatNaN).eval();
  PointCloud result;

  result = this->DoConvert(camera, nullopt, image, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, nullopt, image, 0.1);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, pose, image, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, pose, image, 0.1);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));
}

// Verifies computed point cloud when pixel value is kTooClose or kTooFar.
TYPED_TEST(DepthImageToPointCloudTest, TooNearFar) {
  using Pixel = typename TestFixture::Pixel;
  using Traits = typename TestFixture::ConfiguredImageTraits;
  const auto& camera = this->single_pixel_camera_;
  const auto& pose = this->random_transform_;
  const auto& image_near = Vector1<Pixel>::Constant(Traits::kTooClose).eval();
  const auto& image_far = Vector1<Pixel>::Constant(Traits::kTooFar).eval();
  const auto& expected_cloud = Vector3f::Constant(kFloatInf).eval();
  PointCloud result;

  // Test all combinations of {without pose, with pose} x {near, far} x
  // {without scale, with scale}.
  result = this->DoConvert(camera, nullopt, image_near, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, nullopt, image_near, 0.1);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, nullopt, image_far, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, nullopt, image_far, 0.1);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, pose, image_near, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, pose, image_near, 0.1);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, pose, image_far, nullopt);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));

  result = this->DoConvert(camera, pose, image_far, 0.1);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), expected_cloud));
}

// Verifies CalcOutput resets invalid storage.
TYPED_TEST(DepthImageToPointCloudTest, ResetStorage) {
  // Create the DUT.
  const auto& camera = this->single_pixel_camera_;
  const DepthImageToPointCloud dut(camera, TestFixture::kConfiguredPixelType);
  auto context = dut.CreateDefaultContext();

  // Give it a single-pixel input image (with depth value of 1).
  using ConfiguredImage = typename TestFixture::ConfiguredImage;
  const ConfiguredImage image(1, 1, 1);
  context->FixInputPort(0, Value<ConfiguredImage>(image));

  // Prepare to Calc directly.
  const auto& port = dut.point_cloud_output_port();
  std::unique_ptr<AbstractValue> output_value;
  PointCloud result;

  // Starting from default storage, check the point cloud result.
  output_value = std::make_unique<Value<PointCloud>>();
  port.Calc(*context, output_value.get());
  result = output_value->GetValueOrThrow<PointCloud>();
  ASSERT_EQ(result.fields(), pc_flags::kXYZs);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), Vector3f(0, 0, 1)));

  // If the output has the wrong set of channels, currently the DUT throws an
  // exception because the PointCloud offers no way to change channels after
  // construction.  (Ideally, the dut would produce the expected output by
  // resetting the set of channels, in which case this test should be updated.)
  output_value = std::make_unique<Value<PointCloud>>(
      1, pc_flags::kXYZs | pc_flags::kRGBs);
  EXPECT_THROW(port.Calc(*context, output_value.get()), std::exception);

  // If the storage was the wrong size, then Calc should be able to resize it.
  output_value = std::make_unique<Value<PointCloud>>(22);
  port.Calc(*context, output_value.get());
  result = output_value->GetValueOrThrow<PointCloud>();
  ASSERT_EQ(result.fields(), pc_flags::kXYZs);
  EXPECT_TRUE(CompareMatrices(result.xyzs(), Vector3f(0, 0, 1)));
}

}  // namespace
}  // namespace perception
}  // namespace drake
