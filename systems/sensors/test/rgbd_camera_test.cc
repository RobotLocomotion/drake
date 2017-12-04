#include "drake/systems/sensors/rgbd_camera.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <vtkVersion.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/unused.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

// RgbdCameraDiscrete is tested in :rgbd_camera_publish_lcm_test,
// given the relative simplicity of its interface.

// Note that we use ASSERTs instead of EXPECTs when we need to traverse many
// pixels in an image or many points in a point cloud. This is to prevent
// outputting massive amount of error messages and making debug hard when
// test failed.

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Mac OSX.
constexpr double kTolerance = 1e-12;
constexpr double kFovY = M_PI_4;
constexpr bool kShowWindow = false;
constexpr double kDepthRangeNear = 0.5;
constexpr double kDepthRangeFar = 5.;
constexpr int kWidth = 640;
constexpr int kHeight = 480;

void VerifyCameraInfo(const CameraInfo& camera_info) {
  EXPECT_EQ(kWidth, camera_info.width());
  EXPECT_EQ(kHeight, camera_info.height());
  EXPECT_NEAR(kWidth * 0.5, camera_info.center_x(), kTolerance);
  EXPECT_NEAR(kHeight * 0.5, camera_info.center_y(), kTolerance);

  // The expected focal value is calculated by the equation here:
  // https://github.com/RobotLocomotion/drake/blob/master/drake/systems/sensors/camera_info.h
  constexpr double kExpectedFocal = 579.41125496954282;
  EXPECT_NEAR(kExpectedFocal, camera_info.focal_x(), kTolerance);
  EXPECT_NEAR(kExpectedFocal, camera_info.focal_y(), kTolerance);
}

void VerifyCameraPose(const Eigen::Isometry3d camera_optical_pose) {
  // This is calculated by hand.
  const Eigen::Isometry3d kExpected((
      Eigen::Matrix4d() <<
       0.,  0., 1., 0.,
      -1.,  0., 0., 0.02,
       0., -1., 0., 0.,
       0.,  0., 0., 1.).finished());

  EXPECT_TRUE(CompareMatrices(kExpected.matrix(),
                              camera_optical_pose.matrix(),
                              kTolerance));
}

GTEST_TEST(RgbdCamera, TestInstantiation) {
  auto Verify = [](const RgbdCamera& camera) {
    VerifyCameraInfo(camera.color_camera_info());
    VerifyCameraInfo(camera.depth_camera_info());
    VerifyCameraPose(camera.color_camera_optical_pose());
    VerifyCameraPose(camera.depth_camera_optical_pose());
    EXPECT_NO_THROW(camera.tree());
  };

  RgbdCamera fixed_camera("rgbd_camera",
                          RigidBodyTree<double>(),
                          Eigen::Vector3d(1., 2., 3.),
                          Eigen::Vector3d(0.1, 0.2, 0.3),
                          kDepthRangeNear, kDepthRangeFar,
                          kFovY, kShowWindow);
  Verify(fixed_camera);
  // With the fixed camera use case, RgbdCamera doesn't hold a frame, thus
  // throws an exception.
  EXPECT_THROW(fixed_camera.frame(), std::logic_error);

  RgbdCamera movable_camera("rgbd_camera",
                            RigidBodyTree<double>(),
                            RigidBodyFrame<double>(),
                            kDepthRangeNear, kDepthRangeFar,
                            kFovY, kShowWindow);
  Verify(movable_camera);
  EXPECT_NO_THROW(movable_camera.frame());
}


class RgbdCameraDiagram : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCameraDiagram)

  explicit RgbdCameraDiagram(const std::string& sdf_file) {
    this->set_name("rgbd_camera_diagram");

    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
        sdf_file, multibody::joints::kQuaternion, tree.get());

    drake::multibody::AddFlatTerrainToWorld(tree.get());

    plant_ = builder_.AddSystem<RigidBodyPlant<double>>(std::move(tree));
    plant_->set_name("rigid_body_plant");
  }

  // For fixed camera base.
  void Init(const Eigen::Vector3d& position,
            const Eigen::Vector3d& orientation) {
    rgbd_camera_ = builder_.AddSystem<RgbdCamera>(
        "rgbd_camera", plant_->get_rigid_body_tree(), position, orientation,
        kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow);
    rgbd_camera_->set_name("rgbd_camera");
    Connect();
  }

  // For movable camera base.
  void Init(const Eigen::Isometry3d& transformation) {
    rgbd_camera_frame_ = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(),
        "rgbd camera frame", plant_->get_rigid_body_tree().FindBody("link"),
        transformation);

    rgbd_camera_ = builder_.AddSystem<RgbdCamera>(
        "rgbd_camera", plant_->get_rigid_body_tree(), *rgbd_camera_frame_.get(),
        kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow);
    rgbd_camera_->set_name("rgbd_camera");
    Connect();
  }

 private:
  void Connect() {
    builder_.Connect(plant_->state_output_port(),
                     rgbd_camera_->state_input_port());
    builder_.ExportOutput(rgbd_camera_->color_image_output_port());
    builder_.ExportOutput(rgbd_camera_->depth_image_output_port());
    builder_.ExportOutput(rgbd_camera_->label_image_output_port());
    builder_.ExportOutput(rgbd_camera_->camera_base_pose_output_port());
    builder_.BuildInto(this);
  }

  systems::DiagramBuilder<double> builder_;
  RigidBodyPlant<double>* plant_;
  RgbdCamera* rgbd_camera_;
  std::shared_ptr<RigidBodyFrame<double>> rgbd_camera_frame_;
};

class RgbdCameraDiagramTest : public ::testing::Test {
 public:
  void Verify() {
    diagram_->CalcOutput(*context_, output_.get());
    auto color = output_->GetMutableData(0)->GetMutableValue<
      sensors::ImageRgba8U>();
    auto depth = output_->GetMutableData(1)->GetMutableValue<
      sensors::ImageDepth32F>();
    auto label = output_->GetMutableData(2)->GetMutableValue<
      sensors::ImageLabel16I>();

    EXPECT_EQ(color.width(), kWidth);
    EXPECT_EQ(color.height(), kHeight);
    EXPECT_EQ(depth.width(), kWidth);
    EXPECT_EQ(depth.height(), kHeight);
    EXPECT_EQ(label.width(), kWidth);
    EXPECT_EQ(label.height(), kHeight);

    // Verifying all the pixel has the same values in each images.
    const auto& kColor = color.at(0, 0);
    const auto& kDepth = depth.at(0, 0);
    const auto& kLabel = label.at(0, 0);
    for (int v = 0; v < kHeight; ++v) {
      for (int u = 0; u < kWidth; ++u) {
        for (int ch = 0; ch < color.kNumChannels; ++ch) {
          ASSERT_EQ(kColor[ch], color.at(u, v)[ch]);
        }
        ASSERT_EQ(kDepth[0], depth.at(u, v)[0]);
        ASSERT_EQ(kLabel[0], label.at(u, v)[0]);
      }
    }
  }

 protected:
  // For fixed camera base.
  void Init(const std::string& sdf, const Eigen::Vector3d& position,
            const Eigen::Vector3d& orientation) {
    diagram_ = std::make_unique<RgbdCameraDiagram>(
        FindResourceOrThrow("drake/systems/sensors/test/models/" + sdf));
    diagram_->Init(position, orientation);
    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);
  }

  // For moving camera base.
  void Init(const std::string& sdf,
            const Eigen::Isometry3d& transformation) {
    diagram_ = std::make_unique<RgbdCameraDiagram>(
        FindResourceOrThrow("drake/systems/sensors/test/models/" + sdf));
    diagram_->Init(transformation);
    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);
  }

  std::unique_ptr<systems::SystemOutput<double>> output_;

 private:
  std::unique_ptr<RgbdCameraDiagram> diagram_;
  std::unique_ptr<systems::Context<double>> context_;
};


// Verifies the output images whose all the pixels have the same value as well
// as the output camera pose.
TEST_F(RgbdCameraDiagramTest, FixedCameraOutputTest) {
  // RgbdCamera is looking straight down 1m above the ground.
  const Eigen::Vector3d position(0., 0., 1.);
  const Eigen::Vector3d orientation(0., M_PI_2, 0.);

  Init("nothing.sdf", position, orientation);
  Verify();

  rendering::PoseVector<double>* const camera_base_pose =
      dynamic_cast<rendering::PoseVector<double>*>(
          output_->GetMutableVectorData(3));

  const Eigen::Isometry3d actual = camera_base_pose->get_isometry();
  EXPECT_TRUE(CompareMatrices(position.matrix(),
                              actual.translation().matrix(), kTolerance));
  EXPECT_TRUE(CompareMatrices(math::rpy2rotmat(orientation).matrix(),
                              actual.linear().matrix(), kTolerance));
}

TEST_F(RgbdCameraDiagramTest, MovableCameraOutputTest) {
  // RgbdCamera is looking straight down 1m above the ground.
  const Eigen::Isometry3d X_WB = Eigen::Translation3d(0., 0., 1.) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());

  Init("nothing.sdf", X_WB);
  Verify();

  rendering::PoseVector<double>* const camera_base_pose =
      dynamic_cast<rendering::PoseVector<double>*>(
          output_->GetMutableVectorData(3));

  const Eigen::Isometry3d actual = camera_base_pose->get_isometry();
  EXPECT_TRUE(CompareMatrices(X_WB.matrix(),
                              actual.matrix(), kTolerance));
}

class DepthImageToPointCloudConversionTest : public ::testing::Test {
 public:
  static constexpr float kFocal = 500.f;
  static constexpr int kDepthWidth = 60;
  static constexpr int kDepthHeight = 40;

  DepthImageToPointCloudConversionTest() : camera_info_(
      kDepthWidth, kDepthHeight,
      kFocal, kFocal, kDepthWidth * 0.5, kDepthHeight * 0.5),
      depth_image_(kDepthWidth, kDepthHeight, 1) {}

  // kTooClose is treated as kTooFar. For the detail, refer to the document of
  // RgbdCamera::ConvertDepthImageToPointCloud.
  void VerifyTooFarTooClose() {
    for (int v = 0; v < depth_image_.height(); ++v) {
      for (int u = 0; u < depth_image_.width(); ++u) {
        const int i = v * depth_image_.width() + u;
        Eigen::Vector3f actual = actual_point_cloud_.col(i);
        ASSERT_EQ(actual(0), InvalidDepth::kTooFar);
        ASSERT_EQ(actual(1), InvalidDepth::kTooFar);
        ASSERT_EQ(actual(2), InvalidDepth::kTooFar);
      }
    }
  }

 protected:
  // This must be called by all the test cases first.
  void Init(float depth_value) {
    std::fill(depth_image_.at(0, 0),
              depth_image_.at(0, 0) + depth_image_.size(),
              depth_value);
  }

  const CameraInfo camera_info_;
  ImageDepth32F depth_image_;
  Eigen::Matrix3Xf actual_point_cloud_;
};

// Verifies computed point cloud when pixel values in depth image are valid.
TEST_F(DepthImageToPointCloudConversionTest, ValidValueTest) {
  constexpr float kDepthValue = 1.f;
  Init(kDepthValue);

  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image_, camera_info_, &actual_point_cloud_);

  // This tolerance was determined empirically using Drake's supported
  // platforms.
  constexpr float kDistanceTolerance = 1e-8;
  for (int v = 0; v < depth_image_.height(); ++v) {
    for (int u = 0; u < depth_image_.width(); ++u) {
      const int i = v * depth_image_.width() + u;
      Eigen::Vector3f actual = actual_point_cloud_.col(i);

      const double expected_x =
          kDepthValue * (u - kDepthWidth * 0.5) / kFocal;
      ASSERT_NEAR(actual(0), expected_x, kDistanceTolerance);
      const double expected_y =
          kDepthValue * (v - kDepthHeight * 0.5) / kFocal;
      ASSERT_NEAR(actual(1), expected_y, kDistanceTolerance);
      ASSERT_NEAR(actual(2), kDepthValue, kDistanceTolerance);
    }
  }
}

// Verifies computed point cloud when pixel values in depth image are NaN.
TEST_F(DepthImageToPointCloudConversionTest, NanValueTest) {
  Init(std::numeric_limits<float>::quiet_NaN());

  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image_, camera_info_, &actual_point_cloud_);

  for (int v = 0; v < depth_image_.height(); ++v) {
    for (int u = 0; u < depth_image_.width(); ++u) {
      const int i = v * depth_image_.width() + u;
      Eigen::Vector3f actual = actual_point_cloud_.col(i);
      ASSERT_TRUE(std::isnan(actual(0)));
      ASSERT_TRUE(std::isnan(actual(1)));
      ASSERT_TRUE(std::isnan(actual(2)));
    }
  }
}

// Verifies computed point cloud when pixel values in depth image are kTooFar.
TEST_F(DepthImageToPointCloudConversionTest, TooFarTest) {
  Init(InvalidDepth::kTooFar);

  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image_, camera_info_, &actual_point_cloud_);

  VerifyTooFarTooClose();
}

// Verifies computed point cloud when pixel values in depth image are kTooClose.
TEST_F(DepthImageToPointCloudConversionTest, TooCloseTest) {
  Init(InvalidDepth::kTooClose);

  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image_, camera_info_, &actual_point_cloud_);

  VerifyTooFarTooClose();
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
