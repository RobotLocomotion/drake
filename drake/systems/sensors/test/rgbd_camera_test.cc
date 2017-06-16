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
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/unused.h"
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

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Macintosh OSX.
const double kTolerance = 1e-12;
const double kColorPixelTolerance = 1.001;
const double kDepthTolerance = 1e-4;
const double kFovY = M_PI_4;
const bool kShowWindow = false;

class RgbdCameraTest : public ::testing::Test {
 public:
  RgbdCameraTest() : dut_("rgbd_camera", RigidBodyTree<double>(),
                          Eigen::Vector3d(1., 2., 3.),
                          Eigen::Vector3d(0.1, 0.2, 0.3),
                          kFovY,  kShowWindow) {}

  void SetUp() {}

  void Verify(const CameraInfo& dut) {
    const int kExpectedWidth = 640;
    const int kExpectedHeight = 480;
    EXPECT_EQ(kExpectedWidth, dut.width());
    EXPECT_EQ(kExpectedHeight, dut.height());
    EXPECT_NEAR(kExpectedWidth * 0.5, dut.center_x(), kTolerance);
    EXPECT_NEAR(kExpectedHeight * 0.5, dut.center_y(), kTolerance);

    // The expected focal value is calculated by the equation here:
    // https://github.com/RobotLocomotion/drake/blob/master/drake/systems/sensors/camera_info.h#L87
    const double kExpectedFocal = 579.41125496954282;
    EXPECT_NEAR(kExpectedFocal, dut.focal_x(), kTolerance);
    EXPECT_NEAR(kExpectedFocal, dut.focal_y(), kTolerance);
  }

 protected:
  const RgbdCamera dut_;
};

TEST_F(RgbdCameraTest, InstantiateTest) {
  Verify(dut_.color_camera_info());
  Verify(dut_.depth_camera_info());
}

TEST_F(RgbdCameraTest, ColorAndDepthCameraPoseTest) {
  // This is calculated by hand.
  const Eigen::Isometry3d expected_X_BC((
      Eigen::Matrix4d() <<
       0.,  0., 1., 0.,
      -1.,  0., 0., 0.02,
       0., -1., 0., 0.,
       0.,  0., 0., 1.).finished());

  EXPECT_TRUE(CompareMatrices(expected_X_BC.matrix(),
                              dut_.color_camera_optical_pose().matrix(),
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(expected_X_BC.matrix(),
                              dut_.depth_camera_optical_pose().matrix(),
                              kTolerance));
}

class RenderingSim : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RenderingSim)

  explicit RenderingSim(const std::string& sdf_file) {
    this->set_name("rendering_sim");

    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
        sdf_file, multibody::joints::kQuaternion, tree.get());

    drake::multibody::AddFlatTerrainToWorld(tree.get());

    plant_ = builder_.AddSystem<RigidBodyPlant<double>>(std::move(tree));
    plant_->set_name("rigid_body_plant");
    const double kPenetrationStiffness = 3000.;
    const double kPenetrationDamping = 10.;
    const double kStaticFriction = 0.9;
    const double kDynamicFriction = 0.5;
    const double kStictionSlipTolerance = 0.01;
    plant_->set_normal_contact_parameters(kPenetrationStiffness,
                                          kPenetrationDamping);
    plant_->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                            kStictionSlipTolerance);
  }

  // For fixed camera base.
  void InitFixedCamera(const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation) {
    rgbd_camera_ = builder_.AddSystem<RgbdCamera>(
        "rgbd_camera", plant_->get_rigid_body_tree(),
        position, orientation, kFovY, kShowWindow);
    rgbd_camera_->set_name("rgbd_camera");
    Connect();
  }

  // For movable camera base.
  void InitMovableCamera(const Eigen::Isometry3d& transformation) {
    rgbd_camera_frame_ = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(),
        "rgbd camera frame", plant_->get_rigid_body_tree().FindBody("link"),
        transformation);

    rgbd_camera_ = builder_.AddSystem<RgbdCamera>(
        "rgbd_camera", plant_->get_rigid_body_tree(), *rgbd_camera_frame_.get(),
        kFovY, kShowWindow);
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

// TODO(kunimatsu-tri) Remove this once the arbitrary terrain color support
// is added.
const std::array<uint8_t, 4> kTerrainColor{{255u, 229u, 204u, 255u}};

const int kWidth = 640;
const int kHeight = 480;

// Pixel coordinate system values.
struct UV {
  int u;
  int v;
};

UV kCorners[4] = {
  UV{0, 0},
  UV{kWidth - 1, 0},
  UV{0, kHeight - 1},
  UV{kWidth - 1, kHeight - 1}
};


class ImageTest : public ::testing::Test {
 public:
  typedef std::function<void(
      const sensors::ImageRgba8U& color_image,
      const sensors::ImageDepth32F& depth_image)> ImageVerifier;

  typedef std::function<void(
      const sensors::ImageRgba8U& color_image,
      const sensors::ImageDepth32F& depth_image,
      int horizon)> ImageHorizonVerifier;

  typedef std::function<void(
      const Eigen::Isometry3d& pose)> CameraBasePoseVerifier;

  void Verify(ImageVerifier verifier) {
    diagram_->CalcOutput(*context_, output_.get());

    auto color_image = output_->GetMutableData(0)->GetMutableValue<
      sensors::ImageRgba8U>();
    auto depth_image = output_->GetMutableData(1)->GetMutableValue<
      sensors::ImageDepth32F>();

    verifier(color_image, depth_image);
  }

  // Calculates the pixel location of the horizon.
  static int CalcHorizon(double z, int image_height) {
    const double elevation_per_pixel = kFovY / image_height;
    const double kTerrainSize = 50.;
    return image_height / 2 +
        std::atan(z / kTerrainSize) / elevation_per_pixel;
  }

  void VerifyPoseUpdate(ImageHorizonVerifier verifier) {
    auto& color_image = output_->GetMutableData(0)->GetMutableValue<
      sensors::ImageRgba8U>();
    auto& depth_image = output_->GetMutableData(1)->GetMutableValue<
      sensors::ImageDepth32F>();
    VectorBase<double>* cstate =
        context_->get_mutable_continuous_state_vector();

    const double kZInitial = 1.;
    const std::array<double, 3> kZDiffs{{0., -0.2, -0.5}};

    for (int i = 0; i < 3; ++i) {
      cstate->SetAtIndex(2, kZDiffs[i]);
      diagram_->CalcOutput(*context_, output_.get());
      double expected_horizon = CalcHorizon(kZInitial + kZDiffs[i],
                                            color_image.height());
      verifier(color_image, depth_image, expected_horizon);
    }
  }


  void Verify(CameraBasePoseVerifier verifier) {
    diagram_->CalcOutput(*context_, output_.get());
    rendering::PoseVector<double>* const camera_base_pose =
        dynamic_cast<rendering::PoseVector<double>*>(
            output_->GetMutableVectorData(3));

    verifier(camera_base_pose->get_isometry());
  }

  void VerifyLabelImage() {
    diagram_->CalcOutput(*context_, output_.get());
    auto label_image = output_->GetMutableData(2)->GetMutableValue<
      sensors::ImageLabel16I>();

    std::vector<int16_t> actual_ids;
    for (int v = 0; v < label_image.height(); ++v) {
      for (int u = 0; u < label_image.width(); ++u) {
        const int16_t id = label_image.at(u, v)[0];
        auto it = std::find(actual_ids.begin(), actual_ids.end(), id);
        if (it == actual_ids.end()) {
          actual_ids.push_back(id);
        }
      }
    }
    // We have three objects plus the sky and the terrain.
    const int kExpectedNumIds{5};
    EXPECT_EQ(kExpectedNumIds, actual_ids.size());

    ASSERT_EQ(label_image.at(320, 205)[0], 1);
    ASSERT_EQ(label_image.at(470, 205)[0], 2);
    ASSERT_EQ(label_image.at(170, 205)[0], 3);
    // Terrain
    ASSERT_EQ(label_image.at(0, 479)[0], RgbdCamera::Label::kFlatTerrain);
    // Sky
    ASSERT_EQ(label_image.at(0, 0)[0], RgbdCamera::Label::kNoBody);
  }

  static void VerifyCameraPose(const Eigen::Isometry3d& pose_actual) {
    const Eigen::Isometry3d expected(
        Eigen::Translation3d(0., 0., 4.999) *
        (Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ())));

    EXPECT_TRUE(CompareMatrices(expected.matrix(),
                                pose_actual.matrix(),
                                kTolerance));
  }

  static void VerifyUniformColorAndDepth(
      const sensors::ImageRgba8U& color_image,
      const sensors::ImageDepth32F& depth_image,
      const std::array<uint8_t, 4>& color, float depth) {
    // Verifies by sampling 32 x 24 points instead of 640 x 480 points. The
    // assumption is any defects will be detected by sampling this amount.
    for (int v = 0; v < color_image.height(); v += 20) {
      for (int u = 0; u < color_image.width(); u += 20) {
        for (int ch = 0; ch < color_image.kNumChannels; ++ch) {
          ASSERT_NEAR(color_image.at(u, v)[ch], color[ch],
                      kColorPixelTolerance);
        }
        // Assuming depth value provides 0.1 mm precision.
        ASSERT_NEAR(depth_image.at(u, v)[0], depth, kDepthTolerance);
      }
    }
  }

  static void VerifyTerrain(const sensors::ImageRgba8U& color_image,
                            const sensors::ImageDepth32F& depth_image) {
    VerifyUniformColorAndDepth(color_image, depth_image,
                               kTerrainColor, 4.999f);
  }

  static void VerifyBox(
      const sensors::ImageRgba8U& color_image,
      const sensors::ImageDepth32F& depth_image) {
    // This is given by the material diffuse element in `box.sdf`.
    const std::array<uint8_t, 4> kPixelColor{{255u, 255u, 255u, 255u}};
    VerifyUniformColorAndDepth(color_image, depth_image, kPixelColor, 1.f);
  }

  static void VerifyCylinder(
      const sensors::ImageRgba8U& color_image,
      const sensors::ImageDepth32F& depth_image) {
    // This is given by the material diffuse element in `cylinder.sdf`.
    const std::array<uint8_t, 4> kPixelColor{{255u, 0u, 255u, 255u}};
    VerifyUniformColorAndDepth(color_image, depth_image, kPixelColor, 1.f);
  }

  static void VerifyMeshBox(const sensors::ImageRgba8U& color_image,
                            const sensors::ImageDepth32F& depth_image) {
    // This is given by `box.png` which is the texture file for `box.obj`.
    const std::array<uint8_t, 4> kPixelColor{{4u, 241u, 33u, 255u}};
    VerifyUniformColorAndDepth(color_image, depth_image, kPixelColor, 1.f);
  }

  // Verifies the color and depth of the image at the center and four corners.
  static void VerifySphere(const sensors::ImageRgba8U& color_image,
                           const sensors::ImageDepth32F& depth_image) {
    // Verifies the four corner points.

    for (const auto& corner : kCorners) {
      for (int ch = 0; ch < color_image.kNumChannels; ++ch) {
        ASSERT_NEAR(color_image.at(corner.u, corner.v)[ch],
                    kTerrainColor[ch], kColorPixelTolerance);
      }
      ASSERT_NEAR(depth_image.at(corner.u, corner.v)[0], 2.f, kDepthTolerance);
    }

    // Verifies the center point's color.
    const int half_width = color_image.width() / 2;
    const int half_height = color_image.height() / 2;
    // If there is no material diffuse information provided in the SDF file, the
    // default color given by our SDF parser is `(0.7, 0.7, 0.7, 1.0)` which is
    // `(179u, 179u, 179u, 255u)` in `uint8_t`.
    for (int ch = 0; ch < color_image.kNumChannels - 1; ++ch) {
      ASSERT_NEAR(color_image.at(half_width, half_height)[ch],
                  179u, kColorPixelTolerance);
    }
    ASSERT_NEAR(color_image.at(half_width, half_height)[3],
                255u, kColorPixelTolerance);

    // Verifies the center point's depth.
    ASSERT_NEAR(depth_image.at(half_width, half_height)[0], 1.f,
                kDepthTolerance);
  }

  // Verifies the color and depth of the image at the two visuals (boxes).
  static void VerifyMultipleVisuals(const sensors::ImageRgba8U& color_image,
                                    const sensors::ImageDepth32F& depth_image) {
    // Verifies the four corner points.
    for (const auto& corner : kCorners) {
      for (int ch = 0; ch < color_image.kNumChannels; ++ch) {
        ASSERT_NEAR(color_image.at(corner.u, corner.v)[ch],
                    kTerrainColor[ch], kColorPixelTolerance);
      }
      ASSERT_NEAR(depth_image.at(corner.u, corner.v)[0], 4.999f,
                  kDepthTolerance);
    }

    // Verifies the four corner points.
    UV kRightBox{kWidth / 2 + 144, kHeight / 2};
    UV kLeftBox{kWidth / 2 - 144, kHeight / 2};
    for (int ch = 0; ch < color_image.kNumChannels - 1; ++ch) {
      ASSERT_NEAR(color_image.at(kRightBox.u, kRightBox.v)[ch],
                  179u, kColorPixelTolerance);
      ASSERT_NEAR(color_image.at(kLeftBox.u, kLeftBox.v)[ch],
                  179u, kColorPixelTolerance);
    }
    ASSERT_NEAR(depth_image.at(kRightBox.u, kRightBox.v)[0],
                3.999f, kDepthTolerance);
    ASSERT_NEAR(depth_image.at(kLeftBox.u, kLeftBox.v)[0],
                3.999f, kDepthTolerance);
  }

  static void VerifyMovingCamera(const sensors::ImageRgba8U& color_image,
                                 const sensors::ImageDepth32F& depth_image,
                                 int expected_horizon) {
    // TODO(jamiesnape): Is depth_image actually needed?
    drake::unused(depth_image);
    int actual_horizon{0};
    std::array<uint8_t, 4> color{{0u, 0u, 0u, 0u}};
    for (int v = 0; v < color_image.height(); ++v) {
      if ((color[0] != color_image.at(0, v)[0]) ||
          (color[1] != color_image.at(0, v)[1]) ||
          (color[2] != color_image.at(0, v)[2]) ||
          (color[3] != color_image.at(0, v)[3])) {
        for (int ch = 0; ch < color_image.kNumChannels; ++ch) {
          color[ch] = color_image.at(0, v)[ch];
        }
        actual_horizon = v;
      }
    }

    // We need a tolerance because the result varies depending on the CPU.
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }

 protected:
  void SetUp() override {}

  // For fixed camera base.
  void SetUp(const std::string& sdf, const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation) {
    diagram_ = std::make_unique<RenderingSim>(GetDrakePath() + sdf);
    diagram_->InitFixedCamera(position, orientation);
    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);
  }

  // For moving camera base.
  void SetUp(const std::string& sdf,
             const Eigen::Isometry3d& transformation) {
    diagram_ = std::make_unique<RenderingSim>(GetDrakePath() + sdf);
    diagram_->InitMovableCamera(transformation);
    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);
  }

 private:
  std::unique_ptr<RenderingSim> diagram_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};


// Verifies the rendered terrain and the camera's pose.
TEST_F(ImageTest, TerrainRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/nothing.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 4.999),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifyTerrain);
  Verify(ImageTest::VerifyCameraPose);
}

// Verifies the rendered box.
TEST_F(ImageTest, BoxRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/box.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 2.),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifyBox);
}

// Verifies the rendered cylinder.
TEST_F(ImageTest, CylinderRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/cylinder.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 2.),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifyCylinder);
}

// TODO(jamiesnape, kunimatsu-tri): Fix test for newer versions of VTK.
#if VTK_MAJOR_VERSION <= 5
// Verifies the rendered mesh box.
TEST_F(ImageTest, MeshBoxRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/mesh_box.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 3.),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifyMeshBox);
}
#endif

// Verifies the rendered sphere.
TEST_F(ImageTest, SphereRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/sphere.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 2.),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifySphere);
}

// Verifies the camera pose update.
// RgbdCamera's base is attached to a model.  RgbdCamera's optical axis is
// parallel to the flat terrain, so the camera sees the horizon.   This test
// verifies that the horizon's pixel location changes as the model's state
// updated.
TEST_F(ImageTest, CameraPoseUpdateTest) {
  const std::string sdf("/systems/sensors/test/models/nothing.sdf");
  // Attaches the camera to a location that is 11 m above the model.
  Eigen::Isometry3d transformation((Eigen::Matrix4d() <<
                                    1., 0., 0., 0.,
                                    0., 1., 0., 0.,
                                    0., 0., 1., 11.,
                                    0., 0., 0., 1.).finished());
  SetUp(sdf, transformation);
  VerifyPoseUpdate(ImageTest::VerifyMovingCamera);
}

// Verifies the number of ids in a label image.
TEST_F(ImageTest, LabelRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/three_boxes.sdf");
  SetUp(sdf,
        Eigen::Vector3d(-10., 0., 2.),
        Eigen::Vector3d(0., M_PI_4 * 0.2, 0.));
  VerifyLabelImage();
}

// Verifies the case that a model has multiple visuals.
TEST_F(ImageTest, MultipleVisualsTest) {
  // The following SDF includes a link that has more than two visuals.
  const std::string sdf("/systems/sensors/test/models/multiple_visuals.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 4.999),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifyMultipleVisuals);
}


class DepthImageToPointCloudConversionTest : public ::testing::Test {
 public:
  const float kFocal = 500.f;
  const int kWidth = 6;
  const int kHeight = 4;

  DepthImageToPointCloudConversionTest() : camera_info_(
      kWidth, kHeight, kFocal, kFocal, kWidth * 0.5, kHeight * 0.5),
      depth_image_(kWidth, kHeight, 1) {}

  void VerifyTooFarTooClose() {
    for (int v = 0; v < depth_image_.height(); ++v) {
      for (int u = 0; u < depth_image_.width(); ++u) {
        const int i = v * depth_image_.width() + u;
        Eigen::Vector3f actual(Eigen::Map<Eigen::Vector3f>(
            actual_point_cloud_.col(i).data(), actual_point_cloud_.rows()));

        EXPECT_EQ(actual(0), RgbdCamera::InvalidDepth::kTooFar);
        EXPECT_EQ(actual(1), RgbdCamera::InvalidDepth::kTooFar);
        EXPECT_EQ(actual(2), RgbdCamera::InvalidDepth::kTooFar);
      }
    }
  }

 protected:
  void InitDepthImage(float depth_value) {
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
  const float kDepthValue = 1.f;
  InitDepthImage(kDepthValue);

  RgbdCamera::ConvertDepthImageToPointCloud(depth_image_, camera_info_,
                                            &actual_point_cloud_);

  // This tolerance was determined empirically using Drake's supported
  // platforms.
  const float kDistanceTolerance = 1e-9;
  for (int v = 0; v < depth_image_.height(); ++v) {
    for (int u = 0; u < depth_image_.width(); ++u) {
      const int i = v * depth_image_.width() + u;
      Eigen::Vector3f actual(Eigen::Map<Eigen::Vector3f>(
          actual_point_cloud_.col(i).data(), actual_point_cloud_.rows()));

      EXPECT_NEAR(actual(0), kDepthValue * (u - kWidth * 0.5) / kFocal,
                  kDistanceTolerance);
      EXPECT_NEAR(actual(1), kDepthValue * (v - kHeight * 0.5) / kFocal,
                  kDistanceTolerance);
      EXPECT_NEAR(actual(2), kDepthValue, kDistanceTolerance);
    }
  }
}

// Verifies computed point cloud when pixel values in depth image are NaN.
TEST_F(DepthImageToPointCloudConversionTest, NanValueTest) {
  const float kDepthValue = std::numeric_limits<float>::quiet_NaN();
  InitDepthImage(kDepthValue);

  RgbdCamera::ConvertDepthImageToPointCloud(depth_image_, camera_info_,
                                            &actual_point_cloud_);

  for (int v = 0; v < depth_image_.height(); ++v) {
    for (int u = 0; u < depth_image_.width(); ++u) {
      const int i = v * depth_image_.width() + u;
      Eigen::Vector3f actual(Eigen::Map<Eigen::Vector3f>(
          actual_point_cloud_.col(i).data(), actual_point_cloud_.rows()));

      EXPECT_TRUE(std::isnan(actual(0)));
      EXPECT_TRUE(std::isnan(actual(1)));
      EXPECT_TRUE(std::isnan(actual(2)));
    }
  }
}

// Verifies computed point cloud when pixel values in depth image are kTooFar.
TEST_F(DepthImageToPointCloudConversionTest, TooFarTest) {
  const float kDepthValue = RgbdCamera::InvalidDepth::kTooFar;
  InitDepthImage(kDepthValue);

  RgbdCamera::ConvertDepthImageToPointCloud(depth_image_, camera_info_,
                                            &actual_point_cloud_);

  VerifyTooFarTooClose();
}

// Verifies computed point cloud when pixel values in depth image are kTooClose.
TEST_F(DepthImageToPointCloudConversionTest, TooCloseTest) {
  const float kDepthValue = RgbdCamera::InvalidDepth::kTooClose;
  InitDepthImage(kDepthValue);

  RgbdCamera::ConvertDepthImageToPointCloud(depth_image_, camera_info_,
                                            &actual_point_cloud_);

  VerifyTooFarTooClose();
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
