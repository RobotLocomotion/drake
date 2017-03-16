#include "drake/systems/sensors/rgbd_camera.h"

#include <cmath>
#include <functional>
#include <stdexcept>

#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Macintosh OSX.
const double kTolerance = 1e-12;
const uint8_t kColorPixelTolerance = 1u;
const double kFovY = M_PI_4;
const bool kShowWindow = true;

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

    // Expected focal values are calculated by hand.
    const double kExpectedFocalX = 554.25625842204079;
    const double kExpectedFocalY = 579.41125496954282;
    EXPECT_NEAR(kExpectedFocalX, dut.focal_x(), kTolerance);
    EXPECT_NEAR(kExpectedFocalY, dut.focal_y(), kTolerance);
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
    Connect();
  }

 private:
  void Connect() {
    builder_.Connect(plant_->state_output_port(),
                     rgbd_camera_->state_input_port());
    builder_.ExportOutput(rgbd_camera_->color_image_output_port());
    builder_.ExportOutput(rgbd_camera_->depth_image_output_port());
    builder_.ExportOutput(rgbd_camera_->camera_base_pose_output_port());
    builder_.BuildInto(this);
  }

  systems::DiagramBuilder<double> builder_;
  RigidBodyPlant<double>* plant_;
  RgbdCamera* rgbd_camera_;
  std::shared_ptr<RigidBodyFrame<double>> rgbd_camera_frame_;
};

void AssertIntNear(int value_a, int value_b, int tolerance) {
  ASSERT_LE(std::abs(value_a - value_b), tolerance);
}

const std::array<uint8_t, 4> kBackgroundColor{{204u, 229u, 255u, 255u}};

class ImageTest : public ::testing::Test {
 public:
  typedef std::function<void(
      const sensors::Image<uint8_t>& color_image,
      const sensors::Image<float>& depth_image)> ImageVerifier;

  typedef std::function<void(
      const sensors::Image<uint8_t>& color_image,
      const sensors::Image<float>& depth_image,
      int horizon)> ImageHorizonVerifier;

  typedef std::function<void(
      const Eigen::Isometry3d& pose)> CameraBasePoseVerifier;

  void Verify(ImageVerifier verifier) {
    diagram_->CalcOutput(*context_, output_.get());

    auto color_image = output_->GetMutableData(0)->GetMutableValue<
      sensors::Image<uint8_t>>();
    auto depth_image = output_->GetMutableData(1)->GetMutableValue<
      sensors::Image<float>>();

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
      sensors::Image<uint8_t>>();
    auto& depth_image = output_->GetMutableData(1)->GetMutableValue<
      sensors::Image<float>>();
    VectorBase<double>* cstate =
        context_->get_mutable_continuous_state_vector();

    const double kZInitial = 1.;
    std::array<double, 3> kZDiffs{{0., -0.2, -0.5}};

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
            output_->GetMutableVectorData(2));

    verifier(camera_base_pose->get_isometry());
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
      const sensors::Image<uint8_t>& color_image,
      const sensors::Image<float>& depth_image,
      const std::array<uint8_t, 4>& color, float depth) {
    // Verifies by sampling 32 x 24 points instead of 640 x 480 points. The
    // assumption is any defects will be detected by sampling this amount.
    for (int v = 0; v < color_image.height(); v += 20) {
      for (int u = 0; u < color_image.width(); u += 20) {
        for (int ch = 0; ch < 4; ++ch) {
          AssertIntNear(color_image.at(u, v)[ch], color[ch],
                        kColorPixelTolerance);
        }
        // Assuming depth value provides 0.1 mm precision.
        ASSERT_NEAR(depth_image.at(u, v)[0], depth, 1e-4);
      }
    }
  }

  static void VerifyTerrain(const sensors::Image<uint8_t>& color_image,
                            const sensors::Image<float>& depth_image) {
    VerifyUniformColorAndDepth(color_image, depth_image,
                               kBackgroundColor, 4.999f);
  }

  static void VerifyBox(
      const sensors::Image<uint8_t>& color_image,
      const sensors::Image<float>& depth_image) {
    const std::array<uint8_t, 4> kPixelColor{{255u, 255u, 255u, 255u}};
    VerifyUniformColorAndDepth(color_image, depth_image, kPixelColor, 1.f);
  }

  static void VerifyCylinder(
      const sensors::Image<uint8_t>& color_image,
      const sensors::Image<float>& depth_image) {
    const std::array<uint8_t, 4> kPixelColor{{255u, 255u, 255u, 255u}};
    VerifyUniformColorAndDepth(color_image, depth_image, kPixelColor, 1.f);
  }

  static void VerifyMeshBox(const sensors::Image<uint8_t>& color_image,
                            const sensors::Image<float>& depth_image) {
    const std::array<uint8_t, 4> kPixelColor{{33u, 241u, 4u, 255u}};
    VerifyUniformColorAndDepth(color_image, depth_image, kPixelColor, 1.f);
  }

  struct UV {
    int u;
    int v;
  };

  // Verifies the color and depth of the image at the center and four corners.
  static void VerifySphere(const sensors::Image<uint8_t>& color_image,
                           const sensors::Image<float>& depth_image) {
    // Verifies the four corner points.
    UV kCorners[4] = {UV{0, 0},
                      UV{color_image.width() - 1, 0},
                      UV{0, color_image.height() - 1},
                      UV{color_image.width() - 1,
                         color_image.height() - 1}};

    for (const auto& corner : kCorners) {
      for (int ch = 0; ch < color_image.num_channels(); ++ch) {
        AssertIntNear(color_image.at(corner.u, corner.v)[ch],
                      kBackgroundColor[ch], kColorPixelTolerance);
      }
      ASSERT_NEAR(depth_image.at(corner.u, corner.v)[0], 2.f, 1e-4);
    }

    // Verifies the center point's color.
    const int kHalfWidth = color_image.width() / 2;
    const int kHalfHeight = color_image.height() / 2;
    for (int ch = 0; ch < color_image.num_channels(); ++ch) {
      AssertIntNear(color_image.at(kHalfWidth, kHalfHeight)[ch],
                    255u, kColorPixelTolerance);
    }
    // Verifies the center point's depth.
    ASSERT_NEAR(depth_image.at(kHalfWidth, kHalfHeight)[0], 1.f, 1e-4);
  }

  static void VerifyMovingCamera(const sensors::Image<uint8_t>& color_image,
                                 const sensors::Image<float>& depth_image,
                                 int expected_horizon) {
    int actual_horizon{0};
    std::array<uint8_t, 4> color{{0u, 0u, 0u, 0u}};
    for (int v = 0; v < color_image.height(); ++v) {
      if ((color[0] != color_image.at(0, v)[0]) ||
          (color[1] != color_image.at(0, v)[1]) ||
          (color[2] != color_image.at(0, v)[2]) ||
          (color[3] != color_image.at(0, v)[3])) {
        for (int ch = 0; ch < 4; ++ch) {
          color[ch] = color_image.at(0, v)[ch];
        }
        actual_horizon = v;
      }
    }

    // We need a tolerance because the result varies depending on the CPU.
    AssertIntNear(expected_horizon, actual_horizon, 1);
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

// Verifies the rendered mesh box.
TEST_F(ImageTest, MeshBoxRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/mesh_box.sdf");
  SetUp(sdf,
        Eigen::Vector3d(0., 0., 3.),
        Eigen::Vector3d(0., M_PI_2, 0.));
  Verify(ImageTest::VerifyMeshBox);
}

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


// Verifies an exception is thrown if a link has more than two visuals.
GTEST_TEST(RenderingTest, MultipleVisualsTest) {
  // The following SDF includes a link that has more than two visuals.
  const std::string sdf("/systems/sensors/test/models/bad.sdf");
  RenderingSim diagram(GetDrakePath() + sdf);
  EXPECT_THROW(diagram.InitFixedCamera(Eigen::Vector3d(0., 0., 1.),
                                       Eigen::Vector3d(0., 0., 0.)),
               std::runtime_error);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
