#include "drake/systems/sensors/rgbd_camera.h"

#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image.h"

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

// Verifies the initial camera base pose.
TEST_F(RgbdCameraTest, InitialCameraBasePoseTest) {
  // This is calculated by hand.
  const Eigen::Isometry3d expected((
      Eigen::Matrix4d() <<
      0.9362933635841, -0.2750958473182,  0.2183506631463, 1.,
      0.2896294776255,  0.9564250858492, -0.0369570135246, 2.,
      -0.1986693307950,  0.0978433950072,  0.9751703272018, 3.,
      0., 0., 0., 1.).finished());

  EXPECT_TRUE(CompareMatrices(expected.matrix(),
                              dut_.base_pose().matrix(), kTolerance));
}

TEST_F(RgbdCameraTest, ColorAndDepthCameraPoseTest) {
  // This is calculated by hand.
  const Eigen::Isometry3d expected_base_to_optical((
      Eigen::Matrix4d() <<
       0.,  0., 1., 0.,
      -1.,  0., 0., 0.02,
       0., -1., 0., 0.,
       0.,  0., 0., 1.).finished());

  EXPECT_TRUE(CompareMatrices(expected_base_to_optical.matrix(),
                              dut_.base_pose().inverse().matrix() *
                              dut_.color_camera_optical_pose().matrix(),
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(expected_base_to_optical.matrix(),
                              dut_.base_pose().inverse().matrix() *
                              dut_.depth_camera_optical_pose().matrix(),
                              kTolerance));
}

class RenderingSim : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RenderingSim)

  RenderingSim() { this->set_name("rendering_sim"); }

  void Init(const std::string& sdf_file,
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& orientation) {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
        sdf_file, multibody::joints::kQuaternion, tree.get());

    drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<double> builder;

    plant_ = builder.AddSystem<RigidBodyPlant<double>>(std::move(tree));
    const double kPenetrationStiffness = 3000.;
    const double kPenetrationDamping = 30.;
    plant_->set_normal_contact_parameters(kPenetrationStiffness,
                                          kPenetrationDamping);

    rgbd_camera_ = builder.AddSystem<RgbdCamera>(
        "rgbd_camera", plant_->get_rigid_body_tree(),
        position, orientation, kFovY, kShowWindow);

    builder.Connect(plant_->state_output_port(),
                    rgbd_camera_->state_input_port());
    builder.ExportOutput(rgbd_camera_->color_image_output_port());
    builder.ExportOutput(rgbd_camera_->depth_image_output_port());
    builder.BuildInto(this);
  }

 private:
  RigidBodyPlant<double>* plant_;
  RgbdCamera* rgbd_camera_;
};

void AssertLe(uint8_t value_a, uint8_t value_b, uint8_t tolerance) {
  int a = static_cast<int>(value_a);
  int b = static_cast<int>(value_b);
  int t = static_cast<int>(tolerance);
  ASSERT_LE(std::abs(a - b), t);
}

class ImageTest {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageTest)

  typedef std::function<void(
      const sensors::Image<uint8_t>& color_image,
      const sensors::Image<float>& depth_image)> Verifier;

  ImageTest(const std::string& sdf, const Eigen::Vector3d& position,
            const Eigen::Vector3d& orientation, Verifier verifier) {
    diagram_.Init(GetDrakePath() + sdf, position, orientation);

    std::unique_ptr<systems::Context<double>> context =
        diagram_.CreateDefaultContext();
    std::unique_ptr<systems::SystemOutput<double>> output =
        diagram_.AllocateOutput(*context);
    systems::Simulator<double> simulator(diagram_, std::move(context));
    simulator.Initialize();

    const double kDuration = 0.01;
    for (double time = 0.; time < kDuration ; time += 0.01) {
      simulator.StepTo(time);
      diagram_.CalcOutput(simulator.get_context(), output.get());

      systems::AbstractValue* mutable_data = output->GetMutableData(0);
      systems::AbstractValue* mutable_data_d = output->GetMutableData(1);
      auto color_image =
          mutable_data->GetMutableValue<sensors::Image<uint8_t>>();
      auto depth_image =
          mutable_data_d->GetMutableValue<sensors::Image<float>>();

      verifier(color_image, depth_image);
    }
  }

  static void VerifyTerrain(const sensors::Image<uint8_t>& color_image,
                            const sensors::Image<float>& depth_image) {
    // Verifies by sampling 32 x 24 points instead of 640 x 480 points. The
    // assumption is any defects will be detected by sampling this amount.
    for (int v = 0; v < color_image.height(); v += 20) {
      for (int u = 0; u < color_image.width(); u += 20) {
        // We need kColorPixelTolerance because it is possible to have rendering
        // errors dependeing on the hardware that VTK renderer uses.
        AssertLe(color_image.at(u, v)[0], 204u, kColorPixelTolerance);
        AssertLe(color_image.at(u, v)[1], 229u, kColorPixelTolerance);
        AssertLe(color_image.at(u, v)[2], 255u, kColorPixelTolerance);
        AssertLe(color_image.at(u, v)[3], 255u, kColorPixelTolerance);

        // Assuming depth value provides 0.1 mm precision.
        ASSERT_NEAR(depth_image.at(u, v)[0], 4.999f, 1e-4);
      }
    }
  }

  static void VerifyAllShapes(const sensors::Image<uint8_t>& color_image,
                              const sensors::Image<float>& depth_image) {
    const std::string color_file("/systems/sensors/test/images/color.png");
    const std::string depth_file("/systems/sensors/test/images/depth.png");

    vtkNew<vtkPNGReader> color_reader;
    color_reader->SetFileName((GetDrakePath() + color_file).c_str());
    color_reader->Update();
    vtkImageData* color_expected = color_reader->GetOutput();

    vtkNew<vtkPNGReader> depth_reader;
    depth_reader->SetFileName((GetDrakePath() + depth_file).c_str());
    depth_reader->Update();
    vtkImageData* depth_expected = depth_reader->GetOutput();

    // Verifies by sampling 64 x 48 points instead of 640 x 480 points. The
    // assumption is any defects will be detected by sampling this amount.
    // `y` is traversed in reverse (largest to smallest) order because the
    // origin of VTK's image coordinate system is on the left-bottom corner of
    // the image while `RgbdCamera`'s is on the left-top corner.
    int y = 47;
    int x = 0;
    for (int v = 0; v < color_image.height(); v += 10) {
      x = 0;
      for (int u = 0; u < color_image.width(); u += 10) {
        uint8_t* color = static_cast<uint8_t*>(
            color_expected->GetScalarPointer(x, y, 0));

        AssertLe(color_image.at(u, v)[0], *(color + 2),
                 kColorPixelTolerance);  // B
        AssertLe(color_image.at(u, v)[1], *(color + 1),
                 kColorPixelTolerance);  // G
        AssertLe(color_image.at(u, v)[2], *(color + 0),
                 kColorPixelTolerance);  // R
        AssertLe(color_image.at(u, v)[3], *(color + 3),
                 kColorPixelTolerance);  // A

        float* depth = static_cast<float*>(
            depth_expected->GetScalarPointer(x, y, 0));
        ASSERT_NEAR(depth_image.at(u, v)[0], *depth, 1e-4);
        ++x;
      }
      --y;
    }
  }

 private:
  RenderingSim diagram_;
};

// Verifies the rendered terrain.
GTEST_TEST(RenderingTest, TerrainRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/box.sdf");
  ImageTest dut(sdf,
                Eigen::Vector3d(0., 0., 4.999),
                Eigen::Vector3d(0., M_PI_2, 0.),
                ImageTest::VerifyTerrain);
}

// Verifies the rendered shapes.
GTEST_TEST(RenderingTest, AllShapeRenderingTest) {
  const std::string sdf("/systems/sensors/test/models/all_shapes.sdf");
  ImageTest dut(sdf,
                Eigen::Vector3d(-1., 0., 1),
                Eigen::Vector3d(0., M_PI_2 * 0.5, 0.),
                ImageTest::VerifyAllShapes);
}

// Verifies an exception is thrown if a link has more than two visuals.
GTEST_TEST(RenderingTest, MultipleVisualsTest) {
  RenderingSim diagram;
  // The following SDF includes a link that has more than two visuals.
  const std::string sdf("/systems/sensors/test/models/bad.sdf");
  EXPECT_THROW(diagram.Init(GetDrakePath() + sdf,
                            Eigen::Vector3d(0., 0., 1.),
                            Eigen::Vector3d(0., 0., 0.)),
               std::runtime_error);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
