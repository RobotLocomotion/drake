#include "drake/systems/sensors/rgbd_camera.h"

#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include <Eigen/Dense>

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
    plant_->set_contact_parameters(3000,  // Penetration stiffness
                                   30.0,  // Penetration damping
                                   1.0);  // Friction coefficient
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


// Verifies rendered terrain.
GTEST_TEST(RenderingTest, TerrainRenderingTest) {
  RenderingSim diagram;
  // The following SDF includes a box that is located under the flat terrain.
  // Thus, only the terrain is visible to the RGBD camera.
  const std::string sdf("/systems/sensors/test/models/box.sdf");
  diagram.Init(GetDrakePath() + sdf,
               Eigen::Vector3d(0., 0., 4.999),
               Eigen::Vector3d(0., M_PI_2, 0.));

  std::unique_ptr<systems::Context<double>> context =
      diagram.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram.AllocateOutput(*context);
  systems::Simulator<double> simulator(diagram, std::move(context));
  simulator.Initialize();

  const double duration = 0.03;
  for (double time = 0.; time < duration ; time += 0.01) {
    simulator.StepTo(time);
    diagram.CalcOutput(simulator.get_context(), output.get());

    systems::AbstractValue* mutable_data = output->GetMutableData(0);
    systems::AbstractValue* mutable_data_d = output->GetMutableData(1);
    auto color_image =
        mutable_data->GetMutableValue<sensors::Image<uint8_t>>();
    auto depth_image =
        mutable_data_d->GetMutableValue<sensors::Image<float>>();

    // Verifies by sampling 32 x 24 points instead of 640 x 480 points. The
    // assumption is any defects will be detected by sampling this amount.
    for (int v = 0; v < color_image.height(); v += 20) {
      for (int u = 0; u < color_image.width(); u += 20) {
        ASSERT_NEAR(color_image.at(u, v)[0], 204u, 1.);
        ASSERT_NEAR(color_image.at(u, v)[1], 229u, 1.);
        ASSERT_NEAR(color_image.at(u, v)[2], 255u, 1.);
        ASSERT_NEAR(color_image.at(u, v)[3], 255u, 1.);

        // Assuming depth value provides 0.1mm precision.
        ASSERT_NEAR(depth_image.at(u, v)[0], 4.999f, 1e-4);
      }
    }
  }
}

const float kExpectedDepth[192] = {
1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.93014,
1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.93014, 1.81925, 1.81925,
1.81925, 1.81925, 1.81925, 1.81925, 1.81925, 1.81925, 1.81925, 1.81925, 1.36472,
1.36472, 1.36472, 1.36472, 1.81925, 1.81925, 1.72038, 1.72038, 1.72038, 1.72038,
1.72038, 1.72038, 1.72038, 1.72038, 1.41352, 1.72038, 1.38042, 1.38042, 1.38042,
1.38042, 1.72038, 1.72038, 1.63173, 1.63173, 1.63173, 1.63173, 1.63173, 1.63173,
1.63173, 1.63173, 1.46039, 1.63173, 1.43481, 1.43481, 1.43481, 1.63173, 1.63173,
1.63173, 1.55175, 1.55175, 1.55175, 1.55175, 1.25229, 1.25229, 1.25229, 1.55175,
1.51801, 1.55175, 1.55175, 1.55175, 1.55175, 1.55175, 1.55175, 1.55175, 1.47926,
1.47926, 1.47926, 1.47926, 1.24474, 1.24474, 1.24474, 1.47926, 1.47926, 1.47926,
1.47926, 1.47926, 1.47926, 1.47926, 1.47926, 1.47926, 1.41322, 1.41322, 1.41322,
1.41322, 1.27349, 1.27349, 1.27349, 1.41322, 1.13369, 1.13791, 1.41322, 1.41322,
1.41322, 1.41322, 1.41322, 1.41322, 1.35283, 1.35283, 1.35283, 1.35283, 1.35283,
1.35283, 1.35283, 1.13405, 1.10639, 1.10888, 1.14946, 1.35283, 1.35283, 1.35283,
1.35283, 1.35283, 1.2974, 1.2974, 1.2974, 1.2974, 1.2974, 1.2974, 1.2974,
1.13922, 1.10854, 1.11134, 1.15869, 1.2974, 1.2974, 1.2974, 1.2974, 1.2974,
1.24633, 1.24633, 1.24633, 1.24633, 1.24633, 1.24633, 1.24633, 1.24633, 1.15248,
1.16167, 1.24633, 1.24633, 1.24633, 1.24633, 1.24633, 1.24633, 1.19913, 1.19913,
1.19913, 1.19913, 1.19913, 1.19913, 1.19913, 1.19913, 1.19913, 1.19913, 1.19913,
1.19913, 1.19913, 1.19913, 1.19913, 1.19913, 1.15537, 1.15537, 1.15537, 1.15537,
1.15537, 1.15537, 1.15537, 1.15537, 1.15537, 1.15537, 1.15537, 1.15537, 1.15537,
1.15537, 1.15537, 1.15537};

struct ColorPixel {
  uint8_t b;
  uint8_t g;
  uint8_t r;
  uint8_t a;
};

typedef ColorPixel P;

const ColorPixel kExpectedColor[192] = {};

// Verifies rendered terrain.
GTEST_TEST(RenderingTest, AllShapeRenderingTest) {
  RenderingSim diagram;
  // The following SDF includes a box that is located under the flat terrain.
  // Thus, only the terrain is visible to the RGBD camera.
  const std::string sdf("/systems/sensors/test/models/all_shapes.sdf");
  diagram.Init(GetDrakePath() + sdf,
               Eigen::Vector3d(-1., 0., 1.),
               Eigen::Vector3d(0., M_PI_4, 0.));

  std::unique_ptr<systems::Context<double>> context =
      diagram.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram.AllocateOutput(*context);
  systems::Simulator<double> simulator(diagram, std::move(context));
  simulator.Initialize();

  const double duration = 0.01;
  for (double time = 0.; time < duration ; time += 0.01) {
    simulator.StepTo(time);
    diagram.CalcOutput(simulator.get_context(), output.get());

    systems::AbstractValue* mutable_data = output->GetMutableData(0);
    systems::AbstractValue* mutable_data_d = output->GetMutableData(1);
    auto color_image =
        mutable_data->GetMutableValue<sensors::Image<uint8_t>>();
    auto depth_image =
        mutable_data_d->GetMutableValue<sensors::Image<float>>();


    // vtkNer<vtkImageCanvasSource2D> image_source;
    // image_source->SetExtent(extent);

    // Verifies by sampling 32 x 24 points instead of 640 x 480 points. The
    // assumption is any defects will be detected by sampling this amount.
    int i = 0;
    for (int v = 0; v < color_image.height(); v += 40) {
      for (int u = 0; u < color_image.width(); u += 40) {
    //     ASSERT_NEAR(color_image.at(u, v)[0], 204u, 1.);
    //     ASSERT_NEAR(color_image.at(u, v)[1], 229u, 1.);
    //     ASSERT_NEAR(color_image.at(u, v)[2], 255u, 1.);
    //     ASSERT_NEAR(color_image.at(u, v)[3], 255u, 1.);
        std::cout << static_cast<int>(color_image.at(u, v)[0]) << ", ";
        std::cout << static_cast<int>(color_image.at(u, v)[1]) << ", ";
        std::cout << static_cast<int>(color_image.at(u, v)[2]) << ", ";
        std::cout << static_cast<int>(color_image.at(u, v)[3]) << ", ";
        // std::cout << i << ": ";


        // Assuming depth value provides 0.1mm precision.
        ASSERT_NEAR(depth_image.at(u, v)[0], kExpectedDepth[i], 1e-4);
        ++i;
      }
    }
  }
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
