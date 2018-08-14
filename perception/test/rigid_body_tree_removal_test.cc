#include "drake/perception/rigid_body_tree_removal.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/perception/transform_point_cloud.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace perception {
namespace {

const double kCollisionThreshold = 0.01;
const Eigen::Vector3d kPosition(0.0, 0.0, 2.0);
const Eigen::Vector3d kOrientation(0.0, M_PI_2, 0.0);
const double kFovY = M_PI_4;
const bool kShowWindow = false;
const double kDepthRangeNear = 0.5;
const double kDepthRangeFar = 5.0;
const int kWidth = 640;
const int kHeight = 480;

class RigidBodyTreeRemovalTest : public ::testing::Test {
 public:
  // Checks if the point cloud `cloud` contains the point `x`.
  static bool ContainsPoint(const PointCloud& cloud, const Vector3<float>& x) {
    for (int i = 0; i < cloud.size(); i++) {
      if (cloud.xyz(i)(0) == x(0) && cloud.xyz(i)(1) == x(1) &&
          cloud.xyz(i)(2) == x(2)) {
        return true;
      }
    }
    return false;
  }

  // Calculates the output for a RigidBodyTreeRemoval filter given fixed inputs.
  static PointCloud CreateFilterAndCalcOutput(
      const PointCloud& cloud_input, const RigidBodyTree<double>& tree,
      const VectorX<double>& state_input) {
    auto filter =
        std::make_unique<RigidBodyTreeRemoval>(tree, kCollisionThreshold);

    auto context = filter->CreateDefaultContext();
    context->FixInputPort(
        filter->point_cloud_input_port().get_index(),
        systems::AbstractValue::Make<PointCloud>(cloud_input));
    context->FixInputPort(filter->state_input_port().get_index(), state_input);

    auto output = filter->point_cloud_output_port().Allocate();
    filter->point_cloud_output_port().Calc(*context, output.get());
    auto filtered_cloud = output->GetValueOrThrow<perception::PointCloud>();

    return filtered_cloud;
  }

 protected:
  // Builds the initial diagram that consists of the following systems:
  // PassThrough, RgbdCamera, DepthImageToPoint, TransformPointCloud. The
  // three latter systems are connected sequentially, while the former is
  // connected to both the RgbdCamera and the TransformPointCloud system.
  void SetUp() override {
    tree_ = std::make_unique<RigidBodyTree<double>>();

    const Eigen::Vector3d kBoxPosition(0., 0., 0.);
    const Eigen::Vector3d kBoxSize(0.25, 0.25, 0.25);
    AddBox(tree_.get(), kBoxPosition, kBoxSize);

    AddCameraBody(tree_.get(), kPosition, kOrientation);

    builder_ = std::make_unique<systems::DiagramBuilder<double>>();
  }

  void InitFilterDiagram() {
    auto camera = builder_->AddSystem<systems::sensors::RgbdCamera>(
        std::make_unique<systems::sensors::RgbdCamera>(
            "rgbd_camera", *tree_.get(), kPosition, kOrientation,
            kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow, kWidth,
            kHeight));
    camera->set_name("rgbd_camera");

    auto passthrough =
        builder_->AddSystem(std::make_unique<systems::PassThrough<double>>(
            tree_->get_num_positions() + tree_->get_num_velocities()));

    auto transformer =
        builder_->AddSystem(std::make_unique<TransformPointCloud>(
            *tree_.get(), tree_->findFrame("depth_camera")->get_frame_index()));

    auto converter = builder_->AddSystem<DepthImageToPointCloud>(
        camera->depth_camera_info());

    builder_->Connect(passthrough->get_output_port(),
                      camera->state_input_port());
    builder_->Connect(passthrough->get_output_port(),
                      transformer->state_input_port());

    builder_->Connect(camera->depth_image_output_port(),
                      converter->depth_image_input_port());
    builder_->Connect(converter->point_cloud_output_port(),
                      transformer->point_cloud_input_port());

    builder_->ExportInput(passthrough->get_input_port());
    builder_->ExportOutput(transformer->point_cloud_output_port());

    auto filter = builder_->AddSystem<RigidBodyTreeRemoval>(
        *tree_.get(), kCollisionThreshold);

    builder_->Connect(passthrough->get_output_port(),
                      filter->state_input_port());
    builder_->Connect(transformer->point_cloud_output_port(),
                      filter->point_cloud_input_port());

    builder_->ExportOutput(filter->point_cloud_output_port());
  }

  void InitTransformerDiagram() {
    auto camera = builder_->AddSystem<systems::sensors::RgbdCamera>(
        std::make_unique<systems::sensors::RgbdCamera>(
            "rgbd_camera", *tree_.get(), kPosition, kOrientation,
            kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow, kWidth,
            kHeight));
    camera->set_name("rgbd_camera");

    auto passthrough =
        builder_->AddSystem(std::make_unique<systems::PassThrough<double>>(
            tree_->get_num_positions() + tree_->get_num_velocities()));

    auto transformer =
        builder_->AddSystem(std::make_unique<TransformPointCloud>(
            *tree_.get(), tree_->findFrame("depth_camera")->get_frame_index()));

    auto converter = builder_->AddSystem<DepthImageToPointCloud>(
        camera->depth_camera_info());

    builder_->Connect(passthrough->get_output_port(),
                      camera->state_input_port());
    builder_->Connect(passthrough->get_output_port(),
                      transformer->state_input_port());

    builder_->Connect(camera->depth_image_output_port(),
                      converter->depth_image_input_port());
    builder_->Connect(converter->point_cloud_output_port(),
                      transformer->point_cloud_input_port());

    builder_->ExportInput(passthrough->get_input_port());
    builder_->ExportOutput(transformer->point_cloud_output_port());
  }

  // Builds the diagram and initializes the context.
  void BuildDiagramAndInitContext() {
    diagram_ = builder_->Build();

    context_ = diagram_->CreateDefaultContext();

    state_input_ = tree_->getZeroConfiguration();
    context_->FixInputPort(0, state_input_);

    output_ = diagram_->AllocateOutput();
  }

  // Calculates the expected output of the filter.
  PointCloud CalcExpectedOutput(const PointCloud& cloud) {
    const std::vector<size_t> indices = CollidingPoints(cloud, tree_.get());
    PointCloud cloud_out(cloud.size() - indices.size());
    int j = 0;
    for (int i = 0; i < cloud.size(); i++) {
      if (std::find(indices.begin(), indices.end(), i) == indices.end()) {
        cloud_out.mutable_xyz(j) = cloud.xyz(i);
        j++;
      }
    }
    return cloud_out;
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<systems::DiagramBuilder<double>> builder_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  VectorX<double> state_input_;

 private:
  // Adds a camera body to the RigidBodyTree. Required to retrieve the correct
  // transform between the points in the depth camera frame and the `world`
  // frame.
  void AddCameraBody(RigidBodyTree<double>* tree,
                     const Vector3<double>& position,
                     const Vector3<double>& orientation) {
    auto camera_body = std::make_unique<RigidBody<double>>();
    camera_body->set_name("rgbd_camera");
    Eigen::Isometry3d joint_transform =
        Eigen::Translation3d(position) *
        (Eigen::AngleAxisd(orientation(0), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(orientation(1), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(orientation(2), Eigen::Vector3d::UnitZ()));
    auto joint = std::make_unique<FixedJoint>("camera_joint", joint_transform);
    camera_body->add_joint(&tree->world(), std::move(joint));

    // Transform from camera body frame to depth camera frame. Required to find
    // the correct transformation between the points in a point cloud obtained
    // from an RgbdCamera and the `world` frame.
    Eigen::Isometry3d X_BD;
    X_BD.matrix() << 0., 0., 1., 0., -1., 0., 0., 0.02, 0., -1., 0., 0., 0., 0.,
        0., 1.;
    tree->addFrame(std::make_shared<RigidBodyFrame<double>>(
        "depth_camera", camera_body.get(), X_BD));

    tree->add_rigid_body(std::move(camera_body));
  }

  // Adds a RigidBody with a box shape to the RigidBodyTree at a fixed location.
  void AddBox(RigidBodyTree<double>* tree, const Eigen::Vector3d& position,
              const Eigen::Vector3d& size) {
    auto body = std::make_unique<RigidBody<double>>();
    body->set_name("box");
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const DrakeShapes::Box shape(size);
    const Eigen::Vector4d material(0.5, 0.0, 0.5, 1.0);
    const DrakeShapes::VisualElement visual_element(
        shape, Eigen::Isometry3d::Identity(), material);
    body->AddVisualElement(visual_element);

    Eigen::Isometry3d joint_transform;
    {
      const drake::math::RotationMatrix<double> kRotIdentity;
      joint_transform.matrix() << kRotIdentity.matrix(), position, 0, 0, 0, 1;
    }
    auto joint = std::make_unique<FixedJoint>("box_joint", joint_transform);
    body->add_joint(&tree->world(), std::move(joint));

    RigidBody<double>* body_in_tree = tree->add_rigid_body(std::move(body));

    drake::multibody::collision::Element collision(
        shape, Isometry3<double>::Identity());
    tree->addCollisionElement(collision, *body_in_tree, "default");
  }

  // Finds the points in `cloud` that collide with some body in `tree`.
  std::vector<size_t> CollidingPoints(const PointCloud& cloud,
                                      RigidBodyTree<double>* tree) {
    KinematicsCache<double> kinematics_cache = tree->doKinematics(state_input_);
    std::vector<Eigen::Vector3d> points;
    points.resize(cloud.size());
    for (int i = 0; i < cloud.size(); i++) {
      points[i] = cloud.xyz(i).cast<double>();
    }
    std::vector<size_t> indices =
        tree->collidingPoints(kinematics_cache, points, kCollisionThreshold);
    return indices;
  }
};

// Verifies that the visual geometries of a RigidBody contained in a
// RigidBodyTree are completely removed from the point cloud.
TEST_F(RigidBodyTreeRemovalTest, RemoveBoxTest) {
  tree_->compile();
  InitFilterDiagram();
  BuildDiagramAndInitContext();

  diagram_->CalcOutput(*context_, output_.get());
  const PointCloud unfiltered_cloud =
      output_->GetMutableData(0)->GetMutableValue<PointCloud>();
  const PointCloud filtered_cloud =
      output_->GetMutableData(1)->GetMutableValue<PointCloud>();

  const PointCloud expected_cloud = CalcExpectedOutput(unfiltered_cloud);

  EXPECT_EQ(filtered_cloud.size(), expected_cloud.size());

  // The tolerance used here has this value because the point cloud uses
  // `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(filtered_cloud.xyzs(), expected_cloud.xyzs(),
                              10.0f * std::numeric_limits<float>::epsilon()));
}

// Verifies that points which do not belong to visual geometries of the rigid
// bodies contained in a RigidBodyTree are kept.
TEST_F(RigidBodyTreeRemovalTest, KeepPointsTest) {
  tree_->compile();
  InitTransformerDiagram();
  BuildDiagramAndInitContext();

  // Obtain a point cloud that has been transformed to the "world" frame.
  diagram_->CalcOutput(*context_, output_.get());
  const PointCloud unfiltered_cloud =
      output_->GetMutableData(0)->GetMutableValue<PointCloud>();

  // Add three points to the point cloud. These points should remain after
  // filtering.
  const Eigen::Vector3f point1(0.3, 0., 0.);
  const Eigen::Vector3f point2(0., -0.42, 0.);
  const Eigen::Vector3f point3(0., 0., 0.35);
  const int size = unfiltered_cloud.size();
  PointCloud cloud_with_extra_points(size);
  cloud_with_extra_points.resize(size + 3);
  cloud_with_extra_points.mutable_xyz(size) = point1;
  cloud_with_extra_points.mutable_xyz(size + 1) = point2;
  cloud_with_extra_points.mutable_xyz(size + 2) = point3;

  const PointCloud filtered_cloud = CreateFilterAndCalcOutput(
      cloud_with_extra_points, *tree_.get(), state_input_);

  const PointCloud expected_cloud = CalcExpectedOutput(cloud_with_extra_points);

  EXPECT_EQ(filtered_cloud.size(), expected_cloud.size());

  // The tolerance used here has this value because the point cloud uses
  // `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(filtered_cloud.xyzs(), expected_cloud.xyzs(),
                              10.0f * std::numeric_limits<float>::epsilon()));

  EXPECT_TRUE(ContainsPoint(filtered_cloud, point1));
  EXPECT_TRUE(ContainsPoint(filtered_cloud, point2));
  EXPECT_TRUE(ContainsPoint(filtered_cloud, point3));
}

}  // namespace
}  // namespace perception
}  // namespace drake
