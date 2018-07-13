#include "drake/perception/rigid_body_tree_removal.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace perception {
namespace {

// Creates a box RigidBody.
RigidBody<double> MakeBox() {
  RigidBody<double> body;
  body.set_name("box_body");
  body.set_mass(1.0);
  body.set_spatial_inertia(Matrix6<double>::Identity());

  const Eigen::Vector3d box_size(0.25, 0.25, 0.25);
  const DrakeShapes::Box shape(box_size);
  const Eigen::Vector4d material(0.3, 0.4, 0.5, 1.0);

  const DrakeShapes::VisualElement visual_element(
      shape, Eigen::Isometry3d::Identity(), material);
  body.AddVisualElement(visual_element);

  return body;
}

// Adds a box with a floating joint to the RigidBodyTree.
void AddFloatingBoxToTree(RigidBodyTree<double>* tree) {
  auto body = std::make_unique<RigidBody<double>>(MakeBox());
  body->set_model_instance_id(tree->add_model_instance());

  Eigen::Isometry3d joint_transform;
  {
    const drake::math::RotationMatrix<double> kRotIdentity;
    const Eigen::Vector3d kXyz(0, 0, 0);
    joint_transform.matrix() << kRotIdentity.matrix(), kXyz, 0, 0, 0, 1;
  }

  auto joint =
      std::make_unique<RollPitchYawFloatingJoint>("box_joint", joint_transform);
  body->add_joint(&tree->world(), std::move(joint));

  tree->add_rigid_body(std::move(body));
}

// Adds a box with a fixed joint to the RigidBodyTree.
void AddFixedBoxToTree(RigidBodyTree<double>* tree) {
  auto body = std::make_unique<RigidBody<double>>(MakeBox());
  body->set_model_instance_id(tree->add_model_instance());

  Eigen::Isometry3d joint_transform;
  {
    const drake::math::RotationMatrix<double> kRotIdentity;
    const Eigen::Vector3d kXyz(0, 0, 0);
    joint_transform.matrix() << kRotIdentity.matrix(), kXyz, 0, 0, 0, 1;
  }

  auto joint = std::make_unique<FixedJoint>("box_joint", joint_transform);
  body->add_joint(&tree->world(), std::move(joint));

  tree->add_rigid_body(std::move(body));
}

systems::sensors::RgbdCamera* AddCamera(
    systems::DiagramBuilder<double>* builder,
    systems::RigidBodyPlant<double>* plant) {
  const double kFovY = M_PI_4;
  const bool kShowWindow = false;
  const double kDepthRangeNear = 0.5;
  const double kDepthRangeFar = 5.;
  const Eigen::Vector3d kPosition(0., 0., 1.);
  const Eigen::Vector3d kOrientation(0., M_PI_2, 0.);
  const int kWidth = 640;
  const int kHeight = 480;

  systems::sensors::RgbdCamera* rgbd_camera =
      builder->AddSystem<systems::sensors::RgbdCamera>(
          "rgbd_camera", plant->get_rigid_body_tree(), kPosition, kOrientation,
          kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow, kWidth, kHeight);
  rgbd_camera->set_name("rgbd_camera");

  return rgbd_camera;
}

GTEST_TEST(RigidBodyTreeRemovalTests, FilterFloatingBoxTest) {
  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  AddFloatingBoxToTree(tree_ptr.get());
  tree_ptr->compile();
  systems::DiagramBuilder<double> builder;
  auto plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_ptr));

  auto camera = AddCamera(&builder, plant);

  auto converter =
      builder.AddSystem<DepthImageToPointCloud>(camera->depth_camera_info());

  auto filter =
      builder.AddSystem<RigidBodyTreeRemoval>(plant->get_rigid_body_tree());

  builder.Connect(plant->state_output_port(), camera->state_input_port());
  builder.Connect(camera->depth_image_output_port(),
                  converter->depth_image_input_port());
  builder.Connect(plant->state_output_port(), filter->state_input_port());
  builder.Connect(converter->point_cloud_output_port(),
                  filter->point_cloud_input_port());

  builder.ExportOutput(converter->point_cloud_output_port());
  builder.ExportOutput(filter->point_cloud_output_port());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();

  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput();

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  Eigen::VectorXd q = tree.getZeroConfiguration();

  diagram->CalcOutput(*context, output.get());
  auto const unfiltered_cloud =
      output->GetMutableData(0)->GetMutableValue<PointCloud>();
  auto const filtered_cloud =
      output->GetMutableData(1)->GetMutableValue<PointCloud>();

  EXPECT_GT(unfiltered_cloud.size(), 0);
  EXPECT_EQ(filtered_cloud.size(), 0);
}

GTEST_TEST(RigidBodyTreeRemovalTests, FilterFixedBoxTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  AddFixedBoxToTree(tree.get());
  tree->compile();
  systems::DiagramBuilder<double> builder;
  auto plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));

  auto camera = AddCamera(&builder, plant);

  RigidBodyTreeRemoval filter(plant->get_rigid_body_tree());

  builder.Connect(plant->state_output_port(), camera->state_input_port());
}

}  // namespace
}  // namespace perception
}  // namespace drake
