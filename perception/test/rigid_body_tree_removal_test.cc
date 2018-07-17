#include "drake/perception/rigid_body_tree_removal.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace perception {
namespace {

const double kCollisionThreshold = 0.01;

// Creates a box RigidBody.
RigidBody<double> MakeBox(const Eigen::Vector3d& size) {
  RigidBody<double> body;
  body.set_name("box_body");
  body.set_mass(1.0);
  body.set_spatial_inertia(Matrix6<double>::Identity());

  const DrakeShapes::Box shape(size);
  const Eigen::Vector4d material(0.9, 0.0, 0.0, 1.0);

  const DrakeShapes::VisualElement visual_element(
      shape, Eigen::Isometry3d::Identity(), material);
  body.AddVisualElement(visual_element);

  return body;
}

// Adds a box with a floating joint to the RigidBodyTree.
void AddFloatingBoxToTree(RigidBodyTree<double>* tree,
                          const Eigen::Vector3d& position,
                          const Eigen::Vector3d& size) {
  auto body = std::make_unique<RigidBody<double>>(MakeBox(size));
  body->set_model_instance_id(tree->add_model_instance());

  Eigen::Isometry3d joint_transform;
  {
    const drake::math::RotationMatrix<double> kRotIdentity;
    joint_transform.matrix() << kRotIdentity.matrix(), position, 0, 0, 0, 1;
  }

  const DrakeShapes::Box shape(size);
  drake::multibody::collision::Element body_collision(shape, Isometry3<double>::Identity());
  body->AddCollisionElement("", &body_collision);

  auto joint =
      std::make_unique<RollPitchYawFloatingJoint>("box_joint", joint_transform);
  body->add_joint(&tree->world(), std::move(joint));

  tree->add_rigid_body(std::move(body));
}

systems::sensors::RgbdCamera* AddCamera(
    systems::DiagramBuilder<double>* builder,
    systems::RigidBodyPlant<double>* plant) {
  const double kFovY = M_PI_4;
  const bool kShowWindow = true;
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

void BuildFilterScene(systems::DiagramBuilder<double>* builder,
                      RigidBodyTreeRemoval* filter,
                      std::unique_ptr<RigidBodyTree<double>> tree_ptr) {
  tree_ptr->compile();
  auto plant =
      builder->AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_ptr));

  auto camera = AddCamera(builder, plant);

  auto converter =
      builder->AddSystem<DepthImageToPointCloud>(camera->depth_camera_info());

  filter = builder->AddSystem<RigidBodyTreeRemoval>(
      plant->get_rigid_body_tree(), kCollisionThreshold);

  lcm::DrakeLcm lcm;
  systems::DrakeVisualizer visualizer(plant->get_rigid_body_tree(), &lcm);
  visualizer.set_name("visualizer");
  auto context = visualizer.CreateDefaultContext();
  const int vector_size = plant->get_rigid_body_tree().get_num_positions() +
                          plant->get_rigid_body_tree().get_num_velocities();
  auto input_data = std::make_unique<systems::BasicVector<double>>(vector_size);
  input_data->set_value(Eigen::VectorXd::Zero(vector_size));
  context->FixInputPort(0, std::move(input_data));
  visualizer.PublishLoadRobot();
  visualizer.Publish(*context.get());

  builder->Connect(plant->state_output_port(), camera->state_input_port());
  builder->Connect(camera->depth_image_output_port(),
                   converter->depth_image_input_port());
  builder->Connect(plant->state_output_port(), filter->state_input_port());
  builder->Connect(converter->point_cloud_output_port(),
                   filter->point_cloud_input_port());

  builder->ExportOutput(converter->point_cloud_output_port());
  builder->ExportOutput(filter->point_cloud_output_port());
  builder->ExportOutput(camera->color_image_output_port());
}

// Tests that the visual geometries corresponding to rigid bodies in a
// RigidBodyTree are removed completely from the point cloud.
GTEST_TEST(RigidBodyTreeRemovalTests, FilterFloatingBoxTest) {
  const Eigen::Vector3d kBoxPosition(0., 0., 0.);
  const Eigen::Vector3d kBoxSize(0.25, 0.25, 0.25);

  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  AddFloatingBoxToTree(tree_ptr.get(), kBoxPosition, kBoxSize);

  systems::DiagramBuilder<double> builder;
  std::unique_ptr<RigidBodyTreeRemoval> filter;
  BuildFilterScene(&builder, filter.get(), std::move(tree_ptr));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();

  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput();

  diagram->CalcOutput(*context, output.get());
  auto const unfiltered_cloud =
      output->GetMutableData(0)->GetMutableValue<PointCloud>();
  auto const filtered_cloud =
      output->GetMutableData(1)->GetMutableValue<PointCloud>();
  auto const rgb = output->GetMutableData(2)
                       ->GetMutableValue<systems::sensors::ImageRgba8U>();

/*  for (int i = 0; i < rgb.height(); i++) {
    for (int j = 0; j < rgb.width(); j++) {
      log()->info("{}", rgb.at(j, i));
    }
  }*/

  //  for(int i=0; i < unfiltered_cloud.size(); i++)
  //    log()->info("{}", unfiltered_cloud.xyz(i).transpose());

  EXPECT_GT(unfiltered_cloud.size(), 0);
  EXPECT_EQ(filtered_cloud.size(), 0);
}

// Tests that points which do not belong to visual geometries corresponding to
// rigid bodies in a RigidBodyTree are kept.
GTEST_TEST(RigidBodyTreeRemovalTests, KeepPointsTest) {
  const Eigen::Vector3d kBoxPosition(0., 0., 0.);
  const Eigen::Vector3d kBoxSize(0.25, 0.25, 0.25);
  const Eigen::Vector3f point1(0.3, 0., 0.);
  const Eigen::Vector3f point2(0., -0.42, 0.);
  const Eigen::Vector3f point3(0., 0., 0.35);

  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  AddFloatingBoxToTree(tree_ptr.get(), kBoxPosition, kBoxSize);
  tree_ptr->compile();

  systems::DiagramBuilder<double> builder;
  auto plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_ptr));

  auto camera = AddCamera(&builder, plant);

  auto converter =
      builder.AddSystem<DepthImageToPointCloud>(camera->depth_camera_info());

  builder.Connect(plant->state_output_port(), camera->state_input_port());
  builder.Connect(camera->depth_image_output_port(),
                  converter->depth_image_input_port());

  builder.ExportOutput(converter->point_cloud_output_port());
  builder.ExportOutput(plant->state_output_port());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();

  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput();

  diagram->CalcOutput(*context, output.get());
  auto unfiltered_cloud =
      output->GetMutableData(0)->GetMutableValue<PointCloud>();
  auto const tree_state = output->GetMutableVectorData(1);

  EXPECT_GT(unfiltered_cloud.size(), 0);

  PointCloud cloud_point_unsized(unfiltered_cloud);

  PointCloud cloud_point123(unfiltered_cloud);
  cloud_point123.resize(unfiltered_cloud.size() + 3);
  cloud_point123.mutable_xyz(unfiltered_cloud.size()) = point1;
  cloud_point123.mutable_xyz(unfiltered_cloud.size() + 1) = point2;
  cloud_point123.mutable_xyz(unfiltered_cloud.size() + 2) = point3;

  std::unique_ptr<RigidBodyTreeRemoval> filter =
      std::make_unique<RigidBodyTreeRemoval>(plant->get_rigid_body_tree(),
                                             kCollisionThreshold);

  log()->info("{}, {}", unfiltered_cloud.size(), cloud_point123.size());
  log()->info("{}", cloud_point123.xyz(unfiltered_cloud.size() + 1));
  log()->info("{}", cloud_point123.xyz(unfiltered_cloud.size() + 2));
  log()->info("{}", cloud_point123.xyz(unfiltered_cloud.size() + 3));

  std::unique_ptr<systems::Context<double>> filter_context =
      filter->CreateDefaultContext();
  log()->info("Before FixInputPort");
  filter_context->FixInputPort(
      filter->point_cloud_input_port().get_index(),
      systems::AbstractValue::Make<PointCloud>(cloud_point123));
  log()->info("1111111111111111111");
  filter_context->FixInputPort(filter->state_input_port().get_index(),
                               *tree_state);
  log()->info("DDDDDDDD");
  std::unique_ptr<systems::AbstractValue> filter_output =
      filter->point_cloud_output_port().Allocate();
  filter->point_cloud_output_port().Calc(*filter_context, filter_output.get());
  log()->info("EEEEEE");

  auto output_cloud = filter_output->GetValueOrThrow<perception::PointCloud>();
  log()->info("output_cloud: {}", output_cloud.size());
  EXPECT_EQ(output_cloud.size(), 3);
}

}  // namespace
}  // namespace perception
}  // namespace drake
