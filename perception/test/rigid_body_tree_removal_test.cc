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
  const Eigen::Vector4d material(0.5, 0.0, 0.5, 1.0);

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
  drake::multibody::collision::Element body_collision(
      shape, Isometry3<double>::Identity());
  body->AddCollisionElement("", &body_collision);

  auto joint =
      std::make_unique<FixedJoint>("box_joint", joint_transform);
  body->add_joint(&tree->world(), std::move(joint));

  tree->add_rigid_body(std::move(body));
}

void AddCameraBody(RigidBodyTree<double>* tree) {
  const Eigen::Vector3d kPosition(0., 0., 2.);
  const Eigen::Vector3d kOrientation(0., M_PI_2, 0.);

  auto body = std::make_unique<RigidBody<double>>();
  body->set_name("rgbd_camera");
  body->set_mass(1.0);
  body->set_spatial_inertia(Matrix6<double>::Identity());
  
  const Eigen::Vector3d size(0.02, 0.02, 0.02);
  const DrakeShapes::Box shape(size);
  const Eigen::Vector4d material(0.9, 0.0, 0.0, 0.6);
  const DrakeShapes::VisualElement visual_element(
      shape, Eigen::Isometry3d::Identity(), material);
  body->AddVisualElement(visual_element);
  
  Eigen::Isometry3d joint_transform = Eigen::Translation3d(kPosition)
    * (Eigen::AngleAxisd(kOrientation(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(kOrientation(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(kOrientation(2), Eigen::Vector3d::UnitZ()));
  auto joint = std::make_unique<FixedJoint>("camera_joint", joint_transform);
  body->add_joint(&tree->world(), std::move(joint));
  tree->add_rigid_body(std::move(body));
}

systems::sensors::RgbdCameraDiscrete* AddCamera(
    systems::DiagramBuilder<double>* builder,
    systems::RigidBodyPlant<double>* plant) {
  const double kFovY = M_PI_4;
  const bool kShowWindow = true;
  const double kDepthRangeNear = 0.5;
  const double kDepthRangeFar = 5.;
  const Eigen::Vector3d kPosition(0.0, 0.0, 2.0);
  const Eigen::Vector3d kOrientation(0.0, M_PI_2, 0.0);
  const int kWidth = 640;
  const int kHeight = 480;

  //systems::sensors::RgbdCamera* rgbd_camera =
//      builder->AddSystem<systems::sensors::RgbdCamera>(
//          "rgbd_camera", plant->get_rigid_body_tree(), kPosition, kOrientation,
//          kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow, kWidth, kHeight);
  auto rgbd_camera = builder->AddSystem<systems::sensors::RgbdCameraDiscrete>(
          std::make_unique<systems::sensors::RgbdCamera>("rgbd_camera", plant->get_rigid_body_tree(), kPosition, kOrientation,
          kDepthRangeNear, kDepthRangeFar, kFovY, kShowWindow, kWidth, kHeight), 1./30., false);
  rgbd_camera->set_name("rgbd_camera");

  return rgbd_camera;
}

void BuildFilterScene(systems::DiagramBuilder<double>* builder,
                      RigidBodyTreeRemoval* filter,
                      systems::RigidBodyPlant<double>* plant,
                      std::unique_ptr<RigidBodyTree<double>> tree_ptr) {
  AddCameraBody(tree_ptr.get());
  tree_ptr->compile();
  //tree_ptr->doKinematics(tree_ptr->getZeroConfiguration());
  plant =
      builder->AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_ptr));

  auto camera = AddCamera(builder, plant);

  auto converter =
      builder->AddSystem<DepthImageToPointCloud>(camera->camera().depth_camera_info());

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  
  log()->info("  {}", tree.FindBody("rgbd_camera")->get_name());

  auto transformer =
      builder->AddSystem<TransformPointCloud>(tree,
          tree.findFrame("rgbd_camera")->get_frame_index());

  filter = builder->AddSystem<RigidBodyTreeRemoval>(tree, kCollisionThreshold);

/*  lcm::DrakeLcm lcm;
  systems::DrakeVisualizer visualizer(tree, &lcm);
  visualizer.set_name("visualizer");
  auto context = visualizer.CreateDefaultContext();
  const int vector_size = tree.get_num_positions() + tree.get_num_velocities();
  auto input_data = std::make_unique<systems::BasicVector<double>>(vector_size);
  input_data->set_value(Eigen::VectorXd::Zero(vector_size));
  context->FixInputPort(0, std::move(input_data));
  visualizer.PublishLoadRobot();
  visualizer.Publish(*context.get());*/

  builder->Connect(plant->state_output_port(), camera->state_input_port());
  builder->Connect(camera->depth_image_output_port(),
                   converter->depth_image_input_port());
  builder->Connect(plant->state_output_port(), filter->state_input_port());
  builder->Connect(plant->state_output_port(), transformer->state_input_port());
  builder->Connect(converter->point_cloud_output_port(),
                   transformer->point_cloud_input_port());
  builder->Connect(transformer->point_cloud_output_port(),
                   filter->point_cloud_input_port());

  //builder->ExportInput(camera->state_input_port());
  builder->ExportOutput(converter->point_cloud_output_port());
  builder->ExportOutput(filter->point_cloud_output_port());
  builder->ExportOutput(camera->color_image_output_port());
  builder->ExportOutput(camera->depth_image_output_port());
}

// Tests that the visual geometries corresponding to rigid bodies in a
// RigidBodyTree are removed completely from the point cloud.
GTEST_TEST(RigidBodyTreeRemovalTests, FilterFloatingBoxTest) {
  const Eigen::Vector3d kBoxPosition(0., 0., 0.);
  const Eigen::Vector3d kBoxSize(0.25, 0.25, 0.25);

  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  AddFloatingBoxToTree(tree_ptr.get(), kBoxPosition, kBoxSize);

  //tree_ptr->doKinematics(tree_ptr->getZeroConfiguration());

  systems::DiagramBuilder<double> builder;
  std::unique_ptr<RigidBodyTreeRemoval> filter;
  std::unique_ptr<systems::RigidBodyPlant<double>> plant;
  BuildFilterScene(&builder, filter.get(), plant.get(), std::move(tree_ptr));

  log()->info("Scene built.");

//  const RigidBodyTree<double>& tree_ref = plant->get_rigid_body_tree();
  
//  KinematicsCache<double> cache = tree_ref.CreateKinematicsCache();
//  cache.initialize(tree_ref.getZeroConfiguration());
//  tree_ref.doKinematics(cache);
/*  log()->info("Got ref toh tree.");
  auto q = tree_ref.getZeroConfiguration();
  KinematicsCache<double> cache = tree_ref.doKinematics(q);
  log()->info("Did kinematic with tree.");
  const Isometry3<double> iso = tree_ref.relativeTransform(cache, 0, 1);
  const math::RigidTransform<float> transform(iso.cast<float>());
  log()->info("{}", iso.translation());*/

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();
  
//  context->FixInputPort(0, systems::AbstractValue::Make<math::RigidTransform<float>>(transform));
//  log()->info("{}", iso.translation());
//
  
  //Eigen::VectorXd x = Eigen::VectorXd::Zero(plant->get_rigid_body_tree().get_num_positions() + plant->get_rigid_body_tree().get_num_velocities());
  //context->FixInputPort(0, state);
  //plant->get_rigid_body_tree().doKinematics(x);
  //
  
  log()->info("graphviz of diagram: {}", diagram->GetGraphvizString());
  
  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);
  simulator->set_target_realtime_rate(0.1);
  simulator->Initialize();
  // Run simulation.
  simulator->StepTo(0.3);


  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput();

  diagram->CalcOutput(*context, output.get());
  auto const unfiltered_cloud =
      output->GetMutableData(0)->GetMutableValue<PointCloud>();
  auto const filtered_cloud =
      output->GetMutableData(1)->GetMutableValue<PointCloud>();
  auto const rgb = output->GetMutableData(2)
                       ->GetMutableValue<systems::sensors::ImageRgba8U>();
  auto const depth = output->GetMutableData(3)
                       ->GetMutableValue<systems::sensors::ImageDepth32F>();

  log()->info("rgb: {} x {}, depth: {} x {}, unfiltered cloud: {}, filtered cloud: {}", rgb.width(), 
      rgb.height(), depth.width(), depth.height(), unfiltered_cloud.size(), filtered_cloud.size());

  for (int i = 0; i < depth.height(); i++) {
      for (int j = 0; j < depth.width(); j++) {
        log()->info("x,y: ({}, {}); rgb: ({}, {}, {}); depth: {}", j, i, rgb.at(j, i)[0], rgb.at(j, i)[1], rgb.at(j, i)[2], depth.at(j,i)[0]);
      }
  }

  getchar();

  for (int i = 0; i < filtered_cloud.size(); i++) {
    log()->info("{} ---> {}", unfiltered_cloud.xyz(i).transpose(), filtered_cloud.xyz(i).transpose());
  }

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
      builder.AddSystem<DepthImageToPointCloud>(camera->camera().depth_camera_info());

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
