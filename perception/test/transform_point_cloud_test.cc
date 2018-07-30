#include "drake/perception/transform_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace perception {
namespace {

const Vector3<double> kFrameToBodyRpy(M_PI_4, -M_PI_4, M_PI_2);
const Vector3<double> kFrameToBodyP(-0.3, 5.4, -2.7);

const Vector3<double> kWorldToBodyRpy(M_PI_2, -0.236, M_PI_4);
const Vector3<double> kWorldToBodyP(-1.4, 0.3, 2.8);

const int kWorldFrameIndex = 0;
const Vector3<float> kMin(-10.0, -20.0, -30.0);
const Vector3<float> kMax(10.0, 20.0, 30.0);
const int kNumPoints = 5;

Matrix3X<float> GenerateBoundedSample(const Vector3<float>& min,
                                      const Vector3<float>& max, int num_cols) {
  Matrix3X<float> return_matrix = Matrix3X<float>::Zero(3, num_cols);
  const Vector3<float> increment = (max - min) / static_cast<float>(num_cols);

  for (int i = 0; i < num_cols; ++i) {
    return_matrix.col(i) = increment * static_cast<float>(i);
  }

  return return_matrix;
}

Matrix4X<float> CalcExpectedOutput(const Isometry3<double>& isom,
                                   const MatrixX<float>& data) {
  // The rigid transform below uses `float` because the point cloud uses
  // `float` as the numerical representation.
  math::RigidTransform<double> transform_d(isom);
  Matrix4X<float> transform = transform_d.GetAsMatrix4().cast<float>();
  Matrix4X<float> data_homogeneous(4, data.cols());
  data_homogeneous.block(0, 0, 3, data.cols()) = data;
  data_homogeneous.row(3) = VectorX<float>::Ones(kNumPoints);
  Matrix4X<float> expected_output = transform * data_homogeneous;
  return expected_output;
}

RigidBody<double> AddBodyToTree(RigidBodyTree<double>* tree) {
  RigidBody<double> body;
  body.set_name("body");
  body.set_mass(1.0);
  body.set_spatial_inertia(Matrix6<double>::Identity());

  Isometry3<double> body_transform;
  body_transform =
      Eigen::Translation3d(kWorldToBodyP) *
      (Eigen::AngleAxisd(kWorldToBodyRpy(0), Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(kWorldToBodyRpy(1), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(kWorldToBodyRpy(2), Eigen::Vector3d::UnitZ()));
  body.add_joint(&tree->world(), std::make_unique<RollPitchYawFloatingJoint>(
                                     "base", body_transform));

  return body;
}

// Verifies that the system applies the transform correctly to the point cloud.
GTEST_TEST(TransformPointCloudTest, ApplyTransform) {
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();

  std::unique_ptr<RigidBody<double>> body =
      std::make_unique<RigidBody<double>>(AddBodyToTree(tree.get()));
  tree->add_rigid_body(std::move(body));

  tree->compile();

  std::shared_ptr<RigidBodyFrame<double>> frame = tree->findFrame("body");

  std::unique_ptr<TransformPointCloud> transformer =
      std::make_unique<TransformPointCloud>(*tree.get(), kWorldFrameIndex,
                                            frame->get_frame_index());
  std::unique_ptr<systems::Context<double>> context =
      transformer->CreateDefaultContext();
  std::unique_ptr<systems::AbstractValue> output =
      transformer->point_cloud_output_port().Allocate();

  VectorX<double> state =
      VectorX<double>::Zero(transformer->state_input_port().size());
  state.head(tree->get_num_positions()) << 0.3, -0.4, 2.3, 0, 0, 0, 1;

  MatrixX<float> test_data = GenerateBoundedSample(kMin, kMax, kNumPoints);
  PointCloud cloud(kNumPoints);
  cloud.mutable_xyzs() = test_data;

  context->FixInputPort(0, systems::AbstractValue::Make<PointCloud>(cloud));
  context->FixInputPort(1, state);

  transformer->point_cloud_output_port().Calc(*context, output.get());
  PointCloud output_cloud = output->GetValueOrThrow<PointCloud>();

  // The rigid transform below uses `float` because the point cloud uses
  // `float` as the numerical representation.
  const KinematicsCache<double> cache =
      tree->doKinematics(state.head(tree->get_num_positions()));
  const Isometry3<double> isom =
      tree->relativeTransform(cache, 0, frame->get_frame_index());

  Matrix4X<float> expected_output = CalcExpectedOutput(isom, test_data);

  // The tolerance used here has this value because the point cloud uses
  // `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(output_cloud.xyzs(),
                              expected_output.block(0, 0, 3, kNumPoints),
                              10.0f * std::numeric_limits<float>::epsilon()));
}

// Verifies that the system transforms a point cloud correctly to the world
// frame. The test differs from the previous test in which constructor is
// called for TransformPointCloud.
GTEST_TEST(TransformPointCloudTest, TransformToWorldFrame) {
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();

  std::unique_ptr<RigidBody<double>> body =
      std::make_unique<RigidBody<double>>(AddBodyToTree(tree.get()));
  tree->add_rigid_body(std::move(body));

  tree->compile();

  std::shared_ptr<RigidBodyFrame<double>> frame = tree->findFrame("body");

  std::unique_ptr<TransformPointCloud> transformer =
      std::make_unique<TransformPointCloud>(*tree.get(),
                                            frame->get_frame_index());
  std::unique_ptr<systems::Context<double>> context =
      transformer->CreateDefaultContext();
  std::unique_ptr<systems::AbstractValue> output =
      transformer->point_cloud_output_port().Allocate();

  VectorX<double> state =
      VectorX<double>::Zero(transformer->state_input_port().size());
  state.head(tree->get_num_positions()) << 0.3, -0.4, 2.3, 0, 0, 0, 1;

  MatrixX<float> test_data = GenerateBoundedSample(kMin, kMax, kNumPoints);
  PointCloud cloud(kNumPoints);
  cloud.mutable_xyzs() = test_data;

  context->FixInputPort(0, systems::AbstractValue::Make<PointCloud>(cloud));
  context->FixInputPort(1, state);

  transformer->point_cloud_output_port().Calc(*context, output.get());
  PointCloud output_cloud = output->GetValueOrThrow<PointCloud>();

  // The rigid transform below uses `float` because the point cloud uses
  // `float` as the numerical representation.
  const KinematicsCache<double> cache =
      tree->doKinematics(state.head(tree->get_num_positions()));
  const Isometry3<double> isom =
      tree->relativeTransform(cache, 0, frame->get_frame_index());

  Matrix4X<float> expected_output = CalcExpectedOutput(isom, test_data);

  // The tolerance used here has this value because the point cloud uses
  // `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(output_cloud.xyzs(),
                              expected_output.block(0, 0, 3, kNumPoints),
                              10.0f * std::numeric_limits<float>::epsilon()));
}

}  // namespace
}  // namespace perception
}  // namespace drake
