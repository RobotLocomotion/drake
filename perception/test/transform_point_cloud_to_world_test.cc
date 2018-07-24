#include "drake/perception/transform_point_cloud_to_world.h"

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

const Vector3<double> kFrameToBodyRpy(M_PI_4, -M_PI_2, 0.543);
const Vector3<double> kFrameToBodyP(-0.3, 5.4, -2.7);

const Vector3<double> kWorldToBodyRpy(M_PI, 0.236, M_PI_4);
const Vector3<double> kWorldToBodyP(-1.4, 0.3, 2.8);

class FixedTransformPointCloudTest : public ::testing::Test {
 public:
  static Matrix3X<float> GenerateBoundedSample(const Vector3<float>& min,
                                               const Vector3<float>& max,
                                               int num_cols) {
    Matrix3X<float> return_matrix = Matrix3X<float>::Zero(3, num_cols);
    const Vector3<float> increment = (max - min) / static_cast<float>(num_cols);

    for (int i = 0; i < num_cols; ++i) {
      return_matrix.col(i) = increment * static_cast<float>(i);
    }

    return return_matrix;
  }

 protected:
  void SetUp() override {
    std::unique_ptr<RigidBodyTree<double>> tree =
        std::make_unique<RigidBodyTree<double>>();

    std::unique_ptr<RigidBody<double>> body =
        std::make_unique<RigidBody<double>>();
    body->set_name("body");
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    Isometry3<double> body_transform;
    body_transform =
        Eigen::Translation3d(kWorldToBodyP) *
        (Eigen::AngleAxisd(kWorldToBodyRpy(0), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(kWorldToBodyRpy(1), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(kWorldToBodyRpy(2), Eigen::Vector3d::UnitZ()));
    body->add_joint(&tree->world(), std::make_unique<RollPitchYawFloatingJoint>(
                                        "base", body_transform));

    RigidBody<double>* b = tree->add_rigid_body(std::move(body));

    frame_ = std::make_shared<RigidBodyFrame<double>>("frame", b,
        kFrameToBodyP, kFrameToBodyRpy);

    tree->addFrame(frame_);
    tree->compile();

    plant_ = std::make_unique<systems::RigidBodyPlant<double>>(std::move(tree));

    transformer_ = std::make_unique<TransformPointCloudToWorld>(
        plant_->get_rigid_body_tree(), *frame_);
    context_ = transformer_->CreateDefaultContext();
    output_ = transformer_->point_cloud_output_port().Allocate();
  }

  std::shared_ptr<RigidBodyFrame<double>> frame_;
  std::unique_ptr<systems::RigidBodyPlant<double>> plant_;
  std::unique_ptr<TransformPointCloudToWorld> transformer_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::AbstractValue> output_;
};

// Verifies that the system applies the transform correctly to the point cloud.
TEST_F(FixedTransformPointCloudTest, ApplyTransformTest) {
  const Vector3<float> kMin(-10.0, -20.0, -30.0);
  const Vector3<float> kMax(10.0, 20.0, 30.0);
  const int kNumPoints = 5;

  VectorX<double> state(transformer_->state_input_port().size());
  state << 0.3, -0.4, 2.3, 0, 0, 0, 1;

  MatrixX<float> test_data =
      FixedTransformPointCloudTest::GenerateBoundedSample(kMin, kMax,
                                                          kNumPoints);
  PointCloud cloud(kNumPoints);
  cloud.mutable_xyzs() = test_data;

  context_->FixInputPort(0, systems::AbstractValue::Make<PointCloud>(cloud));
  context_->FixInputPort(1, state);

  transformer_->point_cloud_output_port().Calc(*context_, output_.get());
  auto output_cloud = output_->GetValueOrThrow<PointCloud>();

  // The rigid transform below uses `float` because the point cloud uses
  // `float` as the numerical representation.
  const KinematicsCache<double> cache =
      plant_->get_rigid_body_tree().doKinematics(
          state.head(plant_->get_num_positions()));
  const Isometry3<double> isom =
      plant_->get_rigid_body_tree().CalcFramePoseInWorldFrame(cache, *frame_);
  math::RigidTransform<float> transform(isom.cast<float>());
  Matrix4X<float> test_data_homogeneous(4, kNumPoints);
  test_data_homogeneous.block(0, 0, 3, kNumPoints) = test_data;
  test_data_homogeneous.row(3) = VectorX<float>::Ones(kNumPoints);
  Matrix4X<float> expected_output =
      transform.GetAsMatrix4() * test_data_homogeneous;

  // The tolerance used here has this value because the point cloud uses
  // `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(output_cloud.xyzs(),
                              expected_output.block(0, 0, 3, kNumPoints),
                              16.0f * std::numeric_limits<float>::epsilon()));
}

}  // namespace
}  // namespace perception
}  // namespace drake
