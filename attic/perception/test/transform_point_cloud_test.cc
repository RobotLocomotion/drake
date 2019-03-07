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

const char kOtherFrameName[] = "destination_frame";

class TransformPointCloudTest : public ::testing::Test {
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

  static void AddBody(RigidBodyTree<double>* tree,
                      const Vector3<double>& position,
                      const Vector3<double>& rpy) {
    std::unique_ptr<RigidBody<double>> body =
        std::make_unique<RigidBody<double>>();
    body->set_name("body");
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    Isometry3<double> body_transform;
    body_transform = Eigen::Translation3d(position) *
                     (Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
    body->add_joint(&tree->world(), std::make_unique<RollPitchYawFloatingJoint>(
                                        "base", body_transform));

    tree->add_rigid_body(std::move(body));
  }

 protected:
  void SetUp() override {
    const Vector3<double> kWorldToBodyRpy(M_PI_2, -0.236, M_PI_4);
    const Vector3<double> kWorldToBodyP(-1.4, 0.3, 2.8);

    tree_ = std::make_unique<RigidBodyTree<double>>();

    AddBody(tree_.get(), kWorldToBodyP, kWorldToBodyRpy);
  }

  void InitWithWorldFrame() {
    tree_->compile();
    frame_ = tree_->findFrame("body");
    transformer_ = std::make_unique<TransformPointCloud>(
        *tree_.get(), frame_->get_frame_index());
    InitPortsAndContext();
  }

  void InitWithArbitraryFrames() {
    const Vector3<double> kFrameToBodyRpy(0.0, 0.0, 0.0);
    const Vector3<double> kFrameToBodyP(0.0, 0.0, 0.0);

    std::shared_ptr<RigidBodyFrame<double>> dest_frame =
        std::make_shared<RigidBodyFrame<double>>(
            kOtherFrameName, tree_->FindBody("body"), kFrameToBodyP,
            kFrameToBodyRpy);
    tree_->addFrame(dest_frame);
    tree_->compile();
    frame_ = tree_->findFrame("body");
    transformer_ = std::make_unique<TransformPointCloud>(
        *tree_.get(), frame_->get_frame_index(), dest_frame->get_frame_index());
    InitPortsAndContext();
  }

  void CheckOutput(const std::string& dest_frame_name) {
    // Calculate the system's actual output.
    const PointCloud& output_cloud =
        transformer_->point_cloud_output_port().Eval<PointCloud>(*context_);

    // Calculate the system's expected output.
    // The transform below uses `float` because the point cloud uses `float` as
    // the numerical representation.
    const KinematicsCache<double> cache =
        tree_->doKinematics(state_input_.head(tree_->get_num_positions()));
    const Isometry3<double> isom = tree_->relativeTransform(
        cache, tree_->findFrame(dest_frame_name)->get_frame_index(),
        frame_->get_frame_index());
    const Eigen::Isometry3f isomf(isom);
    const Matrix3X<float> expected_output = isomf * cloud_input_->xyzs();

    // The tolerance used here has this value because the point cloud uses
    // `float` as the numerical representation.
    EXPECT_TRUE(CompareMatrices(output_cloud.xyzs(), expected_output,
                                10.0f * std::numeric_limits<float>::epsilon()));
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::shared_ptr<RigidBodyFrame<double>> frame_;
  std::unique_ptr<TransformPointCloud> transformer_;
  std::unique_ptr<systems::Context<double>> context_;
  VectorX<double> state_input_;
  std::unique_ptr<PointCloud> cloud_input_;

 private:
  void InitPortsAndContext() {
    const Vector3<float> kMin(-10.0, -20.0, -30.0);
    const Vector3<float> kMax(10.0, 20.0, 30.0);
    const int kNumPoints = 5;

    Matrix3X<float> input_data = GenerateBoundedSample(kMin, kMax, kNumPoints);
    cloud_input_ = std::make_unique<PointCloud>(kNumPoints);
    cloud_input_->mutable_xyzs() = input_data;

    state_input_ = VectorX<double>::Zero(tree_->get_num_positions() +
                                         tree_->get_num_velocities());
    state_input_.head(tree_->get_num_positions()) << 0.3, -0.4, 2.3, 0, 0, 0;

    context_ = transformer_->CreateDefaultContext();
    context_->FixInputPort(
        0, AbstractValue::Make<PointCloud>(*cloud_input_.get()));
    context_->FixInputPort(1, state_input_);
  }
};

// Verifies that the point cloud is correctly transformed from an arbitrary
// frame to another arbitrary frame.
TEST_F(TransformPointCloudTest, TransformToArbitraryFrame) {
  InitWithArbitraryFrames();
  CheckOutput(kOtherFrameName);
}

// Verifies that the point cloud is transformed correctly from an arbitrary
// frame to the world frame.
TEST_F(TransformPointCloudTest, TransformToWorldFrame) {
  InitWithWorldFrame();
  CheckOutput("world");
}

}  // namespace
}  // namespace perception
}  // namespace drake
