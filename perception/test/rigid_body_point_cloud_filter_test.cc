#include "drake/perception/rigid_body_point_cloud_filter.h"

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
const Eigen::Vector3d kBoxPosition(0., 0., 0.);
const Eigen::Vector3d kBoxSize(0.2, 0.2, 0.2);
const Eigen::Vector3f point1(0.3, 0., 0.);
const Eigen::Vector3f point2(0., -0.42, 0.);
const Eigen::Vector3f point3(0., 0., 0.35);
const int kPointsPerFace = 10;
const int kFaces = 6;

class RigidBodyPointCloudFilterTest : public ::testing::Test {
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

 protected:
  void SetUp() override {
    tree_ = std::make_unique<RigidBodyTree<double>>();

    AddBox(tree_.get(), kBoxPosition, kBoxSize);
    tree_->compile();

    filter_ = std::make_unique<RigidBodyPointCloudFilter>(*tree_.get(),
                                                          kCollisionThreshold);

    cloud_input_ = std::make_unique<PointCloud>(MakePointCloudFromBox(
        kBoxPosition.cast<float>(), kBoxSize.cast<float>()));

    // Add three points to the point cloud. These points should remain after
    // filtering.
    const int size = cloud_input_->size();
    cloud_input_->resize(size + 3);
    cloud_input_->mutable_xyz(size) = point1;
    cloud_input_->mutable_xyz(size + 1) = point2;
    cloud_input_->mutable_xyz(size + 2) = point3;

    state_input_ = VectorX<double>::Zero(tree_->get_num_positions() +
                                         tree_->get_num_velocities());

    context_ = filter_->CreateDefaultContext();
    context_->FixInputPort(
        filter_->point_cloud_input_port().get_index(),
        systems::AbstractValue::Make<PointCloud>(*cloud_input_.get()));
    context_->FixInputPort(filter_->state_input_port().get_index(),
                           state_input_);

    output_ = filter_->point_cloud_output_port().Allocate();
  }

  // Calculates the expected output of the filter.
  PointCloud CalcExpectedOutput(const PointCloud& cloud) {
    std::vector<size_t> indices = CollidingPoints(cloud, tree_.get());
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
  std::unique_ptr<RigidBodyPointCloudFilter> filter_;
  std::unique_ptr<systems::Context<double>> context_;
  VectorX<double> state_input_;
  std::unique_ptr<PointCloud> cloud_input_;
  std::unique_ptr<systems::AbstractValue> output_;

 private:
  // Adds a RigidBody with a box shape to the RigidBodyTree at a fixed location.
  void AddBox(RigidBodyTree<double>* tree, const Eigen::Vector3d& position,
              const Eigen::Vector3d& size) {
    auto body = std::make_unique<RigidBody<double>>();
    body->set_name("box");
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    const DrakeShapes::Box shape(size);
    const Eigen::Vector4d material(0.0, 0.0, 0.5, 1.0);
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

  // Creates a point cloud from an axis aligned box.
  PointCloud MakePointCloudFromBox(const Eigen::Vector3f& position,
                                   const Eigen::Vector3f& size) {
    PointCloud cloud(kFaces * kPointsPerFace);
    for (int i = 0; i < kFaces; i++) {
      int dim = i / 2;
      int sign = (i + 1) % 2 ? -1 : 1;
      for (int j = 0; j < kPointsPerFace; j++) {
        int idx = i * kPointsPerFace + j;
        cloud.mutable_xyz(idx) = position;
        cloud.mutable_xyz(idx)(dim) += sign * 0.5 * size(dim);
      }
    }
    return cloud;
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

TEST_F(RigidBodyPointCloudFilterTest, RemoveBoxTest) {
  // Calculate the system's actual output.
  filter_->point_cloud_output_port().Calc(*context_, output_.get());
  PointCloud output_cloud = output_->GetValueOrThrow<PointCloud>();

  // Calculate the system's expected output.
  PointCloud expected_cloud = CalcExpectedOutput(*cloud_input_.get());

  EXPECT_EQ(output_cloud.size(), expected_cloud.size());
  EXPECT_GT(output_cloud.size(), 0);
  EXPECT_GT(expected_cloud.size(), 0);

  // The tolerance used here has this value because the point cloud uses
  // `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(output_cloud.xyzs(), expected_cloud.xyzs(),
                              10.0f * std::numeric_limits<float>::epsilon()));

  EXPECT_TRUE(ContainsPoint(output_cloud, point1));
  EXPECT_TRUE(ContainsPoint(output_cloud, point2));
  EXPECT_TRUE(ContainsPoint(output_cloud, point3));
}

}  // namespace
}  // namespace perception
}  // namespace drake
