#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::systems::plants::joints::kFixed;
using drake::systems::plants::joints::kQuaternion;

/**
 * Defines a class that extends Google Test's ::testing::TestWithParam
 * class, which enables parameterized Google Tests. In this case, the parameter
 * is a const char* and it is the file extension of the model files to use in
 * the LoadModelTest unit tests. Specifically, the possible values are "urdf"
 * and "sdf".
 */
class LoadModelTest : public ::testing::TestWithParam<const char*> {
 protected:
  RigidBodySystem rbs;
};

TEST_P(LoadModelTest, TestNoOffset) {
  // Loads the SDF model with zero offset between the model's root frame and
  // the world frame.
  rbs.AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/models/cylindrical_1dof_robot." + GetParam(),
      kQuaternion);

  // Verifies that RigidBodyTree cannot find a link thatn does not exist.
  EXPECT_THROW(rbs.getRigidBodyTree()->FindBody("not_a_link"),
               std::logic_error);

  // Verifies that the world link within the rigid body tree
  // can be found and obtained.
  auto world_body = rbs.getRigidBodyTree()->FindBody("world");
  EXPECT_TRUE(world_body != nullptr);

  // Verifies that the world link does not have a mobilizer joint.
  EXPECT_FALSE(world_body->has_parent_body());

  // Gets the link whose parent joint is called "base".
  auto link1_body = rbs.getRigidBodyTree()->FindChildBodyOfJoint("base");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_EQ(link1_body->get_name(), "link1");

  // Verifies that the transformation from link1's frame to the world's frame
  // is correct. Since the model was not offset from the world, we expect
  // the transformation to be Eigen::Isometry3d::Identity().
  EXPECT_TRUE(link1_body->getJoint().get_transform_to_parent_body().matrix() ==
              Eigen::Isometry3d::Identity().matrix());

  // Gets the link whose parent joint is called "joint1".
  auto link2_body = rbs.getRigidBodyTree()->FindChildBodyOfJoint("joint1");
  EXPECT_TRUE(link2_body != nullptr);
  EXPECT_EQ(link2_body->get_name(), "link2");

  // Verifies that the transformation from link2's frame to link1's frame is
  // correct. From the SDF, the transformation is expected to be
  // x = 0, y = 0, z = 0.6.
  Eigen::Isometry3d T_link2_to_link1;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                    xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.6;
    T_link2_to_link1.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_TRUE(link2_body->getJoint().get_transform_to_parent_body().matrix() ==
              T_link2_to_link1.matrix());
}

TEST_P(LoadModelTest, TestVerticalOffset) {
  // Welds the robot to the world with a Z-offset of 1.0 m, X-offset of 1.0m,
  // and Y-offset of 1.0m.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                    xyz = Eigen::Vector3d::Ones();
    T_model_to_world.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_to_world);

  rbs.AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/models/cylindrical_1dof_robot." + GetParam(),
      kQuaternion, weld_to_frame);

  // Gets the link whose parent joint is called "base".
  auto link1_body = rbs.getRigidBodyTree()->FindChildBodyOfJoint("base");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_EQ(link1_body->get_name(), "link1");

  // Verifies that the transformation from link1's frame to the world's frame
  // is correct. Since the model was offset from the world frame, we expect the
  // transformation of link1_body's frame to the world frame to be equal to
  // T_model_to_world.
  EXPECT_TRUE(link1_body->getJoint().get_transform_to_parent_body().matrix() ==
              T_model_to_world.matrix());

  // Gets the link whose parent joint is called "joint1".
  auto link2_body = rbs.getRigidBodyTree()->FindChildBodyOfJoint("joint1");
  EXPECT_TRUE(link2_body != nullptr);
  EXPECT_EQ(link2_body->get_name(), "link2");

  // Verifies that the transformation from link2's frame to link1's frame is
  // correct. From the SDF, the transformation is expected to be
  // x = 0, y = 0, z = 0.6.
  Eigen::Isometry3d T_link2_to_link1;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                    xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.6;
    T_link2_to_link1.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_TRUE(link2_body->getJoint().get_transform_to_parent_body().matrix() ==
              T_link2_to_link1.matrix());
}

TEST_P(LoadModelTest, TestWeld) {
  // Loads a one-DOF SDF model with zero offset between the model's
  // root frame and the world frame.
  rbs.AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/models/cylindrical_1dof_robot." + GetParam(),
      kQuaternion);

  // Loads a zero-DOF SDF robot model and weld it to the end of the
  // one DOF robot's link 2.
  Eigen::Isometry3d T_model2_to_link2;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.06;
    T_model2_to_link2.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link2_body = rbs.getRigidBodyTree()->FindBody("link2");
  EXPECT_TRUE(link2_body != nullptr);

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "joint2", link2_body,
      T_model2_to_link2);
  rbs.AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/models/cylindrical_0dof_robot." + GetParam(),
      kFixed, weld_to_frame);

  // Verifies that the newly added link exists and is in the correct location.
  auto link_body = rbs.getRigidBodyTree()->FindBody("link");
  EXPECT_TRUE(link_body != nullptr);
  EXPECT_TRUE(link_body->getJoint().get_transform_to_parent_body().matrix() ==
              T_model2_to_link2.matrix());
}

INSTANTIATE_TEST_CASE_P(LOAD_SDF_AND_URF_TESTS, LoadModelTest,
                        ::testing::Values("urdf", "sdf"));

GTEST_TEST(LoadSDFTest, TestInternalOffset) {
  // Loads a one-DOF SDF model with:
  //   1. A Z = 1 offset between the model's root and the model's world
  //   2. Zero offset between the model's world and Drake's world
  RigidBodySystem rbs;
  rbs.AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/models/cylindrical_1dof_robot_offset_z1.sdf",
      kQuaternion);

  // Verifies that the transform between the robot's root node
  // and the world is equal to Z = 1.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    xyz(2) = 1;
    T_model_to_world.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link1_body = rbs.getRigidBodyTree()->FindBody("link1");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_TRUE(link1_body->getJoint().get_transform_to_parent_body().matrix() ==
              T_model_to_world.matrix());
}

GTEST_TEST(LoadSDFTest, TestDualOffset1) {
  // Loads a one-DOF SDF model with:
  //   1. A Z = 1 offset between the model's root and the model's world
  //   2. An X = 2 offset between the model's world and Drake's world
  Eigen::Isometry3d T_model_world_to_drake_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    xyz(0) = 2;
    T_model_world_to_drake_world.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_world_to_drake_world);

  RigidBodySystem rbs;
  rbs.AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/models/cylindrical_1dof_robot_offset_z1.sdf",
      kQuaternion, weld_to_frame);

  // Verifies that the transform between the robot's root node
  // and the world is equal to X = 2, Z = 1.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz;
    xyz << 2, 0, 1;
    T_model_to_world.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link1_body = rbs.getRigidBodyTree()->FindBody("link1");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_TRUE(link1_body->getJoint().get_transform_to_parent_body().matrix() ==
              T_model_to_world.matrix());
}

GTEST_TEST(LoadSDFTest, TestDualOffset2) {
  // Loads a one-DOF SDF model with:
  //   1. A Z = 1 and Roll = 90 degree offset between the model's root link's
  //      frame and the model's world frame.
  //   2. A Y = -1 and Roll = -90 degree offset between the model's world frame
  //      and Drake's world frame.
  // They should cancel out resulting in zero offset between the model's world
  // frame and Drake's world frame.
  Eigen::Isometry3d T_model_world_to_drake_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 0, -1, 0;
    rpy << -1.570796326794896557998982, 0, 0;
    T_model_world_to_drake_world.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_world_to_drake_world);

  RigidBodySystem rbs;
  rbs.AddModelInstanceFromFile(drake::GetDrakePath() +
                           "/systems/plants/test/models/"
                           "cylindrical_1dof_robot_offset_z1_r90.sdf",
                       kQuaternion, weld_to_frame);

  // Verifies that the transform between the robot's root node
  // and the world is equal to identity.
  auto link1_body = rbs.getRigidBodyTree()->FindBody("link1");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_TRUE(
      link1_body->getJoint().get_transform_to_parent_body().matrix().isApprox(
          Eigen::Isometry3d::Identity().matrix(), 1e-10))
      << "Incorrect transform from the link1's frame to Drake's world frame."
      << "Got:\n" << link1_body->getJoint().get_transform_to_parent_body()
                                           .matrix()
      << "\n"
      << "Expected:\n" << Eigen::Isometry3d::Identity().matrix();
}

GTEST_TEST(LoadSDFTest, TestJointLimitParams) {
  // Test that joint limit parameters are correctly loaded from an SDF file.
  RigidBodySystem rbs;
  rbs.AddModelInstanceFromFile(drake::GetDrakePath() +
                               "/systems/plants/test/models/"
                               "cylindrical_1dof_robot.sdf",
                               kFixed);
  const DrakeJoint& joint =
      rbs.getRigidBodyTree()->FindChildBodyOfJoint("joint1")->getJoint();
  EXPECT_NEAR(joint.getJointLimitMin()(0), -1.5708, 1e-6);
  EXPECT_NEAR(joint.getJointLimitMax()(0), 1.5708, 1e-6);
  EXPECT_NEAR(joint.get_joint_limit_stiffness()(0), 1, 1e-6);
  EXPECT_NEAR(joint.get_joint_limit_dissipation()(0), 1, 1e-6);
}

class ModelToWorldTransformTestParams {
 public:
  ModelToWorldTransformTestParams(std::string urdf_path,
                                  std::string root_link_name, double x,
                                  double y, double z, double roll, double pitch,
                                  double yaw)
      : urdf_path_(urdf_path),
        root_link_name_(root_link_name),
        x_(x),
        y_(y),
        z_(z),
        roll_(roll),
        pitch_(pitch),
        yaw_(yaw) {}

  std::string urdf_path_, root_link_name_;
  double x_, y_, z_, roll_, pitch_, yaw_;
};

/**
 * Defines a class that extends Google Test's ::testing::TestWithParam
 * class, which enables parameterized Google Tests. In this case, the parameter
 * is a ModelToWorldTransformTestParams.
 */
class ModelToWorldTransformTest
    : public ::testing::TestWithParam<ModelToWorldTransformTestParams> {};

TEST_P(ModelToWorldTransformTest, TestModelToWorldTransform) {
  std::unique_ptr<RigidBodySystem> rbs(new RigidBodySystem());

  ModelToWorldTransformTestParams params = GetParam();

  rbs->AddModelInstanceFromFile(params.urdf_path_);

  // Verifies that the transform between the robot's root node and the world is
  // as expected.

  // Defines the expected model-to-world transform.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << params.x_, params.y_, params.z_;
    rpy << params.roll_, params.pitch_, params.yaw_;
    T_model_to_world.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link1_body = rbs->getRigidBodyTree()->FindBody(params.root_link_name_);
  EXPECT_TRUE(link1_body != nullptr);

  // Note: The following two local variables are necessary to avoid a transient
  // "unknown file" error on 32-bit Windows platforms. For more information,
  // see:
  //
  // https://github.com/robotlocomotion/drake/pull/2171#issuecomment-219770037
  auto actual_matrix =
      link1_body->getJoint().get_transform_to_parent_body().matrix();

  auto expected_matrix = T_model_to_world.matrix();

  EXPECT_TRUE(actual_matrix.isApprox(expected_matrix, 1e-10))
      << "ERROR: "
      << "Incorrect transform from link1's frame to Drake's world frame.\n"
      << "Got:\n" << actual_matrix << "\n"
      << "Expected:\n" << expected_matrix;
}

INSTANTIATE_TEST_CASE_P(
    MODEL_TO_WORLD_TRANSFORM_TESTS, ModelToWorldTransformTest,
    ::testing::Values(

        // Evaluates the ability to load a URDF model that's connected to the
        // world via a fixed joint.
        ModelToWorldTransformTestParams(
            drake::GetDrakePath() +
                std::string("/systems/plants/test/models/"
                            "cylindrical_1dof_robot_fixed_to_world.urdf"),
            std::string("link1"), 1, 2, 3, 0.1, 0.5, 1.8),

        // Evaluates the ability to load a URDF model that's connected to the
        // world via a floating joint.
        ModelToWorldTransformTestParams(
            drake::GetDrakePath() +
                std::string("/systems/plants/test/models/"
                            "cylindrical_1dof_robot_floating_in_world.urdf"),
            std::string("link1"), 3, 2, 1, 0.2, 0.9, -1.57)));

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
