#include "drake/multibody_dynamics/joints/Joints.h"
#include <iostream>
#include <gtest/gtest.h>
#include <memory>

namespace drake {

using namespace Eigen;

class JointTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    joints.clear();

    Vector3d axis = Vector3d::Random();
    double pitch = 1.0;

    Transform3D<double> tranform_to_parent = Transform3D<double>::Identity();
    joints.push_back(std::move(std::unique_ptr<Joint<double>>(new QuaternionFloatingJoint<double>("quaternion", tranform_to_parent))));
    joints.push_back(std::move(std::unique_ptr<Joint<double>>(new RollPitchYawFloatingJoint<double>("rollpitchyaw", tranform_to_parent))));
    joints.push_back(std::move(std::unique_ptr<Joint<double>>(new FixedJoint<double>("fixed", tranform_to_parent))));
    joints.push_back(std::move(std::unique_ptr<Joint<double>>(new RevoluteJoint<double>("revolute", tranform_to_parent, axis))));
    joints.push_back(std::move(std::unique_ptr<Joint<double>>(new PrismaticJoint<double>("prismatic", tranform_to_parent, axis))));
    joints.push_back(std::move(std::unique_ptr<Joint<double>>(new HelicalJoint<double>("helical", tranform_to_parent, axis, pitch))));
  }

 public:
  std::vector<std::unique_ptr<Joint<double>>> joints;
};


TEST_F(JointTest, ZeroConfiguration) {
  for (const auto& joint_ptr : joints) {
    // zero configuration --> JointTransform identity
    auto q0 = joint_ptr->ZeroConfiguration();
    auto zero_transform = joint_ptr->JointTransform(q0);
    EXPECT_TRUE(zero_transform.isApprox(Transform3D<double>::Identity()));
  }
}

// compute Tdot using autodiff, compare to twist from motion subspace times v


TEST_F(JointTest, ConfigurationDotToVelocityAndBack) {
  std::default_random_engine generator;
  for (const auto& joint_ptr : joints) {
    auto q = joint_ptr->RandomConfiguration(generator);
    auto velocity_to_configuration_dot = joint_ptr->VelocityToConfigurationDerivative(q);
    auto configuration_dot_to_velocity = joint_ptr->ConfigurationDerivativeToVelocity(q);
    int num_velocities = joint_ptr->GetNumVelocities();
    auto bla = (configuration_dot_to_velocity * velocity_to_configuration_dot).eval();
    auto zero = MatrixXd::Identity(num_velocities, num_velocities).eval();
    EXPECT_TRUE(bla.isApprox(zero));
  }
}

} // drake

