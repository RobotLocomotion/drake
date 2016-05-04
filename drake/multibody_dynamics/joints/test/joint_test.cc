#include <iostream>
#include <gtest/gtest.h>
#include "drake/multibody_dynamics/joints/Joints.h"
#include "drake/core/Gradient.h"
#include <memory>

namespace drake {

using namespace Eigen;


template <typename Scalar>
std::vector<std::unique_ptr<Joint<Scalar>>> createJoints() {
  std::vector<std::unique_ptr<Joint<Scalar>>> joints;
  Matrix<Scalar, 3, 1> axis;
  axis << Scalar(1), Scalar(0), Scalar(0);
  Scalar pitch = Scalar(1);

  Transform3D<Scalar> tranform_to_parent = Transform3D<Scalar>::Identity();
  joints.push_back(std::move(std::unique_ptr<Joint<Scalar>>(new QuaternionFloatingJoint<Scalar>("quaternion", tranform_to_parent))));
  joints.push_back(std::move(std::unique_ptr<Joint<Scalar>>(new RollPitchYawFloatingJoint<Scalar>("rollpitchyaw", tranform_to_parent))));
  joints.push_back(std::move(std::unique_ptr<Joint<Scalar>>(new FixedJoint<Scalar>("fixed", tranform_to_parent))));
  joints.push_back(std::move(std::unique_ptr<Joint<Scalar>>(new RevoluteJoint<Scalar>("revolute", tranform_to_parent, axis))));
  joints.push_back(std::move(std::unique_ptr<Joint<Scalar>>(new PrismaticJoint<Scalar>("prismatic", tranform_to_parent, axis))));
  joints.push_back(std::move(std::unique_ptr<Joint<Scalar>>(new HelicalJoint<Scalar>("helical", tranform_to_parent, axis, pitch))));
  return joints;
}

TEST(JointTest, ZeroConfiguration) {
  auto joints = createJoints<double>();
  for (const auto& joint_ptr : joints) {
    auto q0 = joint_ptr->ZeroConfiguration();
    auto zero_transform = joint_ptr->JointTransform(q0);
    EXPECT_TRUE(zero_transform.isApprox(Transform3D<double>::Identity()));
  }
}

TEST(JointTest, ConfigurationDotToVelocityAndBack) {
  auto joints = createJoints<double>();
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

TEST(JointTest, JointTransformDerivativeTwoWays) {
  using namespace Drake;

  auto joints_double = createJoints<double>();
  auto joints_autodiff = createJoints<AutoDiffScalar<Matrix<double, 1, 1>>>();
  std::default_random_engine generator;

  for (int i = 0; i < joints_double.size(); i++) {
    const auto &joint_ptr_double = joints_double[i];
    const auto &joint_ptr_autodiff = joints_autodiff[i];

    // using (2.55) in Murray et al. - A Mathematical Introduction to Robotic Manipulation
    auto q = joint_ptr_double->RandomConfiguration(generator);
    auto v = VectorXd::Random(joint_ptr_double->GetNumVelocities()).eval();
    auto twist = (joint_ptr_double->MotionSubspace(q) * v).eval();
    auto transform = joint_ptr_double->JointTransform(q);
    Matrix4d twist_hat = Matrix4d::Zero();
    auto angular = twist.topRows<3>();
    auto linear = twist.bottomRows<3>();
    twist_hat.topLeftCorner<3, 3>() = vectorToSkewSymmetric(angular);
    twist_hat.topRightCorner<3, 1>() = linear;
    Matrix4d transform_derivative_1 = (transform.matrix() * twist_hat).eval();

    // using autodiff
    auto qd = (joint_ptr_double->VelocityToConfigurationDerivative(q) * v).eval();
    auto q_autodiff = initializeAutoDiffGivenGradientMatrix(q, qd);
    auto transform_autodiff = joint_ptr_autodiff->JointTransform(q_autodiff);
    auto transform_derivative_2_vectorized = autoDiffToGradientMatrix(transform_autodiff.matrix());
    Map<Matrix4d> transform_derivative_2(transform_derivative_2_vectorized.data(), transform_derivative_1.rows(), transform_derivative_1.cols());

    EXPECT_TRUE(transform_derivative_1.isApprox(transform_derivative_2));
  }
}

} // drake

