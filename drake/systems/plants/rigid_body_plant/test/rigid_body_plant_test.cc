#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/joints/PrismaticJoint.h"
#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {
namespace {

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

// Tests the ability to load a URDF model instance into the world of a rigid
// body system.
GTEST_TEST(RigidBodySystemTest, TestLoadURDFWorld) {
  // Instantiates an Multibody Dynamics (MBD) model of the world.
  auto tree_ptr = make_unique<RigidBodyTree>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
      "/systems/plants/rigid_body_plant/test/world.urdf",
      drake::systems::plants::joints::kFixed, nullptr /* weld to frame */,
      tree_ptr.get());

  // Instantiates a RigidBodyPlant from an MBD model of the world.
  RigidBodyPlant<double> rigid_body_sys(move(tree_ptr));

  // Verifies that the number of states, inputs, and outputs are all zero.
  EXPECT_EQ(rigid_body_sys.get_num_states(), 0);
  EXPECT_EQ(rigid_body_sys.get_input_size(), 0);
  EXPECT_EQ(rigid_body_sys.get_output_size(), 0);

  // Obtains a const pointer to the underlying multibody world in the system.
  const RigidBodyTree& tree = rigid_body_sys.get_rigid_body_tree();

  // Checks that the bodies in the multibody world can be obtained by name and
  // that they have the correct model name.
  for (auto& body_name :
       {"floor", "ramp_1", "ramp_2", "box_1", "box_2", "box_3", "box_4"}) {
    RigidBody* body = tree.FindBody(body_name);
    EXPECT_NE(body, nullptr);
    EXPECT_EQ(body->get_model_name(), "dual_ramps");
  }
}

// Unit tests the generalized velocities to generalized coordinates time
// derivatives for a free body with a quaternion base.
GTEST_TEST(RigidBodySystemTest, MapVelocityToConfigurationDerivatives) {
  const int kNumPositions = 7;  // One quaternion + 3d position.
  const int kNumVelocities = 6;  // Angular velocity + linear velocity.
  const int kNumStates = kNumPositions + kNumVelocities;
  // Instantiates a Multibody Dynamics (MBD) model of the world.
  auto tree = make_unique<RigidBodyTree>();

  // Add a single free body with a quaternion base.
  RigidBody* body;
  tree->add_rigid_body(unique_ptr<RigidBody>(body = new RigidBody()));
  body->set_name("free_body");
  // Sets body to have a non-zero spatial inertia. Otherwise the body gets
  // welded by a fixed joint to the world by RigidBodyTree::compile().
  body->set_mass(1.0);
  body->set_spatial_inertia(Matrix6<double>::Identity());

  body->add_joint(
      &tree->world(),
      make_unique<QuaternionFloatingJoint>("base", Isometry3d::Identity()));

  tree->compile();

  // Verifies the correct number of DOF's.
  EXPECT_EQ(tree->get_num_bodies(), 2);
  EXPECT_EQ(tree->get_num_positions(), kNumPositions);
  // There are two bodies: the "world" and "free_body".
  EXPECT_EQ(tree->get_num_velocities(), kNumVelocities);

  // Instantiates a RigidBodyPlant from an MBD model of the world.
  RigidBodyPlant<double> plant(move(tree));
  auto context = plant.CreateDefaultContext();

  // Sets free_body to have zero translation and zero rotation.
  plant.SetZeroConfiguration(context.get());

  // Verifies the number of states, inputs, and outputs.
  EXPECT_EQ(plant.get_num_states(), kNumStates);
  EXPECT_EQ(plant.get_num_positions(), kNumPositions);
  EXPECT_EQ(plant.get_num_velocities(), kNumVelocities);
  EXPECT_EQ(plant.get_input_size(), 0);  // There are no actuators.
  EXPECT_EQ(plant.get_output_size(), kNumStates);

  Vector3d v0(1, 2, 3);    // Linear velocity in body's frame.
  Vector3d w0(-1, 2, -3);  // Angular velocity in body's frame.
  BasicVector<double> generalized_velocities(plant.get_num_velocities());
  generalized_velocities.get_mutable_value() << w0, v0;
  BasicVector<double> positions_derivatives(plant.get_num_positions());

  ASSERT_EQ(positions_derivatives.size(), kNumPositions);
  ASSERT_EQ(generalized_velocities.size(), kNumVelocities);

  plant.MapVelocityToConfigurationDerivatives(
      *context, generalized_velocities, &positions_derivatives);

  // For zero rotation the velocity vector in the body's frame and in the
  // world's frame is the same.
  EXPECT_EQ(v0[0], positions_derivatives.GetAtIndex(0));
  EXPECT_EQ(v0[1], positions_derivatives.GetAtIndex(1));
  EXPECT_EQ(v0[2], positions_derivatives.GetAtIndex(2));

  // Computes the expected value of the time derivative of the quaternion
  // component.
  Quaterniond quaternion = Quaterniond::Identity();
  Quaterniond dqdt =
      Quaterniond(0, w0[0] / 2, w0[1] / 2, w0[2] / 2) * quaternion;

  EXPECT_EQ(dqdt.w(), positions_derivatives.GetAtIndex(3));
  EXPECT_EQ(dqdt.x(), positions_derivatives.GetAtIndex(4));
  EXPECT_EQ(dqdt.y(), positions_derivatives.GetAtIndex(5));
  EXPECT_EQ(dqdt.z(), positions_derivatives.GetAtIndex(6));
}

class KukaArmTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Instantiates an MBD model of the world.
    auto tree = make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
        drake::systems::plants::joints::kFixed, nullptr /* weld to frame */,
        tree.get());

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    kuka_system_ = make_unique<RigidBodyPlant<double>>(move(tree));

    context_ = kuka_system_->CreateDefaultContext();
    output_ = kuka_system_->AllocateOutput(*context_);
    derivatives_ = kuka_system_->AllocateTimeDerivatives();
  }

  const int kNumPositions_{7};
  const int kNumVelocities_{7};
  const int kNumActuators_{kNumPositions_};
  const int kNumStates_{kNumPositions_ + kNumVelocities_};

  unique_ptr<RigidBodyPlant<double>> kuka_system_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Tests that the KukaArm system allocates a continuous state of the proper
// size in the context.
TEST_F(KukaArmTest, StateHasTheRightSizes) {
  const VectorBase<double>& xc =
      context_->get_continuous_state()->get_generalized_position();
  const VectorBase<double>& vc =
      context_->get_continuous_state()->get_generalized_velocity();
  const VectorBase<double>& zc =
      context_->get_continuous_state()->get_misc_continuous_state();

  EXPECT_EQ(kNumPositions_, xc.size());
  EXPECT_EQ(kNumVelocities_, vc.size());
  EXPECT_EQ(0, zc.size());
}

// Tests the method that obtains the zero configuration of the system for a
// Kuka arm model. In this case the zero configuration corresponds to all joint
// angles and velocities being zero.
// The system configuration is written to a context.
TEST_F(KukaArmTest, SetZeroConfiguration) {
  // Connect to a "fake" free standing input.
  // TODO(amcastro-tri): Connect to a ConstantVectorSource once Diagrams have
  // derivatives per #3218.
  context_->SetInputPort(0, MakeInput(
      make_unique<BasicVector<double>>(
          kuka_system_->get_num_actuators())));

  kuka_system_->SetZeroConfiguration(context_.get());

  // Asserts that for this case the zero configuration corresponds to a state
  // vector with all entries equal to zero.
  VectorXd xc = context_->get_continuous_state()->CopyToVector();
  ASSERT_EQ(kNumStates_, xc.size());
  ASSERT_EQ(xc, VectorXd::Zero(xc.size()));
}

// Tests RigidBodyPlant<T>::EvalOutput() for a Kuka arm model.
// For a RigidBodyPlant<T> the first output of the system should equal the
// state vector. The second output from this system should correspond to a
// RigidBodyPlant<T>::VectorOfPoses containing the poses of all bodies in the
// system.
TEST_F(KukaArmTest, EvalOutput) {
  auto& tree = kuka_system_->get_rigid_body_tree();

  // Checks that the number of input and output ports in the system and context
  // are consistent.
  ASSERT_EQ(1, kuka_system_->get_num_input_ports());
  ASSERT_EQ(1, context_->get_num_input_ports());

  // Checks the size of the input ports to match the number of generalized
  // forces that can be applied.
  ASSERT_EQ(kNumPositions_, kuka_system_->get_num_positions());
  ASSERT_EQ(kNumVelocities_, kuka_system_->get_num_velocities());
  ASSERT_EQ(kNumStates_, kuka_system_->get_num_states());
  ASSERT_EQ(kNumActuators_, kuka_system_->get_num_actuators());
  ASSERT_EQ(kNumActuators_, kuka_system_->get_input_port(0).get_size());

  // Connect to a "fake" free standing input.
  // TODO(amcastro-tri): Connect to a ConstantVectorSource once Diagrams have
  // derivatives per #3218.
  context_->SetInputPort(0, MakeInput(
      make_unique<BasicVector<double>>(
          kuka_system_->get_num_actuators())));

  // Zeroes the state.
  kuka_system_->SetZeroConfiguration(context_.get());

  // Sets the state to a non-zero value.
  VectorXd desired_angles(kNumPositions_);
  desired_angles << 0.5, 0.1, -0.1, 0.2, 0.3, -0.2, 0.15;
  for (int i = 0; i < kNumPositions_; ++i) {
    kuka_system_->set_position(context_.get(), i, desired_angles[i]);
  }
  VectorXd desired_state(kNumStates_);
  desired_state << desired_angles, VectorXd::Zero(kNumVelocities_);
  VectorXd xc = context_->get_continuous_state()->CopyToVector();
  ASSERT_EQ(xc, desired_state);

  ASSERT_EQ(2, output_->get_num_ports());
  const BasicVector<double>* output_state = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_state);

  kuka_system_->EvalOutput(*context_, output_.get());

  // Asserts the output equals the state.
  EXPECT_EQ(desired_state, output_state->get_value());

  // Evaluates the correctness of the kinematics results port.
  auto& kinematics_results =
      output_->get_data(1)->GetValue<KinematicsResults<double>>();
  ASSERT_EQ(kinematics_results.get_num_positions(), kNumPositions_);
  ASSERT_EQ(kinematics_results.get_num_velocities(), kNumVelocities_);

  VectorXd q = xc.topRows(kNumPositions_);
  VectorXd v = xc.bottomRows(kNumVelocities_);
  auto cache = tree.doKinematics(q, v);

  for (int ibody = 0; ibody < kuka_system_->get_num_bodies(); ++ibody) {
    Isometry3d pose = tree.relativeTransform(cache, 0, ibody);
    Vector4d quat_vector = drake::math::rotmat2quat(pose.linear());
    // Note that Eigen quaternion elements are not laid out in memory in the
    // same way Drake currently aligns them. See issue #3470.
    // When solved we will not need to instantiate a temporary Quaternion below
    // just to perform a comparison.
    Quaterniond quat(
        quat_vector[0], quat_vector[1], quat_vector[2], quat_vector[3]);
    Vector3d position = pose.translation();
    EXPECT_TRUE(quat.isApprox(kinematics_results.get_body_orientation(ibody)));
    EXPECT_TRUE(position.isApprox(kinematics_results.get_body_position(ibody)));
  }
}

GTEST_TEST(rigid_body_plant_test, TestJointLimitForcesFormula) {
  typedef RigidBodyPlant<double> RBP;
  const double lower_limit = 10.;
  const double upper_limit = 20.;
  const double stiffness = 10.;
  const double dissipation = 0.5;
  const double delta = 1e-6;  // Perturbation used for continuity test.
  const double epsilon = 1e-4;

  // The joint limit force formula is quite complex.  This test checks each of
  // its modes, to ensure that we didn't flip a sign somewhere.
  PrismaticJoint joint("test_joint", Isometry3d::Identity(), Vector3d(1, 0, 0));
  joint.setJointLimits(lower_limit, upper_limit);
  joint.SetJointLimitDynamics(stiffness, dissipation);

  // Force should be continuously near zero at the limit.
  EXPECT_EQ(RBP::JointLimitForce(joint, lower_limit + delta, 0),
            0);
  EXPECT_EQ(RBP::JointLimitForce(joint, lower_limit, 0),
            0);
  EXPECT_NEAR(RBP::JointLimitForce(joint, lower_limit - delta, 0),
              0, epsilon);
  EXPECT_GT(RBP::JointLimitForce(joint, lower_limit - delta, 0),
            0);
  EXPECT_NEAR(RBP::JointLimitForce(joint, upper_limit + delta, 0),
              0, epsilon);
  EXPECT_LT(RBP::JointLimitForce(joint, upper_limit + delta, 0),
            0);
  EXPECT_EQ(RBP::JointLimitForce(joint, upper_limit, 0),
            0);
  EXPECT_EQ(RBP::JointLimitForce(joint, upper_limit - delta, 0),
            0);

  // At zero velocity, expect a spring force law.
  EXPECT_NEAR(RBP::JointLimitForce(joint, lower_limit - 1, 0),
              stiffness, epsilon);
  EXPECT_NEAR(RBP::JointLimitForce(joint, upper_limit + 1, 0),
              -stiffness, epsilon);

  // At outward velocity, a much stiffer counterforce (ie, not "squishy").
  EXPECT_NEAR(RBP::JointLimitForce(joint, lower_limit - 1, -1),
              stiffness * (1 + dissipation), epsilon);
  EXPECT_NEAR(RBP::JointLimitForce(joint, upper_limit + 1, 1),
              -stiffness * (1 + dissipation), epsilon);

  // At inward velocity, a looser counterforce (ie, not "bouncy").
  EXPECT_NEAR(RBP::JointLimitForce(joint, lower_limit - 1, 1),
              stiffness * (1 - dissipation), epsilon);
  EXPECT_NEAR(RBP::JointLimitForce(joint, upper_limit + 1, -1),
              -stiffness * (1 - dissipation), epsilon);

  // At rapid inward velocity, no negative counterforce (ie, not "sticky").
  EXPECT_EQ(RBP::JointLimitForce(joint, lower_limit - 1, 10),
            0);
  EXPECT_EQ(RBP::JointLimitForce(joint, upper_limit + 1, -10),
            0);
}

/// Given a starting @p position and @p applied_force, @return the resulting
/// acceleration of the joint described in `limited_prismatic.sdf`.
double GetPrismaticJointLimitAccel(double position, double applied_force) {
  // Build two links connected by a limited prismatic joint.
  auto rigid_body_tree = std::make_unique<RigidBodyTree>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
      "/systems/plants/rigid_body_plant/test/limited_prismatic.sdf",
      drake::systems::plants::joints::kFixed, nullptr /* weld to frame */,
      rigid_body_tree.get());
  RigidBodyPlant<double> rigid_body_sys(move(rigid_body_tree));

  auto context = rigid_body_sys.CreateDefaultContext();
  rigid_body_sys.SetZeroConfiguration(context.get());
  context->get_mutable_continuous_state()
      ->get_mutable_generalized_position()
      ->SetAtIndex(0, position);

  // Apply a constant force on the input.
  Vector1d input;
  input << applied_force;
  auto input_vector = std::make_unique<BasicVector<double>>(1);
  input_vector->set_value(input);
  context->SetInputPort(0, MakeInput(move(input_vector)));

  // Obtain the time derivatives; test that speed is zero, return acceleration.
  auto derivatives = rigid_body_sys.AllocateTimeDerivatives();
  rigid_body_sys.EvalTimeDerivatives(*context, derivatives.get());
  auto xdot = derivatives->CopyToVector();
  EXPECT_EQ(xdot(0), 0.);  // Not moving.
  return xdot(1);
}

GTEST_TEST(rigid_body_plant_test, TestJointLimitForces) {
  // Test that joint limit forces are applied correctly in the rigid body
  // tree.  This tests for a sign error at rigid_body_plant.cc@417b03e:240.

  // Past the lower limit, acceleration should be upward.
  EXPECT_GT(GetPrismaticJointLimitAccel(-1.05, 0.), 0.);

  // Between the limits, acceleration should equal force (the moving mass in
  // the SDF is 1kg).
  EXPECT_EQ(GetPrismaticJointLimitAccel(0., -1.), -1.);
  EXPECT_EQ(GetPrismaticJointLimitAccel(0., 0), 0.);
  EXPECT_EQ(GetPrismaticJointLimitAccel(0., 1.), 1.);

  // Past the upper limit, acceleration should be downward.
  EXPECT_LT(GetPrismaticJointLimitAccel(1.05, 0.), 0.);
}

}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
