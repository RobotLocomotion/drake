#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using multibody::joints::kFixed;
using multibody::joints::kQuaternion;
using parsers::ModelInstanceIdTable;
using parsers::sdf::AddModelInstancesFromSdfFile;

namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {
namespace {

// Tests the ability to load an instance of a URDF model into a RigidBodyPlant.
GTEST_TEST(RigidBodyPlantTest, TestLoadUrdf) {
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/multibody/rigid_body_plant/test/world.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree_ptr.get());

  RigidBodyPlant<double> plant(move(tree_ptr));

  // Verifies that the number of states, inputs, and outputs are all zero.
  EXPECT_EQ(plant.get_num_states(), 0);
  EXPECT_EQ(plant.get_input_size(), 0);
  EXPECT_EQ(plant.get_output_size(), 0);

  // Obtains a const reference to the underlying RigidBodyTree within the
  // RigidBodyPlant.
  const RigidBodyTree<double>& tree = plant.get_rigid_body_tree();

  // Checks that the bodies can be obtained by name and that they have the
  // correct model name.
  for (auto& body_name :
       {"floor", "ramp_1", "ramp_2", "box_1", "box_2", "box_3", "box_4"}) {
    RigidBody<double>* body = tree.FindBody(body_name);
    EXPECT_NE(body, nullptr);
    EXPECT_EQ(body->get_model_name(), "dual_ramps");
  }
}

// Tests the generalized velocities to generalized coordinates time
// derivatives for a free body with a quaternion base.
GTEST_TEST(RigidBodyPlantTest, MapVelocityToConfigurationDerivativesAndBack) {
  const double kTol = 5e-12;     // Loosest tolerance that all tests succeed.
  const int kNumPositions = 7;   // One quaternion + 3d position.
  const int kNumVelocities = 6;  // Angular velocity + linear velocity.
  const int kNumStates = kNumPositions + kNumVelocities;

  auto tree = make_unique<RigidBodyTree<double>>();

  // Adds a single free body with a quaternion base.
  RigidBody<double>* body;
  tree->add_rigid_body(
      unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
  body->set_name("free_body");
  // Sets body to have a non-zero spatial inertia. Otherwise the body gets
  // welded by a fixed joint to the world by RigidBodyTree::compile().
  body->set_mass(1.0);
  body->set_spatial_inertia(Matrix6<double>::Identity());

  body->add_joint(&tree->world(), make_unique<QuaternionFloatingJoint>(
                                      "base", Isometry3d::Identity()));

  tree->compile();

  // Verifies the correct number of DOF's.
  EXPECT_EQ(tree->get_num_bodies(), 2);
  EXPECT_EQ(tree->get_num_positions(), kNumPositions);
  // There are two bodies: the "world" and "free_body".
  EXPECT_EQ(tree->get_num_velocities(), kNumVelocities);

  // Instantiates a RigidBodyPlant from the previously instantiated
  // RigidBodyTree.
  RigidBodyPlant<double> plant(move(tree));
  auto context = plant.CreateDefaultContext();

  // Verifies the number of states, inputs, and outputs.
  EXPECT_EQ(plant.get_num_states(), kNumStates);
  EXPECT_EQ(plant.get_num_positions(), kNumPositions);
  EXPECT_EQ(plant.get_num_velocities(), kNumVelocities);
  EXPECT_EQ(plant.get_input_size(), 0);  // There are no actuators.
  EXPECT_EQ(plant.get_output_size(), kNumStates);

  const Vector3d v0(1, 2, 3);    // Linear velocity in body's frame.
  const Vector3d w0(-4, 5, -6);  // Angular velocity in body's frame.
  BasicVector<double> generalized_velocities(plant.get_num_velocities());
  generalized_velocities.get_mutable_value() << w0, v0;
  BasicVector<double> positions_derivatives(plant.get_num_positions());

  ASSERT_EQ(positions_derivatives.size(), kNumPositions);
  ASSERT_EQ(generalized_velocities.size(), kNumVelocities);

  // Transforms the generalized velocities to time derivative of generalized
  // coordinates.
  plant.MapVelocityToQDot(*context, generalized_velocities,
                          &positions_derivatives);

  // For zero rotation the velocity vector in the body's frame and in the
  // world's frame is the same.
  EXPECT_EQ(v0[0], positions_derivatives.GetAtIndex(0));
  EXPECT_EQ(v0[1], positions_derivatives.GetAtIndex(1));
  EXPECT_EQ(v0[2], positions_derivatives.GetAtIndex(2));

  // Loop over roll-pitch-yaw values: this will run approximately 1,000 tests.
  const double kAngleInc = 10.0 * M_PI / 180.0;  // 10 degree increments
  for (double roll = 0; roll <= M_PI_2; roll += kAngleInc) {
    for (double pitch = 0; pitch <= M_PI_2; pitch += kAngleInc) {
      for (double yaw = 0; yaw <= M_PI_2; yaw += kAngleInc) {
        // Get the mutable state.
        VectorBase<double>* xc = context->get_mutable_state()
                                     ->get_mutable_continuous_state()
                                     ->get_mutable_generalized_position();

        // Update the orientation.
        const Quaterniond q = Eigen::AngleAxisd(roll, Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
                              Eigen::AngleAxisd(yaw, Vector3d::UnitX());

        // Verify normalization.
        DRAKE_ASSERT(std::abs(q.norm() - 1.0) < 1e-15);
        xc->SetAtIndex(3, q.w());
        xc->SetAtIndex(4, q.x());
        xc->SetAtIndex(5, q.y());
        xc->SetAtIndex(6, q.z());

        // Transform the generalized velocities to time derivative of
        // generalized coordinates.
        plant.MapVelocityToQDot(*context, generalized_velocities,
                                &positions_derivatives);

        // TODO(edrumwri): Uncomment the following test when the quaternion
        // derivative code is correct. See #4121.

        // Test q * qdot near zero.
        Quaterniond qdot(positions_derivatives.GetAtIndex(3),
                         positions_derivatives.GetAtIndex(4),
                         positions_derivatives.GetAtIndex(5),
                         positions_derivatives.GetAtIndex(6));
        DRAKE_ASSERT(std::abs(q.dot(qdot)) < 1e-15);

        // Map time derivative of generalized configuration back to generalized
        // velocity.
        plant.MapQDotToVelocity(*context, positions_derivatives,
                                &generalized_velocities);

        // Ordering is angular velocities first, linear velocities second.
        EXPECT_NEAR(w0[0], generalized_velocities.GetAtIndex(0), kTol);
        EXPECT_NEAR(w0[1], generalized_velocities.GetAtIndex(1), kTol);
        EXPECT_NEAR(w0[2], generalized_velocities.GetAtIndex(2), kTol);
        EXPECT_NEAR(v0[0], generalized_velocities.GetAtIndex(3), kTol);
        EXPECT_NEAR(v0[1], generalized_velocities.GetAtIndex(4), kTol);
        EXPECT_NEAR(v0[2], generalized_velocities.GetAtIndex(5), kTol);
      }
    }
  }
}

class KukaArmTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto tree = make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf",
        drake::multibody::joints::kFixed, nullptr /* weld to frame */,
        tree.get());

    kuka_plant_ = make_unique<RigidBodyPlant<double>>(move(tree));

    context_ = kuka_plant_->CreateDefaultContext();
    output_ = kuka_plant_->AllocateOutput(*context_);
    derivatives_ = kuka_plant_->AllocateTimeDerivatives();
  }

  const int kNumPositions_{7};
  const int kNumVelocities_{7};
  const int kNumActuators_{kNumPositions_};
  const int kNumStates_{kNumPositions_ + kNumVelocities_};

  unique_ptr<RigidBodyPlant<double>> kuka_plant_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Tests that the KUKA iiwa arm's RigidBodyPlant allocates a continuous state
// of the proper size in the context.
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
TEST_F(KukaArmTest, SetDefaultState) {
  // Connect to a "fake" free standing input.
  // TODO(amcastro-tri): Connect to a ConstantVectorSource once Diagrams have
  // derivatives per #3218.
  context_->FixInputPort(
      kuka_plant_->actuator_command_input_port().get_index(),
      make_unique<BasicVector<double>>(kuka_plant_->get_num_actuators()));

  // Asserts that for this case the zero configuration corresponds to a state
  // vector with all entries equal to zero.
  VectorXd xc = context_->get_continuous_state()->CopyToVector();
  ASSERT_EQ(kNumStates_, xc.size());
  ASSERT_EQ(xc, VectorXd::Zero(xc.size()));
}

// Tests RigidBodyPlant<T>::CalcOutput() for a KUKA iiwa arm model.
TEST_F(KukaArmTest, EvalOutput) {
  auto& tree = kuka_plant_->get_rigid_body_tree();

  // Checks that the number of input and output ports in the system and context
  // are consistent.
  ASSERT_EQ(1, kuka_plant_->get_num_input_ports());
  ASSERT_EQ(1, context_->get_num_input_ports());
  ASSERT_EQ(1, kuka_plant_->get_num_model_instances());

  const int kModelInstanceId =
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;

  // Checks the size of the input ports to match the number of generalized
  // forces that can be applied.
  ASSERT_EQ(kNumPositions_, kuka_plant_->get_num_positions());
  ASSERT_EQ(kNumPositions_, kuka_plant_->get_num_positions(0));
  ASSERT_EQ(kNumVelocities_, kuka_plant_->get_num_velocities());
  ASSERT_EQ(kNumVelocities_, kuka_plant_->get_num_velocities(0));
  ASSERT_EQ(kNumStates_, kuka_plant_->get_num_states());
  ASSERT_EQ(kNumStates_, kuka_plant_->get_num_states(0));
  ASSERT_EQ(kNumActuators_, kuka_plant_->get_num_actuators());
  ASSERT_EQ(kNumActuators_, kuka_plant_->get_num_actuators(0));
  ASSERT_EQ(
      kNumActuators_,
      kuka_plant_->model_instance_actuator_command_input_port(kModelInstanceId)
          .size());

  // Connect to a "fake" free standing input.
  // TODO(amcastro-tri): Connect to a ConstantVectorSource once Diagrams have
  // derivatives per #3218.
  context_->FixInputPort(
      kuka_plant_->model_instance_actuator_command_input_port(kModelInstanceId)
          .get_index(),
      make_unique<BasicVector<double>>(kuka_plant_->get_num_actuators()));

  // Sets the state to a non-zero value.
  VectorXd desired_angles(kNumPositions_);
  desired_angles << 0.5, 0.1, -0.1, 0.2, 0.3, -0.2, 0.15;
  for (int i = 0; i < kNumPositions_; ++i) {
    kuka_plant_->set_position(context_.get(), i, desired_angles[i]);
  }
  VectorXd desired_state(kNumStates_);
  desired_state << desired_angles, VectorXd::Zero(kNumVelocities_);
  VectorXd xc = context_->get_continuous_state()->CopyToVector();
  ASSERT_EQ(xc, desired_state);

  // Four output ports:
  //
  //    (1) plant state
  //    (2) model instance state for tree containing a single model instance
  //    (3) kinematic results
  //    (4) contact results
  //
  // (In this context, there is only one model instance and thus only one model
  // instance state port.)
  ASSERT_EQ(4, output_->get_num_ports());

  kuka_plant_->CalcOutput(*context_, output_.get());

  // Check that the per-instance port (we should only have one) equals
  // the expected state.
  const int output_index =
      kuka_plant_->model_instance_state_output_port(kModelInstanceId)
          .get_index();
  const BasicVector<double>* instance_output =
      output_->get_vector_data(output_index);
  ASSERT_NE(nullptr, instance_output);
  EXPECT_EQ(desired_state, instance_output->get_value().eval());

  // Evaluates the correctness of the kinematics results port.
  const int index = kuka_plant_->kinematics_results_output_port().get_index();
  auto& kinematics_results =
      output_->get_data(index)->GetValue<KinematicsResults<double>>();
  ASSERT_EQ(kinematics_results.get_num_positions(), kNumPositions_);
  ASSERT_EQ(kinematics_results.get_num_velocities(), kNumVelocities_);

  VectorXd q = xc.topRows(kNumPositions_);
  VectorXd v = xc.bottomRows(kNumVelocities_);
  auto cache = tree.doKinematics(q, v);

  for (int ibody = 0; ibody < kuka_plant_->get_num_bodies(); ++ibody) {
    Isometry3d pose = tree.relativeTransform(cache, 0, ibody);
    Vector4d quat_vector = drake::math::rotmat2quat(pose.linear());
    // Note that Eigen quaternion elements are not laid out in memory in the
    // same way Drake currently aligns them. See issue #3470.
    // When solved we will not need to instantiate a temporary Quaternion below
    // just to perform a comparison.
    Quaterniond quat(quat_vector[0], quat_vector[1], quat_vector[2],
                     quat_vector[3]);
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
  EXPECT_EQ(RBP::JointLimitForce(joint, lower_limit + delta, 0), 0);
  EXPECT_EQ(RBP::JointLimitForce(joint, lower_limit, 0), 0);
  EXPECT_NEAR(RBP::JointLimitForce(joint, lower_limit - delta, 0), 0, epsilon);
  EXPECT_GT(RBP::JointLimitForce(joint, lower_limit - delta, 0), 0);
  EXPECT_NEAR(RBP::JointLimitForce(joint, upper_limit + delta, 0), 0, epsilon);
  EXPECT_LT(RBP::JointLimitForce(joint, upper_limit + delta, 0), 0);
  EXPECT_EQ(RBP::JointLimitForce(joint, upper_limit, 0), 0);
  EXPECT_EQ(RBP::JointLimitForce(joint, upper_limit - delta, 0), 0);

  // At zero velocity, expect a spring force law.
  EXPECT_NEAR(RBP::JointLimitForce(joint, lower_limit - 1, 0), stiffness,
              epsilon);
  EXPECT_NEAR(RBP::JointLimitForce(joint, upper_limit + 1, 0), -stiffness,
              epsilon);

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
  EXPECT_EQ(RBP::JointLimitForce(joint, lower_limit - 1, 10), 0);
  EXPECT_EQ(RBP::JointLimitForce(joint, upper_limit + 1, -10), 0);
}

/// Given a starting @p position and @p applied_force, @return the resulting
/// acceleration of the joint described in `limited_prismatic.sdf`.
double GetPrismaticJointLimitAccel(double position, double applied_force) {
  // Build two links connected by a limited prismatic joint.
  auto tree = std::make_unique<RigidBodyTree<double>>();
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
          "/multibody/rigid_body_plant/test/limited_prismatic.sdf",
      kFixed, nullptr /* weld to frame */, tree.get());
  RigidBodyPlant<double> plant(move(tree));

  auto context = plant.CreateDefaultContext();
  context->get_mutable_continuous_state()
      ->get_mutable_generalized_position()
      ->SetAtIndex(0, position);

  // Apply a constant force on the input.
  Vector1d input;
  input << applied_force;
  auto input_vector = std::make_unique<BasicVector<double>>(1);
  input_vector->set_value(input);
  context->FixInputPort(plant.actuator_command_input_port().get_index(),
                        move(input_vector));

  // Obtain the time derivatives; test that speed is zero, return acceleration.
  auto derivatives = plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(*context, derivatives.get());
  auto xdot = derivatives->CopyToVector();
  EXPECT_EQ(xdot(0), 0.);  // Not moving.
  return xdot(1);
}

// Tests that joint limit forces are applied correctly in the rigid body
// tree.  This tests for a sign error at rigid_body_plant.cc@417b03e:240.
GTEST_TEST(rigid_body_plant_test, TestJointLimitForces) {
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

// Tests that the given 3x3 matrix is orthonormal.
void ExpectOrthonormal(const Matrix3<double>& R) {
  const double kEpsilon = Eigen::NumTraits<double>::dummy_precision();
  // Confirms normal length.
  EXPECT_NEAR(1.0, R.col(0).norm(), kEpsilon);
  EXPECT_NEAR(1.0, R.col(1).norm(), kEpsilon);
  EXPECT_NEAR(1.0, R.col(2).norm(), kEpsilon);
  // Confirms orthogonality.
  EXPECT_NEAR(0.0, R.col(0).dot(R.col(1)), kEpsilon);
  EXPECT_NEAR(0.0, R.col(0).dot(R.col(2)), kEpsilon);
  EXPECT_NEAR(0.0, R.col(1).dot(R.col(2)), kEpsilon);
  // Test right-handedness.
  EXPECT_TRUE(R.col(0).isApprox(R.col(1).cross(R.col(2))));
  EXPECT_TRUE(R.col(1).isApprox(R.col(2).cross(R.col(0))));
  EXPECT_TRUE(R.col(2).isApprox(R.col(0).cross(R.col(1))));
}

// Tests the contact frame to confirm that a robust, right-handed orthonormal
// frame is generated.
GTEST_TEST(rigid_body_plant_test, TestContactFrameCreation) {
  Vector3<double> z;

  // Case 1: z-axis is simply world aligned.
  z << 1, 0, 0;
  Matrix3<double> R_WL = RigidBodyPlant<double>::ComputeBasisFromZ(z);
  ExpectOrthonormal(R_WL);
  EXPECT_EQ(z, R_WL.col(2));

  // Case 2: z-axis is *slightly* off of z-axis.
  z << 1, 0.01, 0.01;
  z = z.normalized();
  R_WL = RigidBodyPlant<double>::ComputeBasisFromZ(z);
  ExpectOrthonormal(R_WL);
  EXPECT_EQ(z, R_WL.col(2));

  // Case 3: z-axis points into the "middle" of the "first" quadrant.
  z << 1, 1, 1;
  z = z.normalized();
  R_WL = RigidBodyPlant<double>::ComputeBasisFromZ(z);
  ExpectOrthonormal(R_WL);
  EXPECT_EQ(z, R_WL.col(2));

  // Case 4: z-axis points in direction with negative components.
  z << -1, -1, 1;
  z = z.normalized();
  R_WL = RigidBodyPlant<double>::ComputeBasisFromZ(z);
  ExpectOrthonormal(R_WL);
  EXPECT_EQ(z, R_WL.col(2));
}

// Verifies that various model-instance-specific accessor methods work.
GTEST_TEST(RigidBodyPlantTest, InstancePortTest) {
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/three_dof_robot.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree_ptr.get());
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      Vector3d(1., 1., 0));
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/four_dof_robot.urdf",
      drake::multibody::joints::kFixed, weld_to_frame, tree_ptr.get());

  RigidBodyPlant<double> plant(move(tree_ptr));

  EXPECT_EQ(plant.get_num_states(), 14);
  EXPECT_EQ(plant.get_input_size(), 7);
  EXPECT_EQ(plant.get_output_size(), 14);

  EXPECT_EQ(plant.get_num_actuators(0), 3);
  EXPECT_EQ(plant.get_num_positions(0), 3);
  EXPECT_EQ(plant.get_num_velocities(0), 3);
  EXPECT_EQ(plant.get_num_states(0), 6);
  EXPECT_EQ(plant.get_num_actuators(1), 4);
  EXPECT_EQ(plant.get_num_positions(1), 4);
  EXPECT_EQ(plant.get_num_velocities(1), 4);
  EXPECT_EQ(plant.get_num_states(1), 8);

  // TODO(liang.fok) The following has a bug, see #4697.
  const RigidBodyTree<double>& tree = plant.get_rigid_body_tree();
  const std::map<std::string, int> position_name_to_index_map =
      tree.computePositionNameToIndexMap();
  const int joint4_world = position_name_to_index_map.at("joint4");
  ASSERT_EQ(joint4_world, 6);
  const int joint4_instance =
      plant.FindInstancePositionIndexFromWorldIndex(1, joint4_world);
  EXPECT_EQ(joint4_instance, 3);
  EXPECT_ANY_THROW(
      plant.FindInstancePositionIndexFromWorldIndex(0, joint4_world));
}

GTEST_TEST(rigid_body_plant_test, BasicTimeSteppingTest) {
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/multibody/models/box.urdf",
      drake::multibody::joints::kQuaternion, nullptr /* weld to frame */,
      tree_ptr.get());

  const double timestep = 0.1;
  RigidBodyPlant<double> continuous_plant(tree_ptr->Clone());
  RigidBodyPlant<double> time_stepping_plant(move(tree_ptr), timestep);

  auto continuous_context = continuous_plant.AllocateContext();
  continuous_plant.SetDefaults(continuous_context.get());

  auto time_stepping_context = time_stepping_plant.AllocateContext();
  time_stepping_plant.SetDefaults(time_stepping_context.get());

  // Check that the time-stepping model has the same states as the continuous,
  // but as discrete state.
  EXPECT_TRUE(continuous_context->has_only_continuous_state());
  EXPECT_TRUE(time_stepping_context->has_only_discrete_state());
  EXPECT_EQ(continuous_context->get_continuous_state()->size(),
            time_stepping_context->get_discrete_state(0)->size());

  // Check that the dynamics of the time-stepping model match the
  // (backwards-)Euler approximation of the continuous time dynamics.
  auto derivatives = continuous_plant.AllocateTimeDerivatives();
  continuous_plant.CalcTimeDerivatives(*continuous_context, derivatives.get());
  auto updates = time_stepping_plant.AllocateDiscreteVariables();
  DiscreteEvent<double> update_event;
  update_event.action = DiscreteEvent<double>::kDiscreteUpdateAction;
  time_stepping_plant.CalcDiscreteVariableUpdates(*time_stepping_context,
                                                  update_event, updates.get());

  const VectorXd x = continuous_context->get_continuous_state()->CopyToVector();
  EXPECT_TRUE(CompareMatrices(
      x, time_stepping_context->get_discrete_state(0)->CopyToVector()));

  const VectorXd q = continuous_context->get_continuous_state()
                         ->get_generalized_position()
                         .CopyToVector();
  const VectorXd v = continuous_context->get_continuous_state()
                         ->get_generalized_velocity()
                         .CopyToVector();

  const VectorXd vn =
      v + timestep * derivatives->get_generalized_velocity().CopyToVector();

  auto kinsol = continuous_plant.get_rigid_body_tree().doKinematics(q, v);
  const VectorXd qn =
      q +
      timestep *
          continuous_plant.get_rigid_body_tree().transformVelocityToQDot(kinsol,
                                                                         vn);
  VectorXd xn(qn.rows() + vn.rows());
  xn << qn, vn;

  EXPECT_TRUE(
      CompareMatrices(updates->get_discrete_state(0)->CopyToVector(), xn));
}

}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
