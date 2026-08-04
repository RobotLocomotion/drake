#include "drake/systems/controllers/inverse_dynamics.h"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/controllers/test_utilities/compute_torque.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using drake::multibody::MultibodyPlant;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace controllers {
namespace {

class InverseDynamicsTest : public ::testing::Test {
 protected:
  void Init(std::unique_ptr<MultibodyPlant<double>> plant,
            const InverseDynamics<double>::InverseDynamicsMode mode,
            std::unique_ptr<Context<double>> plant_context = nullptr) {
    multibody_plant_ = std::move(plant);
    multibody_context_ = (plant_context == nullptr)
                             ? multibody_plant_->CreateDefaultContext()
                             : std::move(plant_context);
    inverse_dynamics_ = make_unique<InverseDynamics<double>>(
        multibody_plant_.get(), mode, multibody_context_.get());
    FinishInit(mode);
  }

  void FinishInit(const InverseDynamics<double>::InverseDynamicsMode mode) {
    inverse_dynamics_context_ = inverse_dynamics_->CreateDefaultContext();
    output_ = inverse_dynamics_->AllocateOutput();

    // Checks that the system has no state.
    EXPECT_TRUE(inverse_dynamics_context_->is_stateless());

    // Checks that the number of input and output ports are as desired.
    if (mode == InverseDynamics<double>::kGravityCompensation) {
      EXPECT_EQ(inverse_dynamics_->num_input_ports(), 1);
    } else {
      EXPECT_EQ(inverse_dynamics_->num_input_ports(), 2);
    }
    EXPECT_EQ(inverse_dynamics_->num_output_ports(), 1);
  }

  void CheckGravityTorque(const Eigen::VectorXd& position) {
    CheckTorque(position, VectorXd::Zero(num_velocities()),
                VectorXd::Zero(num_velocities()));
  }

  void CheckTorque(const Eigen::VectorXd& position,
                   const Eigen::VectorXd& velocity,
                   const Eigen::VectorXd& acceleration_desired) {
    // Desired acceleration.
    VectorXd vd_d = VectorXd::Zero(num_velocities());
    if (!inverse_dynamics_->is_pure_gravity_compensation()) {
      vd_d = acceleration_desired;
    }

    VectorXd state_input(num_positions() + num_velocities());
    state_input << position, velocity;
    inverse_dynamics_->get_input_port_estimated_state().FixValue(
        inverse_dynamics_context_.get(), state_input);

    if (!inverse_dynamics_->is_pure_gravity_compensation()) {
      inverse_dynamics_->get_input_port_desired_acceleration().FixValue(
          inverse_dynamics_context_.get(), vd_d);
    }

    // Hook input of the expected size.
    inverse_dynamics_->CalcOutput(*inverse_dynamics_context_, output_.get());

    // Compute the expected torque.
    VectorXd expected_torque;
    ASSERT_TRUE(multibody_plant_.get());
    ASSERT_TRUE(multibody_context_);
    expected_torque = controllers_test::ComputeTorque(
        *multibody_plant_, position, velocity, vd_d, multibody_context_.get());

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                                1e-10, MatrixCompareType::absolute));
  }

  // Determines whether gravity is modeled by checking the generalized forces
  // due to gravity.
  bool GravityModeled(const VectorXd& q) const {
    multibody_plant_->SetPositions(multibody_context_.get(), q);
    // Verify that gravitational forces are nonzero (validating that the tree
    // is put into the proper configuration and gravity is modeled).
    const VectorXd tau_g =
        multibody_plant_->CalcGravityGeneralizedForces(*multibody_context_);
    return tau_g.norm() > std::numeric_limits<double>::epsilon();
  }

 private:
  int num_positions() const {
    DRAKE_DEMAND(multibody_plant_.get() != nullptr);
    return multibody_plant_->num_positions();
  }

  int num_velocities() const {
    DRAKE_DEMAND(multibody_plant_.get() != nullptr);
    return multibody_plant_->num_velocities();
  }

  std::unique_ptr<MultibodyPlant<double>> multibody_plant_;
  std::unique_ptr<InverseDynamics<double>> inverse_dynamics_;
  std::unique_ptr<Context<double>> inverse_dynamics_context_;
  std::unique_ptr<Context<double>> multibody_context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that inverse dynamics returns the expected torque for a given state and
// desired acceleration for the iiwa arm.
TEST_F(InverseDynamicsTest, InverseDynamicsTest) {
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(mbp.get()).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));

  // Add gravitational forces, finalize the model, and transfer ownership.
  mbp->mutable_gravity_field().set_gravity_vector(-9.8 *
                                                  Vector3<double>::UnitZ());
  mbp->Finalize();
  Init(std::move(mbp),
       InverseDynamics<double>::InverseDynamicsMode::kInverseDynamics);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd vd_d = Eigen::VectorXd::Zero(7);
  for (int i = 0; i < 7; ++i) {
    q[i] = i * 0.1 - 0.3;
    v[i] = i - 3;
    vd_d[i] = i - 3;
  }

  // Check that gravity is modeled.
  EXPECT_TRUE(GravityModeled(q));

  CheckTorque(q, v, vd_d);
}

// Tests that inverse dynamics returns the expected torque for a given state and
// desired acceleration for the iiwa arm with a custom context.
TEST_F(InverseDynamicsTest, InverseDynamicsWithCustomContextTest) {
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(mbp.get()).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));
  mbp->Finalize();

  // Create custom context.
  auto custom_context = mbp->CreateDefaultContext();
  const auto& iiwa_link_7 = mbp->GetBodyByName("iiwa_link_7");
  iiwa_link_7.SetMass(custom_context.get(), 10.0);

  // Transfer ownership.
  Init(std::move(mbp),
       InverseDynamics<double>::InverseDynamicsMode::kInverseDynamics,
       std::move(custom_context));

  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd vd_d = Eigen::VectorXd::Zero(7);
  for (int i = 0; i < 7; ++i) {
    q[i] = i * 0.1 - 0.3;
    v[i] = i - 3;
    vd_d[i] = i - 3;
  }

  // Check torques with the custom context.
  CheckTorque(q, v, vd_d);
}

// Tests that the expected value of the gravity compensating torque and the
// value computed by the InverseDynamics in pure gravity compensation mode
// for a given joint configuration of the KUKA IIWA Arm are identical.
TEST_F(InverseDynamicsTest, GravityCompensationTest) {
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  const std::string url =
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  multibody::Parser(mbp.get()).AddModelsFromUrl(url);
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));

  mbp->mutable_gravity_field().set_gravity_vector(Vector3<double>::Zero());

  // Finalize the model and transfer ownership.
  mbp->Finalize();
  Init(std::move(mbp),
       InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  // Verify that gravity is *not* modeled.
  EXPECT_FALSE(GravityModeled(robot_position));

  // Re-initialize the model so we can add gravity.
  mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(mbp.get()).AddModelsFromUrl(url);
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));

  // Add gravitational forces, finalize the model, and transfer ownership.
  mbp->mutable_gravity_field().set_gravity_vector(-9.8 *
                                                  Vector3<double>::UnitZ());
  mbp->Finalize();
  Init(std::move(mbp),
       InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  // Verify that gravity is modeled.
  EXPECT_TRUE(GravityModeled(robot_position));

  CheckGravityTorque(robot_position);
}

GTEST_TEST(AdditionalInverseDynamicsTest, ScalarConversion) {
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(mbp.get()).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));
  mbp->Finalize();
  const int num_states = mbp->num_multibody_states();

  InverseDynamics<double> id(mbp.get());
  // Test AutoDiffXd.
  auto id_ad = systems::System<double>::ToAutoDiffXd(id);
  // Check the multibody plant.
  EXPECT_EQ(id_ad->get_input_port_estimated_state().size(), num_states);
  // Check the mode.
  EXPECT_FALSE(id_ad->is_pure_gravity_compensation());

  // Test Expression.
  auto id_sym = systems::System<double>::ToSymbolic(id);
  EXPECT_EQ(id_sym->get_input_port_estimated_state().size(), num_states);
  EXPECT_FALSE(id_sym->is_pure_gravity_compensation());

  // Create custom context.
  auto custom_context = mbp->CreateDefaultContext();
  const auto& iiwa_link_7 = mbp->GetBodyByName("iiwa_link_7");
  iiwa_link_7.SetMass(custom_context.get(), 10.0);

  auto mbp_copy = drake::multibody::MultibodyPlant<double>::Clone(*mbp);
  InverseDynamics<double> id_with_modified_mass(
      std::move(mbp), InverseDynamics<double>::kGravityCompensation,
      custom_context.get());

  // Test AutoDiffXd.
  id_ad = systems::System<double>::ToAutoDiffXd(id_with_modified_mass);
  // Check the multibody plant.
  EXPECT_EQ(id_ad->get_input_port_estimated_state().size(), num_states);
  // Check the mode.
  EXPECT_TRUE(id_ad->is_pure_gravity_compensation());

  // Test Expression.
  id_sym = systems::System<double>::ToSymbolic(id_with_modified_mass);
  EXPECT_EQ(id_sym->get_input_port_estimated_state().size(), num_states);
  EXPECT_TRUE(id_sym->is_pure_gravity_compensation());

  // Test AutoDiffXd to double.
  auto id_double = systems::System<AutoDiffXd>::ToScalarType<double>(*id_ad);
  // Check the multibody plant.
  EXPECT_EQ(id_double->get_input_port_estimated_state().size(), num_states);
  // Check the mode.
  EXPECT_TRUE(id_double->is_pure_gravity_compensation());
  // Check gravity torque with custom context.
  custom_context = mbp_copy->CreateDefaultContext();
  iiwa_link_7.SetMass(custom_context.get(), 10.0);
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  VectorXd state_input(14);
  state_input << robot_position, Eigen::VectorXd::Zero(7);
  auto id_double_context = id_double->CreateDefaultContext();
  id_double->get_input_port_estimated_state().FixValue(id_double_context.get(),
                                                       state_input);
  auto output = id_double->AllocateOutput();
  id_double->CalcOutput(*id_double_context, output.get());
  VectorXd expected_torque;
  expected_torque = controllers_test::ComputeTorque(
      *mbp_copy, robot_position, Eigen::VectorXd::Zero(7),
      Eigen::VectorXd::Zero(7), custom_context.get());
  auto output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                              1e-14, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
