#include "drake/systems/controllers/inverse_dynamics.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/math/spatial_acceleration.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/controllers/test_utilities/compute_torque.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;
using drake::multibody::multibody_plant::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {
namespace {

class InverseDynamicsTest : public ::testing::Test {
 protected:
  void Init(std::unique_ptr<MultibodyPlant<double>> plant,
            const InverseDynamics<double>::InverseDynamicsMode mode) {
    multibody_plant_ = std::move(plant);
    multibody_context_ = multibody_plant_->CreateDefaultContext();
    inverse_dynamics_ = make_unique<InverseDynamics<double>>(
        multibody_plant_.get(), mode);
    FinishInit(mode);
  }

  void FinishInit(const InverseDynamics<double>::InverseDynamicsMode mode) {
    inverse_dynamics_context_ = inverse_dynamics_->CreateDefaultContext();
    output_ = inverse_dynamics_->AllocateOutput();

    // Checks that the number of input ports in the Gravity Compensator system
    // and the Context are consistent.
    if (mode == InverseDynamics<double>::kGravityCompensation) {
      EXPECT_EQ(inverse_dynamics_->get_num_input_ports(), 1);
      EXPECT_EQ(inverse_dynamics_context_->get_num_input_ports(), 1);
    } else {
      EXPECT_EQ(inverse_dynamics_->get_num_input_ports(), 2);
      EXPECT_EQ(inverse_dynamics_context_->get_num_input_ports(), 2);
    }

    // Checks that no state variables are allocated in the context.
    EXPECT_EQ(inverse_dynamics_context_->get_continuous_state().size(), 0);

    // Checks that the number of output ports in the Gravity Compensator system
    // and the SystemOutput are consistent.
    EXPECT_EQ(output_->get_num_ports(), 1);
    EXPECT_EQ(inverse_dynamics_->get_num_output_ports(), 1);
  }

  void CheckGravityTorque(const Eigen::VectorXd& position) {
    CheckTorque(position, VectorXd::Zero(num_velocities()),
                VectorXd::Zero(num_velocities()));
  }

  void CheckTorque(const Eigen::VectorXd& position,
                   const Eigen::VectorXd& velocity,
                   const Eigen::VectorXd& acceleration_desired) {
    // desired acceleration.
    VectorXd vd_d = VectorXd::Zero(num_velocities());
    if (!inverse_dynamics_->is_pure_gravity_compensation()) {
      vd_d = acceleration_desired;
    }

    auto state_input = make_unique<BasicVector<double>>(
        num_positions() + num_velocities());
    state_input->get_mutable_value() << position, velocity;
    inverse_dynamics_context_->FixInputPort(
        inverse_dynamics_->get_input_port_estimated_state().get_index(),
        std::move(state_input));

    if (!inverse_dynamics_->is_pure_gravity_compensation()) {
      auto vd_input =
          make_unique<BasicVector<double>>(num_velocities());
      vd_input->get_mutable_value() << vd_d;
      inverse_dynamics_context_->FixInputPort(
          inverse_dynamics_->get_input_port_desired_acceleration().get_index(),
          std::move(vd_input));
    }

    // Hook input of the expected size.
    inverse_dynamics_->CalcOutput(*inverse_dynamics_context_, output_.get());

    // Compute the expected torque.
    VectorXd expected_torque;
    ASSERT_TRUE(multibody_plant_.get());
    ASSERT_TRUE(multibody_context_.get());
    expected_torque = controllers_test::ComputeTorque(
        *multibody_plant_, position, velocity, vd_d,
        multibody_context_.get());

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                                1e-10, MatrixCompareType::absolute));
  }

  // Determines whether gravity is modeled by checking the generalized forces
  // due to gravity.
  bool GravityModeled(const VectorXd& q) const {
    // The state can only be altered in this way because the state is known to
    // be continuous while the state can generally be discrete for MBT/MBP.
    multibody_context_->get_mutable_continuous_state().
        get_mutable_generalized_position().SetFromVector(q);

    // Verify that gravitational forces are nonzero (validating that the tree
    // is put into the proper configuration and gravity is modeled).
    const auto& tree = multibody_plant_->tree();
    return tree.CalcGravityGeneralizedForces(*multibody_context_).norm() >
              std::numeric_limits<double>::epsilon();
  }

 private:
  int num_positions() const {
    DRAKE_DEMAND(multibody_plant_.get() != nullptr);
    return multibody_plant_->tree().num_positions();
  }

  int num_velocities() const {
    DRAKE_DEMAND(multibody_plant_.get() != nullptr);
    return multibody_plant_->tree().num_velocities();
  }

  std::unique_ptr<MultibodyPlant<double>> multibody_plant_;
  std::unique_ptr<InverseDynamics<double>> inverse_dynamics_;
  std::unique_ptr<Context<double>> inverse_dynamics_context_;
  std::unique_ptr<Context<double>> multibody_context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the expected value of the gravity compensating torque and the
// value computed by the InverseDynamics in pure gravity compensation mode
// for a given joint configuration of the KUKA IIWA Arm are identical.
TEST_F(InverseDynamicsTest, GravityCompensationTest) {
  auto mbp = std::make_unique<MultibodyPlant<double>>();
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  multibody::parsing::AddModelFromSdfFile(full_name, mbp.get());
  mbp->WeldFrames(mbp->world_frame(),
                  mbp->GetFrameByName("iiwa_link_0"));

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
  mbp = std::make_unique<MultibodyPlant<double>>();
  multibody::parsing::AddModelFromSdfFile(full_name, mbp.get());
  mbp->WeldFrames(mbp->world_frame(),
                  mbp->GetFrameByName("iiwa_link_0"));

  // Add gravitational forces, finalize the model, and transfer ownership.
  mbp->AddForceElement<multibody::UniformGravityFieldElement>(-9.8 *
      Vector3<double>::UnitZ());
  mbp->Finalize();
  Init(std::move(mbp),
       InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  // Verify that gravity is modeled.
  EXPECT_TRUE(GravityModeled(robot_position));

  CheckGravityTorque(robot_position);
}

// Tests that inverse dynamics returns the expected torque for a given state and
// desired acceleration for the iiwa arm.
TEST_F(InverseDynamicsTest, InverseDynamicsTest) {
  auto mbp = std::make_unique<MultibodyPlant<double>>();
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  multibody::parsing::AddModelFromSdfFile(full_name, mbp.get());
  mbp->WeldFrames(mbp->world_frame(),
                  mbp->GetFrameByName("iiwa_link_0"));

  // Add gravitational forces, finalize the model, and transfer ownership.
  mbp->AddForceElement<multibody::UniformGravityFieldElement>(-9.8 *
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

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
