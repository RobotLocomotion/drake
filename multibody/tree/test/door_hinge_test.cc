#include "drake/multibody/tree/door_hinge.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

class DoorHingeTester {
 public:
  // Input argument door_hinge is aliased and must be valid whenever this
  // class exists.
  explicit DoorHingeTester(const DoorHinge<double>& door_hinge)
      : door_hinge_(door_hinge) {}

  double CalcHingeFrictionalTorque(double angular_velocity,
                                   const DoorHingeConfig& config) const {
    return door_hinge_.CalcHingeFrictionalTorque(angular_velocity, config);
  }

  double CalcHingeSpringTorque(double angle,
                               const DoorHingeConfig& config) const {
    return door_hinge_.CalcHingeSpringTorque(angle, config);
  }

  const DoorHingeConfig& door_hinge_config() const {
    return door_hinge_.config_;
  }

  const DoorHinge<double>& door_hinge() const { return door_hinge_; }

 private:
  const DoorHinge<double>& door_hinge_;
};

namespace {

constexpr double kMass = 1.0;
constexpr double kPrincipleInertia = 1.0;
constexpr double kAngle = 0.01;
constexpr double kAngularRate = 1.0;
constexpr double kIntegrationTimeStep = 1e-6;
constexpr double kTotalSimTime = 0.05;

class DoorHingeTest : public ::testing::Test {
 protected:
  // Based on the DoorHingeConfig to set up the door hinge joint and the plant.
  const DoorHingeTester& BuildDoorHingeTester(const DoorHingeConfig& config) {
    plant_ = std::make_unique<MultibodyPlant<double>>(kIntegrationTimeStep);

    // Neither the mass nor the center of mass position will affect the purpose
    // of the tests. They are set arbitrarily. The rotational inertia is also
    // set arbitrarily.
    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        kMass, drake::Vector3<double>::Zero(),
        kMass * UnitInertia<double>::TriaxiallySymmetric(kPrincipleInertia));
    auto& body1 = plant_->AddRigidBody("body1", M_B);

    revolute_joint_ = &plant_->AddJoint<RevoluteJoint>(
        "Joint1", plant_->world_body(), std::nullopt, body1, std::nullopt,
        Eigen::Vector3d::UnitZ(), 0.0);

    door_hinge_ = &plant_->AddForceElement<DoorHinge>(*revolute_joint_, config);

    plant_->mutable_gravity_field().set_gravity_vector(Vector3<double>::Zero());
    // Finish building the model and create the context.
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();

    // Create a tester for testing purpose.
    door_hinge_tester_ = std::make_unique<DoorHingeTester>(*door_hinge_);

    return *door_hinge_tester_;
  }

  void SetHingeJointState(double angle, double angular_rate) {
    revolute_joint_->set_angle(plant_context_.get(), angle);
    revolute_joint_->set_angular_rate(plant_context_.get(), angular_rate);
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;

  const RevoluteJoint<double>* revolute_joint_{nullptr};
  const DoorHinge<double>* door_hinge_{nullptr};

  std::unique_ptr<DoorHingeTester> door_hinge_tester_;
};

DoorHingeConfig no_forces_config() {
  DoorHingeConfig config;
  config.spring_zero_angle_rad = 0;
  config.spring_constant = 0;
  config.dynamic_friction_torque = 0;
  config.static_friction_torque = 0;
  config.viscous_friction = 0;
  config.catch_width = 0;
  config.catch_torque = 0;
  return config;
}

// With the condition that there is only frictional torque, this function
// confirms that a) the potential energy and conservative power should be zero;
// b) on-conservative power equals to the corresponding torque times velocity.
void TestFrictionOnlyEnergyAndPower(const DoorHingeTester& dut,
                                    const systems::Context<double>& context,
                                    const MultibodyPlant<double>& plant,
                                    double angular_rate) {
  const double potential_energy_half_qc = dut.door_hinge().CalcPotentialEnergy(
      context, plant.EvalPositionKinematics(context));
  EXPECT_EQ(potential_energy_half_qc, 0.0);

  const double conserv_power = dut.door_hinge().CalcConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));
  EXPECT_EQ(conserv_power, 0.0);

  // Verify the non-conservative power
  const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));
  EXPECT_EQ(non_conserv_power, dut.CalcHingeFrictionalTorque(
                                   angular_rate, dut.door_hinge_config()) *
                                   angular_rate);
}

// With the condition that there is only spring related torque, this function
// confirms that a) the potential energy equals to the corresponding torque
// times velocity; b) on-conservative power should be zero.
void TestSpringTorqueOnlyPower(const DoorHingeTester& dut,
                               const systems::Context<double>& context,
                               const MultibodyPlant<double>& plant,
                               double angle, double angular_rate) {
  auto spring_power = [&dut](double q, double v) {
    return dut.CalcHingeSpringTorque(q, dut.door_hinge_config()) * v;
  };

  const double conserv_power = dut.door_hinge().CalcConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));
  EXPECT_EQ(conserv_power, spring_power(angle, angular_rate));

  const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));
  EXPECT_EQ(non_conserv_power, 0.0);
}

// This function integrates the conservative power (P) to get the corresponding
// energy (PE), i.e., PE = -∫Pdt. We assume the hinge joint moves from zero to
// the input `angle` with a constant `angular_rate`.
double IntegrateConservativePower(const DoorHingeTester& dut,
                                  double target_angle, double angular_rate) {
  auto conserv_power = [&dut](double q, double v) {
    return dut.CalcHingeSpringTorque(q, dut.door_hinge_config()) * v;
  };

  double angle = 0.0;
  double pe_integrated = 0.0;
  while (angle < target_angle) {
    pe_integrated -= conserv_power(angle, angular_rate) * kIntegrationTimeStep;
    angle += angular_rate * kIntegrationTimeStep;
  }
  return pe_integrated;
}

// Verify the torques and the energy should be zero when the config parameters
// are all zero.
TEST_F(DoorHingeTest, ZeroTest) {
  DoorHingeConfig config = no_forces_config();
  DoorHingeTester dut = BuildDoorHingeTester(config);

  // If no frictions, springs, etc. are applied, our torques should be 0.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test energy should be zero at a default condition.
  const double potential_energy_1 = door_hinge_->CalcPotentialEnergy(
      *plant_context_, plant().EvalPositionKinematics(*plant_context_));
  EXPECT_EQ(potential_energy_1, 0.0);
  // Test energy should be zero at a non-default condition.
  SetHingeJointState(0.2, 0.1);
  const double potential_energy_2 = door_hinge_->CalcPotentialEnergy(
      *plant_context_, plant().EvalPositionKinematics(*plant_context_));
  EXPECT_EQ(potential_energy_2, 0.0);
}

// Test the case with only the torional spring torque, the corresponding energy
// and power are computed correctly at different states.
TEST_F(DoorHingeTest, SpringTest) {
  DoorHingeConfig config = no_forces_config();
  config.spring_constant = 1;
  DoorHingeTester dut = BuildDoorHingeTester(config);

  // Springs make spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_LT(dut.CalcHingeSpringTorque(1., config), 0);  // Pulls toward zero.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);

  // Test potential energy at non-zero angle.
  SetHingeJointState(kAngle, kAngularRate);
  const double integrated_potential_energy =
      IntegrateConservativePower(dut, kAngle, kAngularRate);
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      *plant_context_, plant().EvalPositionKinematics(*plant_context_));
  EXPECT_NEAR(potential_energy, integrated_potential_energy,
              kIntegrationTimeStep);

  // Test the powers are computed correctly.
  TestSpringTorqueOnlyPower(dut, *plant_context_, plant(), kAngle,
                            kAngularRate);
}

// Test the case with only the catch spring torque, the corresponding
// energy and power are computed correctly at different states.
TEST_F(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = no_forces_config();
  config.catch_width = 2 * kAngle;
  config.catch_torque = 1.0;
  DoorHingeTester dut = BuildDoorHingeTester(config);

  // The catch makes spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);

  // Resists closing.
  EXPECT_GT(dut.CalcHingeSpringTorque(config.catch_width, config), 0);
  // Tipover point.
  EXPECT_EQ(dut.CalcHingeSpringTorque(config.catch_width / 2, config), 0);
  // Detent pulls in.
  EXPECT_LT(dut.CalcHingeSpringTorque(0., config), 0);

  // Verify that the potential energy with only the catch spring torque at zero
  // position and the catch_width position should be 0.
  const double potential_energy_q0 = dut.door_hinge().CalcPotentialEnergy(
      *plant_context_, plant().EvalPositionKinematics(*plant_context_));
  EXPECT_EQ(potential_energy_q0, 0.0);

  SetHingeJointState(config.catch_width, 0.0);
  const double potential_energy_qc = dut.door_hinge().CalcPotentialEnergy(
      *plant_context_, plant().EvalPositionKinematics(*plant_context_));
  EXPECT_EQ(potential_energy_qc, 0.0);

  // Test the energy from power integration.
  SetHingeJointState(kAngle, kAngularRate);
  const double integrated_potential_energy =
      IntegrateConservativePower(dut, kAngle, kAngularRate);
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      *plant_context_, plant().EvalPositionKinematics(*plant_context_));
  EXPECT_NEAR(potential_energy, integrated_potential_energy,
              kIntegrationTimeStep);

  // Verify the power terms are computed correctly.
  // Test the powers are computed correctly
  TestSpringTorqueOnlyPower(dut, *plant_context_, plant(), kAngle,
                            kAngularRate);
}

TEST_F(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.static_friction_torque = 1;
  DoorHingeTester dut = BuildDoorHingeTester(config);

  // Friction opposes tiny motion, but falls away with substantial motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), 0, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, *plant_context_, plant(), kAngularRate);
}

TEST_F(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.dynamic_friction_torque = 1;
  DoorHingeTester dut = BuildDoorHingeTester(config);

  // Friction opposes any motion, even tiny motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), -1, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, *plant_context_, plant(), kAngularRate);
}

TEST_F(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.viscous_friction = 1;
  DoorHingeTester dut = BuildDoorHingeTester(config);

  // Friction opposes motion proprotionally.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), -1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-1., config), 1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-2., config), 2);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, *plant_context_, plant(), kAngularRate);
}

// Returns the total conservative energy of the constructed plant. It assumes
// there is only one non-world body and one joint.
double CalcConservativeEnergy(const systems::Simulator<double>& simulator,
                              const DoorHingeTester& dut,
                              const MultibodyPlant<double>& plant) {
  const auto& context = simulator.get_context();
  const auto& state = context.get_discrete_state_vector();
  DRAKE_THROW_UNLESS(state.size() == 2);
  // Calculate the kinetic energy. If m is inertia, v should be angular rate.
  auto calc_kinetic_energy = [](double m, double v) { return 0.5 * m * v * v; };

  const double kinetic_energy =
      calc_kinetic_energy(kPrincipleInertia, state[1]);
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      context, plant.EvalPositionKinematics(context));
  return kinetic_energy + potential_energy;
}

// Runs a simulator to forward the system (i.e., `plant`) from the
// `init_state`and confirm that the non-conservative energy should equal to the
// change of the conservative energy within the given tolerance. It returns
// the conservative energy loss defined as the initial minus the final. The loss
// should be non-negative.
// N.B. Since the MultibodyPlant does not provide methods to compute the
// non-conservative power, the DoorHingeTester has to be passed in for this
// purpose.
// N.B. for simplicity, instead of building a new system and defining a
// continuous state for the non-conservative energy, we use `set_monitor()`
// function to explicitly integrate the non-conservative power to get
// non-conservative energy. Therefore, there is no error-control or anything. If
// the user chose to use a continuous system, the error between energy loss
// (which computed from the error-controlled RK3 integrator) and the
// non-conservative energy can be big, around 1e-3. If chose to use a discrete
// system, this error will be much smaller (same magnitude of the integration
// step) since both terms use the same integration method.
double TestEnergyConservative(const MultibodyPlant<double>& plant,
                              const DoorHingeTester& dut,
                              const Eigen::Vector2d& init_state,
                              double tolerance) {
  systems::Simulator<double> simulator(plant);
  simulator.Initialize();

  double non_conserv_energy = 0.0;
  double prev_time = 0.0;
  simulator.set_monitor([&plant, &dut, &prev_time, &non_conserv_energy](
                            const systems::Context<double>& root_context) {
    // Compute delta time and update previous time.
    const double curr_time = root_context.get_time();
    const double delta_time = curr_time - prev_time;
    prev_time = curr_time;

    const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
        root_context, plant.EvalPositionKinematics(root_context),
        plant.EvalVelocityKinematics(root_context));
    non_conserv_energy += non_conserv_power * delta_time;

    // Assume there is only one joint.
    const auto& state = root_context.get_discrete_state_vector();
    DRAKE_THROW_UNLESS(state.size() == 2);
    if (state[0] < 0.0 && state[1] < 0.0) {
      return systems::EventStatus::ReachedTermination(
          &plant, "Simulation achieved the desired goal.");
    }
    return systems::EventStatus::Succeeded();
  });

  // Set initial condition of the simulation
  auto& init_plant_context = simulator.get_mutable_context();
  init_plant_context.get_mutable_discrete_state_vector().SetFromVector(
      init_state);

  const double init_conserv_energy =
      CalcConservativeEnergy(simulator, dut, plant);

  simulator.AdvanceTo(kTotalSimTime);

  const double final_conserv_energy =
      CalcConservativeEnergy(simulator, dut, plant);

  const double energy_loss = init_conserv_energy - final_conserv_energy;
  EXPECT_NEAR(energy_loss, -non_conserv_energy, tolerance);

  return energy_loss;
}

// Confirm no energy loss if there is no non-conservative torques.
TEST_F(DoorHingeTest, EnergyTestWithOnlyConservativeTorques) {
  DoorHingeConfig config = no_forces_config();
  config.spring_constant = 6;
  config.spring_zero_angle_rad = 1.0;
  config.catch_width = 2 * kAngle;
  config.catch_torque = 1.0;

  const DoorHingeTester dut = BuildDoorHingeTester(config);
  const Eigen::Vector2d init_state{0, 1.0};
  const double energy_loss = TestEnergyConservative(*plant_, dut, init_state,
                                                    10 * kIntegrationTimeStep);
  EXPECT_NEAR(energy_loss, 0.0, 10 * kIntegrationTimeStep);
}

// Confirm the energy loss should equal to the non-conservative energy, i.e.,
// dissipative energy. The loss should be greater than 0.0.
TEST_F(DoorHingeTest, EnergyTestWithAllTorques) {
  // Use the default door hinge configuration.
  const DoorHingeConfig config;
  const DoorHingeTester dut = BuildDoorHingeTester(config);
  const Eigen::Vector2d init_state{0, 1.0};
  const double energy_loss = TestEnergyConservative(*plant_, dut, init_state,
                                                    10 * kIntegrationTimeStep);
  EXPECT_GT(energy_loss, 0.0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
