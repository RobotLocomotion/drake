#include "drake/multibody/tree/door_hinge.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

namespace {

constexpr double kMass = 1.0;              // [Kg]
constexpr double kPrincipalInertia = 1.0;  // [Kg m^2]
constexpr double kAngle = 0.5;             // [rad]
constexpr double kAngularRate = 1.0;       // [rad/s]
constexpr double kMaximumTimeStep = 1e-5;  // [s]
constexpr double kTotalSimTime = 0.5;      // [s]
constexpr double kIntegrationAccuracy = 1e-9;
const char kRevoluteJointName[] = "RevoluteJoint";

class DoorHingeTest : public ::testing::Test {
 protected:
  // Based on the DoorHingeConfig, this function sets up a plant that includes a
  // DoorHinge force element. Then, it returns a const reference of the internal
  // DoorHinge for testing purpose.
  const DoorHinge<double>& BuildDoorHinge(const DoorHingeConfig& config) {
    plant_ = std::make_unique<MultibodyPlant<double>>(kMaximumTimeStep);

    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        kMass, drake::Vector3<double>::Zero(),
        kMass * UnitInertia<double>::TriaxiallySymmetric(kPrincipalInertia));
    auto& body1 = plant_->AddRigidBody("body1", M_B);

    revolute_joint_ = &plant_->AddJoint<RevoluteJoint>(
        kRevoluteJointName, plant_->world_body(), {}, body1, {},
        Eigen::Vector3d::UnitZ(), 0.0 /*set joint damping to zero*/);

    door_hinge_ = &plant_->AddForceElement<DoorHinge>(*revolute_joint_, config);

    // Set gravity to zero so that we have one less energy term to consider when
    // verifying the energy correctness of the DoorHinge force element.
    plant_->mutable_gravity_field().set_gravity_vector(Vector3<double>::Zero());

    // Finish building the model and create the default context.
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();

    return *door_hinge_;
  }

  void SetHingeJointState(double angle, double angular_rate) {
    revolute_joint_->set_angle(plant_context_.get(), angle);
    revolute_joint_->set_angular_rate(plant_context_.get(), angular_rate);
  }

  double CalcHingePotentialEnergy() const {
    return door_hinge_->CalcPotentialEnergy(
        *plant_context_, plant_->EvalPositionKinematics(*plant_context_));
  }

  double CalcHingeConservativePower() const {
    return door_hinge_->CalcConservativePower(
        *plant_context_, plant_->EvalPositionKinematics(*plant_context_),
        plant_->EvalVelocityKinematics(*plant_context_));
  }

  double CalcHingeNonConservativePower() const {
    return door_hinge_->CalcNonConservativePower(
        *plant_context_, plant_->EvalPositionKinematics(*plant_context_),
        plant_->EvalVelocityKinematics(*plant_context_));
  }

  // This function confirms the potential energy (PE) is computed correctly by
  // comparing it against the result from integrating the conservative power
  // (Pc), i.e. we should have PE = -∫Pcdt. The `InitialValueProblem` class is
  // used for this purpose. The state is defined as `x = {PE, q, v}`, where `q`
  // is the angle and `v` is the angular rate. In particular, this function
  // assumes the hinge joint moves from zero (initial state) to the current
  // angle qₜ with a constant speed (current angular rate vₜ). It also assumes
  // that the initial potential energy is 0 and initial angle is 0.
  // Correspondingly, the ODEs can be defined as `ẋ[0] = Pc`, `ẋ[1] = v`, `ẋ[2]
  // = 0` and the initial value is `x₀ = {0, 0, vₜ}`. Since velocity is
  // constant, the total simulation time can be derived as t = qₜ / vₜ. To
  // guarantee the comparison is numerically meaningful (not affected by
  // integration error), this function requires the value of the potential
  // energy has to be bigger than 1, which is significantly bigger than the
  // integration accuracy (1e-9).
  void TestPotentialEnergyCalculation() {
    const double potential_energy = CalcHingePotentialEnergy();
    DRAKE_DEMAND(std::abs(potential_energy) >= 1.0);

    const auto& door_hinge_temp = door_hinge();
    auto energy_ode = [&door_hinge_temp](
                          const double& t, const VectorX<double>& x,
                          const VectorX<double>& k) -> VectorX<double> {
      unused(t);
      unused(k);
      VectorX<double> ret(x.size());
      ret[0] = -door_hinge_temp.CalcHingeConservativePower(x[1], x[2]);
      ret[1] = x[2];
      ret[2] = 0.0;
      return ret;
    };

    const double target_angle = plant_->GetJointByName(kRevoluteJointName)
                                    .GetOnePosition(*plant_context_);
    const double angular_rate = plant_->GetJointByName(kRevoluteJointName)
                                    .GetOneVelocity(*plant_context_);

    // Set initial value and default parameters of the problem.
    const double kInitialTime = 0.0;
    const VectorX<double> kDefaultParameters = VectorX<double>::Zero(3);
    const VectorX<double> init_state =
        (VectorX<double>(3) << 0.0, 0.0, angular_rate).finished();

    const systems::InitialValueProblem<double>::OdeContext default_values(
        kInitialTime, init_state, kDefaultParameters);
    systems::InitialValueProblem<double> ivp(energy_ode, default_values);
    ivp.get_mutable_integrator().set_target_accuracy(kIntegrationAccuracy);

    // Get the integration time.
    DRAKE_THROW_UNLESS(angular_rate != 0);
    const double integration_time = std::abs(target_angle / angular_rate);
    const VectorX<double> result = ivp.Solve(integration_time);

    // Confirm the velocity is not changed.
    DRAKE_THROW_UNLESS(result[2] == angular_rate);

    // Compute an absolute tolerance from the relative integration accuracy.
    const double tol = std::abs(potential_energy) * kIntegrationAccuracy;
    EXPECT_NEAR(potential_energy, result[0], tol);
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }
  const systems::Context<double>& plant_context() const {
    return *plant_context_;
  }

  const DoorHinge<double>& door_hinge() const { return *door_hinge_; }

  DoorHingeConfig CreateZeroForcesDoorHingeConfig() const {
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

 private:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;

  const RevoluteJoint<double>* revolute_joint_{nullptr};
  const DoorHinge<double>* door_hinge_{nullptr};
};

// Verify the torques and the energy should be zero when the config parameters
// are all zero.
TEST_F(DoorHingeTest, ZeroTest) {
  const DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  const DoorHinge<double>& dut = BuildDoorHinge(config);

  // If no frictions, springs, etc. are applied, our torques should be 0.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1.), 0);

  // Test energy should be zero at the default state.
  EXPECT_EQ(CalcHingePotentialEnergy(), 0.0);

  // Test energy should be zero at a non-zero state.
  SetHingeJointState(kAngle, kAngularRate);
  EXPECT_EQ(CalcHingePotentialEnergy(), 0.0);
}

// Test that scalar conversion produces properly constructed results.
TEST_F(DoorHingeTest, CloneTest) {
  // Create an arbitrary configuration so we can see it get copied properly.
  DoorHingeConfig config;
  config.spring_zero_angle_rad = -1.25;
  config.spring_constant = 100.;
  config.dynamic_friction_torque = 10.;
  config.static_friction_torque = 7.;
  config.viscous_friction = 2.;
  config.catch_width = 0.05;
  config.catch_torque = 20;
  config.motion_threshold = 0.125;

  BuildDoorHinge(config);
  MultibodyPlant<AutoDiffXd> plant_ad(plant());

  EXPECT_EQ(plant_ad.num_positions(), 1);
  EXPECT_EQ(plant_ad.num_velocities(), 1);
  EXPECT_EQ(plant_ad.num_actuated_dofs(), 0);

  // Should include gravity and the door hinge.
  EXPECT_EQ(plant_ad.num_force_elements(), 2);
  const DoorHinge<AutoDiffXd>& door_hinge_ad =
      plant_ad.GetForceElement<DoorHinge>(ForceElementIndex(1));

  EXPECT_EQ(door_hinge_ad.config().spring_zero_angle_rad,
            config.spring_zero_angle_rad);
  EXPECT_EQ(door_hinge_ad.config().spring_constant, config.spring_constant);
  EXPECT_EQ(door_hinge_ad.config().dynamic_friction_torque,
            config.dynamic_friction_torque);
  EXPECT_EQ(door_hinge_ad.config().static_friction_torque,
            config.static_friction_torque);
  EXPECT_EQ(door_hinge_ad.config().viscous_friction, config.viscous_friction);
  EXPECT_EQ(door_hinge_ad.config().catch_width, config.catch_width);
  EXPECT_EQ(door_hinge_ad.config().catch_torque, config.catch_torque);
  EXPECT_EQ(door_hinge_ad.config().motion_threshold, config.motion_threshold);
}

// Test with only the torsional spring torque, the corresponding energy
// and power are computed correctly at non-zero states.
TEST_F(DoorHingeTest, SpringTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.spring_constant = 10;
  const DoorHinge<double>& dut = BuildDoorHinge(config);

  // Springs make spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1.), 0);
  EXPECT_LT(dut.CalcHingeSpringTorque(1.), 0);  // Pulls toward zero.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.), 0);

  // Test potential energy at non-zero angle.
  SetHingeJointState(kAngle, kAngularRate);
  // Verify the potential energy is computed correctly.
  TestPotentialEnergyCalculation();

  // Test the powers are computed correctly: a) the conservative power equals to
  // the corresponding torque times velocity; b) non-conservative power should
  // be zero.
  EXPECT_EQ(CalcHingeConservativePower(),
            dut.CalcHingeSpringTorque(kAngle) * kAngularRate);
  EXPECT_EQ(CalcHingeNonConservativePower(), 0.0);
}

// Test with only the catch spring torque, the corresponding energy and power
// are computed correctly at different states.
TEST_F(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.catch_width = 2 * kAngle;
  config.catch_torque = 10.0;
  const DoorHinge<double>& dut = BuildDoorHinge(config);

  // The catch makes spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1.), 0);

  // Resists closing.
  EXPECT_GT(dut.CalcHingeSpringTorque(config.catch_width), 0);
  // Tipover point.
  EXPECT_EQ(dut.CalcHingeSpringTorque(config.catch_width / 2), 0);
  // Detent pulls in.
  EXPECT_LT(dut.CalcHingeSpringTorque(0.), 0);

  // Verify that the potential energy with only the catch spring torque at zero
  // position and the catch_width position should be 0.
  EXPECT_EQ(CalcHingePotentialEnergy(), 0.0);

  SetHingeJointState(config.catch_width, 0.0);
  EXPECT_EQ(CalcHingePotentialEnergy(), 0.0);

  // Test the energy from power integration.
  SetHingeJointState(kAngle, kAngularRate);
  // Verify the potential energy is computed correctly.
  TestPotentialEnergyCalculation();

  // Test the powers are computed correctly: a) the conservative power equals to
  // the corresponding torque times velocity; b) non-conservative power should
  // be zero.
  EXPECT_EQ(CalcHingeConservativePower(),
            dut.CalcHingeSpringTorque(kAngle) * kAngularRate);
  EXPECT_EQ(CalcHingeNonConservativePower(), 0.0);
}

// Test with only the static friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.static_friction_torque = 1;
  const DoorHinge<double>& dut = BuildDoorHinge(config);

  // Friction opposes tiny motion, but falls away with substantial motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01), 0, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1.), 0);

  // Test the energy and power with a given non-zero state: a) the
  // non-conservative power equals to the corresponding torque times velocity;
  // b) conservative power should be zero.
  SetHingeJointState(kAngle, kAngularRate);
  EXPECT_EQ(CalcHingeNonConservativePower(),
            dut.CalcHingeFrictionalTorque(kAngularRate) * kAngularRate);
  EXPECT_EQ(CalcHingeConservativePower(), 0.0);
}

// Test with only the dynamic friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.dynamic_friction_torque = 1;
  const DoorHinge<double>& dut = BuildDoorHinge(config);

  // Friction opposes any motion, even tiny motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01), -1, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1.), 0);

  // Test the energy and power with a given non-zero state: a) the
  // non-conservative power equals to the corresponding torque times velocity;
  // b) conservative power should be zero.
  SetHingeJointState(kAngle, kAngularRate);
  EXPECT_EQ(CalcHingeNonConservativePower(),
            dut.CalcHingeFrictionalTorque(kAngularRate) * kAngularRate);
  EXPECT_EQ(CalcHingeConservativePower(), 0.0);
}

// Test with only the viscous friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.viscous_friction = 1;
  const DoorHinge<double>& dut = BuildDoorHinge(config);

  // Friction opposes motion proprotionally.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1.), -1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-1.), 1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-2.), 2);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1.), 0);

  // Test the energy and power with a given non-zero state: a) the
  // non-conservative power equals to the corresponding torque times velocity;
  // b) conservative power should be zero.
  SetHingeJointState(kAngle, kAngularRate);
  EXPECT_EQ(CalcHingeNonConservativePower(),
            dut.CalcHingeFrictionalTorque(kAngularRate) * kAngularRate);
  EXPECT_EQ(CalcHingeConservativePower(), 0.0);
}

// Returns the total energy `E = KE + PE` of the plant.
double CalcSystemTotalEnergy(const systems::Simulator<double>& simulator,
                             const MultibodyPlant<double>& plant) {
  const auto& context = simulator.get_context();

  return plant.EvalKineticEnergy(context) + plant.EvalPotentialEnergy(context);
}

// Runs a simulator to integrate the system (i.e., `plant`) forward to time
// `kTotalSimTime` from the `init_state` and confirm that the energy and power
// computations are correct. To be more specific, this function confirms
// `E(t) − W(t) = E(0)` within the given tolerance, where `E = KE + PE` is the
// total energy of the system and `W(t)` is work done by non-conservative
// torques. It assumes that there is no external torques other than the torques
// introduced by the DoorHinge force element.
//
// N.B. for simplicity, instead of building a new system and defining a
// continuous state to compute the work `W(t)` done by conservative torques,
// this function uses `set_monitor()` function to explicitly integrate the
// non-conservative power to get `W(t)`, i.e. W(t) =∫Pnc dt. Therefore, there is
// no error-control or anything on `W(t)`. If we chose to use a continuous
// system, the error between energy loss (which computed from the
// error-controlled RK3 integrator for example) and the work `W(t)` can be big.
// If we chose to use a discrete system, this error will be more meaningful
// since both terms use the similar integration method. The test here chooses to
// use the discrete system for an apple-to-apple comparison. Therefore, the
// error tolerance of the comparison will purely depend on the maximum time
// step. After running multiple tests with different maximum time steps, the
// error is roughly about 100 times bigger than the time step. Note that, to
// guarantee the test is meaningful, two requirements have to be posted. First,
// the initial total energy should be bigger than the tolerance by several
// orders of magnitude. Here, we choose the number to be 1e3. Second, the change
// of angular rate before and after the simulation has to be more than 20% to
// make sure that the energy has a significant change or shift. There two
// numbers are set by heuristic. Since the main focus of this test is to verify
// the correctness of the energy calculation, the settings are good enough.
void TestEnergyConservation(const MultibodyPlant<double>& plant,
                            const Eigen::Vector2d& init_state) {
  systems::Simulator<double> simulator(plant);
  simulator.Initialize();

  double non_conserv_work = 0.0;
  double prev_time = 0.0;
  simulator.set_monitor([&plant, &prev_time, &non_conserv_work](
                            const systems::Context<double>& root_context) {
    // Compute delta time and update previous time.
    const double curr_time = root_context.get_time();
    const double delta_time = curr_time - prev_time;
    prev_time = curr_time;

    const double non_conserv_power =
        plant.EvalNonConservativePower(root_context);
    non_conserv_work += non_conserv_power * delta_time;

    return systems::EventStatus::Succeeded();
  });

  // Set initial condition of the simulation
  auto& init_plant_context = simulator.get_mutable_context();
  init_plant_context.get_mutable_discrete_state_vector().SetFromVector(
      init_state);

  const double init_total_energy = CalcSystemTotalEnergy(simulator, plant);
  simulator.AdvanceTo(kTotalSimTime);

  // Require the velocity state to change enough to make the comparison
  // numerically meaningful.
  const double kMinVelocityChangeRatio = 0.2;
  const auto& final_state = simulator.get_context().get_discrete_state_vector();
  const double velocity_change = std::abs(final_state[1] - init_state[1]);
  DRAKE_DEMAND(velocity_change >
               kMinVelocityChangeRatio * std::abs(init_state[1]));

  // Require the initial energy to be big enough to make the comparison
  // numerically meaningful.
  const double kEnergyToToleranceRatio = 1e3;
  const double tolerance = 1e2 * kMaximumTimeStep;
  DRAKE_DEMAND(init_total_energy > kEnergyToToleranceRatio * tolerance);

  const double final_total_energy = CalcSystemTotalEnergy(simulator, plant);
  const double energy_loss = final_total_energy - init_total_energy;

  EXPECT_NEAR(energy_loss, non_conserv_work, tolerance);
}

// Confirm no energy loss if there are no non-conservative torques.
TEST_F(DoorHingeTest, EnergyTestWithOnlyConservativeTorques) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.spring_constant = 6;
  config.spring_zero_angle_rad = 1.0;
  config.catch_width = 0.02;
  config.catch_torque = 1.0;
  BuildDoorHinge(config);

  // Set the initial velocity to a relative large value to make sure the energy
  // is numerically significant comparing to the tolerance.
  const Eigen::Vector2d init_state{0, 2.0};
  TestEnergyConservation(plant(), init_state);
}

// Confirm the energy loss should equal to the non-conservative energy, i.e.,
// dissipative energy. The loss should be greater than 0.0.
TEST_F(DoorHingeTest, EnergyTestWithAllTorques) {
  // Use the default door hinge configuration.
  const DoorHingeConfig config;
  BuildDoorHinge(config);

  // Set the initial velocity to a relative large value to make sure the energy
  // is numerically significant comparing to the tolerance.
  const Eigen::Vector2d init_state{0, 2.0};
  TestEnergyConservation(plant(), init_state);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
