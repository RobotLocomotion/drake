#include "drake/multibody/tree/door_hinge.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/initial_value_problem.h"
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

  // This function will be used to confirm that the scalar conversion methods
  // work properly.
  const internal::MultibodyTree<double>& parent_tree() const {
    return door_hinge_.get_parent_tree();
  }

  const DoorHinge<double>& door_hinge() const { return door_hinge_; }

 private:
  const DoorHinge<double>& door_hinge_;
};

namespace {

constexpr double kMass = 1.0;
constexpr double kPrincipalInertia = 1.0;
constexpr double kAngle = 0.01;
constexpr double kAngularRate = 1.0;
constexpr double kIntegrationTimeStep = 1e-6;
constexpr double kTotalSimTime = 0.05;
// Error tolerance for the explicit non-Drake Euler integrator.
constexpr double kErrorTolerance = 1e-4;
const std::string kRevoluteJointName = "RevoluteJoint";

class DoorHingeTest : public ::testing::Test {
 protected:
  // Based on the DoorHingeConfig, this function sets up a plant that includes a
  // DoorHinge force element. Then, it returns a DoorHingeTester for testing
  // purpose.
  const DoorHingeTester& BuildDoorHingeTester(const DoorHingeConfig& config) {
    plant_ = std::make_unique<MultibodyPlant<double>>(kIntegrationTimeStep);

    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        kMass, drake::Vector3<double>::Zero(),
        kMass * UnitInertia<double>::TriaxiallySymmetric(kPrincipalInertia));
    auto& body1 = plant_->AddRigidBody("body1", M_B);

    revolute_joint_ = &plant_->AddJoint<RevoluteJoint>(
        kRevoluteJointName, plant_->world_body(), {}, body1, {},
        Eigen::Vector3d::UnitZ(), 0.0 /*set joint damping to zero*/);

    door_hinge_ = &plant_->AddForceElement<DoorHinge>(*revolute_joint_, config);

    // Set gravity to zero so that one less energy term to consider when verify
    // the energy correctness of the DoorHinge force element.
    plant_->mutable_gravity_field().set_gravity_vector(Vector3<double>::Zero());

    // Finish building the model and create the context.
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();

    door_hinge_tester_ = std::make_unique<DoorHingeTester>(*door_hinge_);

    return *door_hinge_tester_;
  }

  void SetHingeJointState(double angle, double angular_rate) {
    revolute_joint_->set_angle(plant_context_.get(), angle);
    revolute_joint_->set_angular_rate(plant_context_.get(), angular_rate);
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }
  const systems::Context<double>& plant_context() const {
    return *plant_context_;
  }

 private:
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
// b) non-conservative power equals to the corresponding torque times velocity.
void TestFrictionOnlyEnergyAndPower(const DoorHingeTester& dut,
                                    const systems::Context<double>& context,
                                    const MultibodyPlant<double>& plant) {
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      context, plant.EvalPositionKinematics(context));
  EXPECT_EQ(potential_energy, 0.0);

  const double conserv_power = dut.door_hinge().CalcConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));
  EXPECT_EQ(conserv_power, 0.0);

  // Verify the non-conservative power
  const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));

  const double angular_rate =
      plant.GetJointByName(kRevoluteJointName).GetOneVelocity(context);
  EXPECT_EQ(non_conserv_power, dut.CalcHingeFrictionalTorque(
                                   angular_rate, dut.door_hinge().config()) *
                                   angular_rate);
}

// With the condition that there is only spring related torque, this function
// confirms that a) the conservative power equals to the corresponding torque
// times velocity; b) non-conservative power should be zero.
void TestSpringTorqueOnlyPower(const DoorHingeTester& dut,
                               const systems::Context<double>& context,
                               const MultibodyPlant<double>& plant) {
  auto spring_power = [&dut](double q, double v) {
    return dut.CalcHingeSpringTorque(q, dut.door_hinge().config()) * v;
  };

  const double conserv_power = dut.door_hinge().CalcConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));

  const double angle =
      plant.GetJointByName(kRevoluteJointName).GetOnePosition(context);
  const double angular_rate =
      plant.GetJointByName(kRevoluteJointName).GetOneVelocity(context);
  EXPECT_EQ(conserv_power, spring_power(angle, angular_rate));

  const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
      context, plant.EvalPositionKinematics(context),
      plant.EvalVelocityKinematics(context));
  EXPECT_EQ(non_conserv_power, 0.0);
}

// This function confirms the potential energy (PE) is computed correctly by
// comparing it against the result from integrating the conservative power (P),
// i.e. we should have PE = -∫Pdt. The `InitialValueProblem` class is used for
// this purpose. The state is defined as `x = {PE, q, v}`, where `q` is the
// angle and `v` is the angular rate. In particular, this function assumes the
// hinge joint moves from zero (initial state) to the current angle qₜ with a
// constant speed (current angular rate vₜ). It also assumes that the initial
// potential energy is 0 and initial angle is 0. Correspondingly, the ODEs can
// be defined as `ẋ[0] = P`, `ẋ[1] = v`, `ẋ[2] = 0` and the initial value is
// `x₀ = {0, 0, vₜ}`. Since velocity is constant, the total simulation time
// can be derived as t = qₜ / vₜ.
void TestPotentialEnergyCalculation(const DoorHingeTester& dut,
                                    const systems::Context<double>& context,
                                    const MultibodyPlant<double>& plant) {
  auto energy_ode = [&dut](const double& t, const VectorX<double> x,
                           const VectorX<double>& k) -> VectorX<double> {
    unused(t);
    unused(k);
    VectorX<double> ret(x.size());
    ret[0] = -dut.CalcHingeSpringTorque(x[1], dut.door_hinge().config()) * x[2];
    ret[1] = x[2];
    ret[2] = 0.0;
    return ret;
  };

  const double target_angle =
      plant.GetJointByName(kRevoluteJointName).GetOnePosition(context);
  const double angular_rate =
      plant.GetJointByName(kRevoluteJointName).GetOneVelocity(context);

  const double kInitialTime = 0.0;
  const VectorX<double> kInitialState =
      (VectorX<double>(3) << 0.0, 0.0, angular_rate).finished();
  const VectorX<double> kDefaultParameters = VectorX<double>::Zero(3);

  const systems::InitialValueProblem<double>::SpecifiedValues kDefaultValues(
      kInitialTime, kInitialState, kDefaultParameters);
  const systems::InitialValueProblem<double> ivp(energy_ode, kDefaultValues);

  DRAKE_THROW_UNLESS(angular_rate != 0);
  const double kTotalTime = std::abs(target_angle / angular_rate);
  const auto result = ivp.Solve(kTotalTime);
  // Confirm the velocity is not changed.
  DRAKE_THROW_UNLESS(result[2] == angular_rate);

  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      context, plant.EvalPositionKinematics(context));

  EXPECT_NEAR(potential_energy, result[0], 1e-3);
}

// Verify the torques and the energy should be zero when the config parameters
// are all zero.
TEST_F(DoorHingeTest, ZeroTest) {
  const DoorHingeConfig config = no_forces_config();
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  // If no frictions, springs, etc. are applied, our torques should be 0.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test energy should be zero at a default state.
  const double potential_energy_1 = dut.door_hinge().CalcPotentialEnergy(
      plant_context(), plant().EvalPositionKinematics(plant_context()));
  EXPECT_EQ(potential_energy_1, 0.0);
  // Test energy should be zero at a non-zero state.
  SetHingeJointState(kAngle, kAngularRate);
  const double potential_energy_2 = dut.door_hinge().CalcPotentialEnergy(
      plant_context(), plant().EvalPositionKinematics(plant_context()));
  EXPECT_EQ(potential_energy_2, 0.0);
}

// Test that scalar conversion produces properly constructed results.
TEST_F(DoorHingeTest, CloneTest) {
  const DoorHingeConfig config = no_forces_config();
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  std::unique_ptr<internal::MultibodyTree<AutoDiffXd>> tree_ad =
      dut.parent_tree().CloneToScalar<AutoDiffXd>();
  EXPECT_EQ(tree_ad->num_positions(), 1);
  EXPECT_EQ(tree_ad->num_velocities(), 1);
  EXPECT_EQ(tree_ad->num_actuated_dofs(), 0);
}

// Test with only the torsional spring torque, the corresponding energy
// and power are computed correctly at non-zero states.
TEST_F(DoorHingeTest, SpringTest) {
  DoorHingeConfig config = no_forces_config();
  config.spring_constant = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  // Springs make spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_LT(dut.CalcHingeSpringTorque(1., config), 0);  // Pulls toward zero.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);

  // Test potential energy at non-zero angle.
  SetHingeJointState(kAngle, kAngularRate);
  // Verify the potential energy are computed correctly.
  TestPotentialEnergyCalculation(dut, plant_context(), plant());
  // Test the powers are computed correctly.
  TestSpringTorqueOnlyPower(dut, plant_context(), plant());
}

// Test with only the catch spring torque, the corresponding energy and power
// are computed correctly at different states.
TEST_F(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = no_forces_config();
  config.catch_width = 2 * kAngle;
  config.catch_torque = 1.0;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

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
      plant_context(), plant().EvalPositionKinematics(plant_context()));
  EXPECT_EQ(potential_energy_q0, 0.0);

  SetHingeJointState(config.catch_width, 0.0);
  const double potential_energy_qc = dut.door_hinge().CalcPotentialEnergy(
      plant_context(), plant().EvalPositionKinematics(plant_context()));
  EXPECT_EQ(potential_energy_qc, 0.0);

  // Test the energy from power integration.
  SetHingeJointState(kAngle, kAngularRate);
  // Verify the potential energy are computed correctly.
  TestPotentialEnergyCalculation(dut, plant_context(), plant());
  // Verify the power terms are computed correctly.
  TestSpringTorqueOnlyPower(dut, plant_context(), plant());
}

// Test with only the static friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.static_friction_torque = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  // Friction opposes tiny motion, but falls away with substantial motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), 0, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given non-zero state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, plant_context(), plant());
}

// Test with only the dynamic friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.dynamic_friction_torque = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  // Friction opposes any motion, even tiny motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), -1, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given non-zero state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, plant_context(), plant());
}

// Test with only the viscous friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.viscous_friction = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  // Friction opposes motion proprotionally.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), -1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-1., config), 1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-2., config), 2);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given non-zero state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, plant_context(), plant());
}

// Returns the total energy `E = KE + PE` of the plant. It assumes there is only
// one non-world body and one joint.
double CalcSystemTotalEnergy(const systems::Simulator<double>& simulator,
                             const MultibodyPlant<double>& plant) {
  const auto& context = simulator.get_context();
  const auto& state = context.get_discrete_state_vector();
  DRAKE_THROW_UNLESS(state.size() == 2);
  // Calculate the kinetic energy. If m is inertia, v should be angular rate.
  // TODO(huihua.zhao) Use the MBP::CalcKineticEnergy() once it is available
  // (issue #12886 and PR #12895).
  auto calc_kinetic_energy = [](double m, double v) { return 0.5 * m * v * v; };

  const double kinetic_energy =
      calc_kinetic_energy(kPrincipalInertia, state[1]);
  const double potential_energy = plant.CalcPotentialEnergy(context);
  return kinetic_energy + potential_energy;
}

// Runs a simulator to integrate the system (i.e., `plant`) forward to time
// `kTotalSimTime` from the `init_state` and confirm that the energy and power
// computations are correct. To be more specific, this function confirms
//`E(t) − W(t) = E(0)` within the given tolerance, where `E = KE + PE` is the
// total energy of the system and `W(t)` is work done by non-conservative
// torques. It assumes that there is no external torques other than the torques
// introduced by the DoorHinge force element. It returns the total energy loss
// (E(0) − E(t)), which should be non-negative.
//
// N.B. Since the MultibodyPlant does not provide methods to compute the
// non-conservative power, the DoorHingeTester has to be passed in for this
// purpose.
// TODO(huihua.zhao) Fix this note once the PR #12895 lands.
//
// N.B. for simplicity, instead of building a new system and defining a
// continuous state to compute the non-conservative work `W(t)`, this function
// uses `set_monitor()` function to explicitly integrate the non-conservative
// power to get `W(t)`, i.e. W(t) =∫Pnc dt. Therefore, there is no error-control
// or anything. If the user chose to use a continuous system, the error between
// energy loss (which computed from the error-controlled RK3 integrator for
// example) and the non-conservative work `W(t)` can be big. If chose to use a
// discrete system, this error will be much smaller (about the same magnitude of
// the integration step) since both terms use the similar integration method.
double TestEnergyConservative(const MultibodyPlant<double>& plant,
                              const DoorHingeTester& dut,
                              const Eigen::Vector2d& init_state,
                              double tolerance) {
  systems::Simulator<double> simulator(plant);
  simulator.Initialize();

  double non_conserv_work = 0.0;
  double prev_time = 0.0;
  simulator.set_monitor([&plant, &dut, &prev_time, &non_conserv_work](
                            const systems::Context<double>& root_context) {
    // Compute delta time and update previous time.
    const double curr_time = root_context.get_time();
    const double delta_time = curr_time - prev_time;
    prev_time = curr_time;

    const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
        root_context, plant.EvalPositionKinematics(root_context),
        plant.EvalVelocityKinematics(root_context));
    non_conserv_work += non_conserv_power * delta_time;

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

  const double init_total_energy = CalcSystemTotalEnergy(simulator, plant);

  simulator.AdvanceTo(kTotalSimTime);

  const double final_total_energy = CalcSystemTotalEnergy(simulator, plant);

  const double energy_loss = init_total_energy - final_total_energy;
  EXPECT_NEAR(energy_loss, -non_conserv_work, tolerance);

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
  const Eigen::Vector2d init_state{0, kAngularRate};

  const double kTolerance = 10 * kIntegrationTimeStep;
  const double energy_loss =
      TestEnergyConservative(plant(), dut, init_state, kTolerance);
  EXPECT_NEAR(energy_loss, 0.0, kTolerance);
}

// Confirm the energy loss should equal to the non-conservative energy, i.e.,
// dissipative energy. The loss should be greater than 0.0.
TEST_F(DoorHingeTest, EnergyTestWithAllTorques) {
  // Use the default door hinge configuration.
  const DoorHingeConfig config;
  const DoorHingeTester dut = BuildDoorHingeTester(config);
  const Eigen::Vector2d init_state{0, kAngularRate};

  const double kTolerance = 10 * kIntegrationTimeStep;
  const double energy_loss =
      TestEnergyConservative(plant(), dut, init_state, kTolerance);
  EXPECT_GT(energy_loss, 0.0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
