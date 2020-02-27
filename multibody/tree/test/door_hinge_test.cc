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

  double CalcHingeFrictionalTorque(double angular_rate) const {
    return door_hinge_.CalcHingeFrictionalTorque(angular_rate);
  }

  double CalcHingeSpringTorque(double angle) const {
    return door_hinge_.CalcHingeSpringTorque(angle);
  }

  double CalcHingeConservativePower(double angle, double angular_rate) const {
    return door_hinge_.CalcHingeConservativePower(angle, angular_rate);
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
  // DoorHinge force element. Then, it returns a DoorHingeTester for testing
  // purpose.
  const DoorHingeTester& BuildDoorHingeTester(const DoorHingeConfig& config) {
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

    door_hinge_tester_ = std::make_unique<DoorHingeTester>(*door_hinge_);
    return *door_hinge_tester_;
  }

  void SetHingeJointState(double angle, double angular_rate) {
    revolute_joint_->set_angle(plant_context_.get(), angle);
    revolute_joint_->set_angular_rate(plant_context_.get(), angular_rate);
  }

  double CalcPotentialEnergy() const {
    return door_hinge_->CalcPotentialEnergy(
        *plant_context_, plant_->EvalPositionKinematics(*plant_context_));
  }

  double CalcConservPower() const {
    return door_hinge_->CalcConservativePower(
        *plant_context_, plant_->EvalPositionKinematics(*plant_context_),
        plant_->EvalVelocityKinematics(*plant_context_));
  }

  double CalcNonConservPower() const {
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
    const double potential_energy = CalcPotentialEnergy();
    DRAKE_DEMAND(std::abs(potential_energy) >= 1.0);

    const auto& hinge_tester = door_hinge_tester();
    auto energy_ode = [&hinge_tester](
                          const double& t, const VectorX<double>& x,
                          const VectorX<double>& k) -> VectorX<double> {
      unused(t);
      unused(k);
      VectorX<double> ret(x.size());
      ret[0] = -hinge_tester.CalcHingeConservativePower(x[1], x[2]);
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

    const systems::InitialValueProblem<double>::SpecifiedValues default_values(
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

  const DoorHingeTester& door_hinge_tester() const {
    return *door_hinge_tester_;
  }

 private:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;

  const RevoluteJoint<double>* revolute_joint_{nullptr};
  const DoorHinge<double>* door_hinge_{nullptr};
  std::unique_ptr<DoorHingeTester> door_hinge_tester_;
};

DoorHingeConfig CreateZeroForcesDoorHingeConfig() {
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

// Verify the torques and the energy should be zero when the config parameters
// are all zero.
TEST_F(DoorHingeTest, ZeroTest) {
  const DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

  // If no frictions, springs, etc. are applied, our torques should be 0.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1.), 0);

  // Test energy should be zero at the default state.
  EXPECT_EQ(CalcPotentialEnergy(), 0.0);

  // Test energy should be zero at a non-zero state.
  SetHingeJointState(kAngle, kAngularRate);
  EXPECT_EQ(CalcPotentialEnergy(), 0.0);
}

// Test that scalar conversion produces properly constructed results.
TEST_F(DoorHingeTest, CloneTest) {
  const DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
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
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.spring_constant = 10;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

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
  EXPECT_EQ(CalcConservPower(),
            dut.CalcHingeSpringTorque(kAngle) * kAngularRate);
  EXPECT_EQ(CalcNonConservPower(), 0.0);
}

// Test with only the catch spring torque, the corresponding energy and power
// are computed correctly at different states.
TEST_F(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.catch_width = 2 * kAngle;
  config.catch_torque = 10.0;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

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
  EXPECT_EQ(CalcPotentialEnergy(), 0.0);

  SetHingeJointState(config.catch_width, 0.0);
  EXPECT_EQ(CalcPotentialEnergy(), 0.0);

  // Test the energy from power integration.
  SetHingeJointState(kAngle, kAngularRate);
  // Verify the potential energy is computed correctly.
  TestPotentialEnergyCalculation();

  // Test the powers are computed correctly: a) the conservative power equals to
  // the corresponding torque times velocity; b) non-conservative power should
  // be zero.
  EXPECT_EQ(CalcConservPower(),
            dut.CalcHingeSpringTorque(kAngle) * kAngularRate);
  EXPECT_EQ(CalcNonConservPower(), 0.0);
}

// Test with only the static friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.static_friction_torque = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

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
  EXPECT_EQ(CalcNonConservPower(),
            dut.CalcHingeFrictionalTorque(kAngularRate) * kAngularRate);
  EXPECT_EQ(CalcConservPower(), 0.0);
}

// Test with only the dynamic friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.dynamic_friction_torque = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

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
  EXPECT_EQ(CalcNonConservPower(),
            dut.CalcHingeFrictionalTorque(kAngularRate) * kAngularRate);
  EXPECT_EQ(CalcConservPower(), 0.0);
}

// Test with only the viscous friction torque, the torques, energy and power are
// computed correctly.
TEST_F(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = CreateZeroForcesDoorHingeConfig();
  config.viscous_friction = 1;
  const DoorHingeTester& dut = BuildDoorHingeTester(config);

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
  EXPECT_EQ(CalcNonConservPower(),
            dut.CalcHingeFrictionalTorque(kAngularRate) * kAngularRate);
  EXPECT_EQ(CalcConservPower(), 0.0);
}

// Returns the total energy `E = KE + PE` of the plant. It assumes there is only
// one non-world body and one joint.
// N.B. Since the MultibodyPlant does not provide methods to compute the
// kinetic energy, we have to provide a simple function to compute it.
// Once the plant has this method, this function could be removed.
// TODO(huihua.zhao) Fix this note once the PR #12895 lands.
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
// `E(t) − W(t) = E(0)` within the given tolerance, where `E = KE + PE` is the
// total energy of the system and `W(t)` is work done by non-conservative
// torques. It assumes that there is no external torques other than the torques
// introduced by the DoorHinge force element.
//
// N.B. Since the MultibodyPlant does not provide methods to compute the
// non-conservative power, the DoorHingeTester has to be passed in for this
// purpose.
// TODO(huihua.zhao) Fix this note once the PR #12895 lands.
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
void TestEnergyConservative(const MultibodyPlant<double>& plant,
                            const DoorHingeTester& dut,
                            const Eigen::Vector2d& init_state) {
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

  const DoorHingeTester dut = BuildDoorHingeTester(config);
  // Set the initial velocity to a relative large value to make sure the energy
  // is numerically significant comparing to the tolerance.
  const Eigen::Vector2d init_state{0, 2.0};
  TestEnergyConservative(plant(), dut, init_state);
}

// Confirm the energy loss should equal to the non-conservative energy, i.e.,
// dissipative energy. The loss should be greater than 0.0.
TEST_F(DoorHingeTest, EnergyTestWithAllTorques) {
  // Use the default door hinge configuration.
  const DoorHingeConfig config;
  const DoorHingeTester dut = BuildDoorHingeTester(config);
  // Set the initial velocity to a relative large value to make sure the energy
  // is numerically significant comparing to the tolerance.
  const Eigen::Vector2d init_state{0, 2.0};
  TestEnergyConservative(plant(), dut, init_state);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
