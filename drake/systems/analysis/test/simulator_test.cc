#include "drake/systems/analysis/simulator.h"

#include <cmath>
#include <complex>
#include <functional>
#include <map>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/test/controlled_spring_mass_system/controlled_spring_mass_system.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

using Eigen::AutoDiffScalar;
using Eigen::NumTraits;
using std::complex;

namespace drake {
namespace systems {
namespace {

GTEST_TEST(SimulatorTest, SecondConstructor) {
  // Create the spring-mass sytem and context.
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  auto context = spring_mass.CreateDefaultContext();

  // Mark the context with an arbitrary value.
  context->set_time(3.);

  /// Construct the simulator with the created context.
  Simulator<double> simulator(spring_mass, std::move(context));

  // Verify that context pointers are equivalent.
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
}

GTEST_TEST(SimulatorTest, MiscAPI) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Default realtime rate should be zero.
  EXPECT_TRUE(simulator.get_target_realtime_rate() == 0.);

  simulator.set_target_realtime_rate(1.25);
  EXPECT_TRUE(simulator.get_target_realtime_rate() == 1.25);

  EXPECT_TRUE(std::isnan(simulator.get_actual_realtime_rate()));

  // Set the integrator default step size.
  const double dt = 1e-3;

  // Create a context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  // Initialize the simulator first.
  simulator.Initialize();
}

GTEST_TEST(SimulatorTest, ContextAccess) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set the integrator default step size.
  const double dt = 1e-3;

  // Get the context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  // Initialize the simulator first.
  simulator.Initialize();

  // Try some other context stuff.
  simulator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  simulator.release_context();
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
  EXPECT_THROW(simulator.Initialize(), std::logic_error);

  // Create another context.
  auto ucontext = spring_mass.CreateDefaultContext();
  ucontext->set_time(3.);
  simulator.reset_context(std::move(ucontext));
  EXPECT_EQ(simulator.get_context().get_time(), 3.);
  simulator.reset_context(nullptr);
  EXPECT_TRUE(simulator.get_mutable_context() == nullptr);
}

// Try a purely continuous system with no sampling.
GTEST_TEST(SimulatorTest, SpringMassNoSample) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Set the integrator default step size.
  const double dt = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Get the context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  simulator.set_target_realtime_rate(0.5);
  // Set the integrator and initialize the simulator.
  simulator.Initialize();

  // Simulate for 1 second.
  simulator.StepTo(1.);

  EXPECT_NEAR(context->get_time(), 1., 1e-8);
  EXPECT_EQ(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_updates(), 0);

  EXPECT_EQ(spring_mass.get_publish_count(), 1001);
  EXPECT_EQ(spring_mass.get_update_count(), 0);

  // Current time is 1. An earlier final time should fail.
  EXPECT_THROW(simulator.StepTo(0.5), std::runtime_error);
}

// Test ability to swap integrators mid-stream.
GTEST_TEST(SimulatorTest, ResetIntegratorTest) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // set the integrator default step size
  const double dt = 1e-3;

  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Set initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Get the context.
  auto context = simulator.get_mutable_context();

  // Create the integrator.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(spring_mass, dt,
                                                              context);

  // set the integrator and initialize the simulator
  simulator.Initialize();

  // Simulate for 1/2 second.
  simulator.StepTo(0.5);

  // Reset the integrator.
  simulator.reset_integrator<RungeKutta2Integrator<double>>(
      simulator.get_system(), dt, simulator.get_mutable_context());

  // Simulate to 1 second..
  simulator.StepTo(1.);

  EXPECT_NEAR(context->get_time(), 1., 1e-8);

  // Number of steps will have been reset.
  EXPECT_EQ(simulator.get_num_steps_taken(), 500);
}

// Because of arbitrary possible delays we can't do a very careful test of
// the realtime rate control. However, we can at least say that the simulation
// should not proceed much *faster* than the rate we select.
GTEST_TEST(SimulatorTest, RealtimeRate) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  simulator.set_target_realtime_rate(1.);  // No faster than 1X real time.
  simulator.get_mutable_context()->set_time(0.);
  simulator.Initialize();
  simulator.StepTo(1.);  // Simulate for 1 simulated second.
  EXPECT_TRUE(simulator.get_actual_realtime_rate() <= 1.1);

  simulator.set_target_realtime_rate(5.);  // No faster than 5X real time.
  simulator.get_mutable_context()->set_time(0.);
  simulator.Initialize();
  simulator.StepTo(1.);  // Simulate for 1 more simulated second.
  EXPECT_TRUE(simulator.get_actual_realtime_rate() <= 5.1);
}

// Tests that if publishing every timestep is disabled, publish only happens
// on initialization.
GTEST_TEST(SimulatorTest, DisablePublishEveryTimestep) {
  analysis_test::MySpringMassSystem<double> spring_mass(1., 1., 0.);
  Simulator<double> simulator(spring_mass);  // Use default Context.
  simulator.set_publish_every_time_step(false);

  simulator.get_mutable_context()->set_time(0.);
  simulator.Initialize();
  // Publish should happen on initialization.
  EXPECT_EQ(1, simulator.get_num_publishes());

  // Simulate for 1 simulated second.  Publish should not happen.
  simulator.StepTo(1.);
  EXPECT_EQ(1, simulator.get_num_publishes());
}

// Repeat the previous test but now the continuous steps are interrupted
// by a discrete sample every 1/30 second. The step size doesn't divide that
// evenly so we should get some step size modification here.
GTEST_TEST(SimulatorTest, SpringMass) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Set the integrator default step size.
  const double dt = 1e-3;

  // Create the mass spring system and the simulator.
  analysis_test::MySpringMassSystem<double> spring_mass(kSpring, kMass, 30.);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Get the context.
  auto context = simulator.get_mutable_context();

  // TODO(edrumwri): Remove this when discrete state has been created
  // automatically.
  // Create the discrete state.
  context->set_discrete_state(std::make_unique<DiscreteState<double>>());

  // Set initial condition using the Simulator's internal context.
  spring_mass.set_position(simulator.get_mutable_context(), 0.1);

  // Create the integrator and initialize it.
  auto integrator = simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      spring_mass, dt, context);
  integrator->Initialize();

  // Set the integrator and initialize the simulator.
  simulator.Initialize();

  // Simulate to one second.
  simulator.StepTo(1.);

  EXPECT_GT(simulator.get_num_steps_taken(), 1000);
  EXPECT_EQ(simulator.get_num_discrete_updates(), 30);

  // We're calling Publish() every step, and extra steps have to be taken
  // since the step size doesn't divide evenly into the sample rate. Shouldn't
  // require more than one extra step per sample though.
  EXPECT_LE(spring_mass.get_publish_count(), 1030);
  EXPECT_EQ(spring_mass.get_update_count(), 30);
}

// A mock System that requests a single publication at a prespecified time.
namespace {
class UnrestrictedUpdater : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrestrictedUpdater)

  explicit UnrestrictedUpdater(double t_upd) : t_upd_(t_upd) {
  }

  ~UnrestrictedUpdater() override {}

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoCalcNextUpdateTime(const systems::Context<double>& context,
                            systems::UpdateActions<double>* actions)
                              const override {
    const double inf = std::numeric_limits<double>::infinity();
    actions->time = (context.get_time() < t_upd_) ? t_upd_ : inf;
    actions->events.push_back(systems::DiscreteEvent<double>());
    actions->events.back().action = systems::DiscreteEvent<double>::
                                               kUnrestrictedUpdateAction;
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const override {
    if (unrestricted_update_callback_ != nullptr)
      unrestricted_update_callback_(context, state);
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    if (derivatives_callback_ != nullptr) derivatives_callback_(context);
  }

  void set_unrestricted_update_callback(
      std::function<void(const Context<double>&, State<double>*)> callback) {
    unrestricted_update_callback_ = callback;
  }

  void set_derivatives_callback(
      std::function<void(const Context<double>&)> callback) {
    derivatives_callback_ = callback;
  }

 private:
  const double t_upd_{0.0};
  std::function<void(const Context<double>&, State<double>*)>
                                      unrestricted_update_callback_{nullptr};
  std::function<void(const Context<double>&)> derivatives_callback_{nullptr};
};
}  // namespace

// Tests that the simulator captures an unrestricted update at the exact time
// (i.e., without accumulating floating point error).
GTEST_TEST(SimulatorTest, ExactUpdateTime) {
  // Create the UnrestrictedUpdater system.
  const double t_upd = 1e-10;                // Inexact floating point rep.
  UnrestrictedUpdater unrest_upd(t_upd);
  Simulator<double> simulator(unrest_upd);  // Use default Context.

  // Set time to an exact floating point representation; we want t_upd to
  // be much smaller in magnitude than the time, hence the negative time.
  simulator.get_mutable_context()->set_time(-1.0/1024);

  // Capture the time at which an update is done using a callback function.
  std::vector<double> updates;
  unrest_upd.set_unrestricted_update_callback(
      [&updates](const Context<double>& context, State<double>* state) {
    updates.push_back(context.get_time());
  });

  // Simulate forward.
  simulator.Initialize();
  simulator.StepTo(1.);

  // Check that the update occurs at exactly the desired time.
  EXPECT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates.front(), t_upd);
}

// Tests Simulator for a Diagram system consisting of a tree of systems.
// In this case the System is a PidControlledSpringMassSystem which is a
// Diagram containing a SpringMassSystem (the plant) and a PidController which
// in turn is a Diagram composed of primitives such as Gain and Adder systems.
GTEST_TEST(SimulatorTest, ControlledSpringMass) {
  typedef complex<double> complexd;
  typedef AutoDiffScalar<Vector1d> SingleVarAutoDiff;

  // SpringMassSystem parameters.
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg
  // System's open loop frequency.
  const double wol = std::sqrt(kSpring / kMass);

  // Initial conditions.
  const double x0 = 0.1;
  const double v0 = 0.0;

  // Choose some controller constants.
  const double kp = kSpring;
  const double ki = 0.0;
  // System's undamped frequency (when kd = 0).
  const double w0 = std::sqrt(wol * wol + kp / kMass);
  // Damping ratio (underdamped).
  const double zeta = 0.5;
  const double kd = 2.0 * kMass * w0 * zeta;
  const double x_target = 0.0;

  PidControlledSpringMassSystem<double> spring_mass(kSpring, kMass, kp, ki, kd,
                                                    x_target);
  Simulator<double> simulator(spring_mass);  // Use default Context.

  // Sets initial condition using the Simulator's internal Context.
  spring_mass.set_position(simulator.get_mutable_context(), x0);
  spring_mass.set_velocity(simulator.get_mutable_context(), v0);

  // Takes all the defaults for the simulator.
  simulator.Initialize();

  // Computes analytical solution.
  // 1) Roots of the characteristic equation.
  complexd lambda1 = -zeta * w0 + w0 * std::sqrt(complexd(zeta * zeta - 1));
  complexd lambda2 = -zeta * w0 - w0 * std::sqrt(complexd(zeta * zeta - 1));

// Roots should be the complex conjugate of each other.
#ifdef __APPLE__
  // The factor of 20 is needed for OS X builds where the comparison needs a
  // looser tolerance, see #3636.
  auto abs_error = 20.0 * NumTraits<double>::epsilon();
#else
  auto abs_error = NumTraits<double>::epsilon();
#endif
  EXPECT_NEAR(lambda1.real(), lambda2.real(), abs_error);
  EXPECT_NEAR(lambda1.imag(), -lambda2.imag(), abs_error);

  // The damped frequency corresponds to the absolute value of the imaginary
  // part of any of the roots.
  double wd = std::abs(lambda1.imag());

  // 2) Differential equation's constants of integration.
  double C1 = x0;
  double C2 = (zeta * w0 * x0 + v0) / wd;

  // 3) Computes analytical solution at time final_time.
  // Velocity is computed using AutoDiffScalar.
  double final_time = 0.2;
  double x_final{}, v_final{};
  {
    // At the end of this local scope x_final and v_final are properly
    // initialized.
    // Auxiliary AutoDiffScalar variables are confined to this local scope so
    // that we don't pollute the test's scope with them.
    SingleVarAutoDiff time(final_time);
    time.derivatives() << 1.0;
    auto x =
        exp(-zeta * w0 * time) * (C1 * cos(wd * time) + C2 * sin(wd * time));
    x_final = x.value();
    v_final = x.derivatives()[0];
  }

  // Simulates to final_time.
  simulator.StepTo(final_time);

  EXPECT_EQ(simulator.get_num_steps_taken(), 200);

  const auto& context = simulator.get_context();
  EXPECT_NEAR(context.get_time(), final_time, 1e-8);

  // Compares with analytical solution (to numerical integration error).
  EXPECT_NEAR(spring_mass.get_position(context), x_final, 3.0e-6);
  EXPECT_NEAR(spring_mass.get_velocity(context), v_final, 1.0e-5);
}


// A mock System that requests discrete update at 1 kHz, and publishes at 400
// Hz. Calls user-configured callbacks on DoPublish,
// DoCalcDiscreteVariableUpdates, and EvalTimeDerivatives.
class DiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteSystem)

  DiscreteSystem() {
    // Deliberately choose a period that is identical to, and therefore courts
    // floating-point error with, the default max step size.
    const double offset = 0.0;
    this->DeclarePeriodicDiscreteUpdate(kUpdatePeriod, offset);
    this->DeclarePublishPeriodSec(kPublishPeriod);

    set_name("TestSystem");
  }

  ~DiscreteSystem() override {}

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteState<double>* updates) const override {
    if (update_callback_ != nullptr) update_callback_(context);
  }

  void DoPublish(
      const drake::systems::Context<double>& context) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    if (derivatives_callback_ != nullptr) derivatives_callback_(context);
  }

  void set_update_callback(
      std::function<void(const Context<double>&)> callback) {
    update_callback_ = callback;
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

  void set_derivatives_callback(
      std::function<void(const Context<double>&)> callback) {
    derivatives_callback_ = callback;
  }

  double update_period() const { return kUpdatePeriod; }
  double publish_period() const { return kPublishPeriod; }

 private:
  const double kUpdatePeriod{0.001};
  const double kPublishPeriod{0.0025};
  std::function<void(const Context<double>&)> update_callback_{nullptr};
  std::function<void(const Context<double>&)> publish_callback_{nullptr};
  std::function<void(const Context<double>&)> derivatives_callback_{nullptr};
};

// Returns true if the time in the @p context is a multiple of the @p period.
bool CheckSampleTime(const Context<double>& context, double period) {
  const double k = context.get_time() / period;
  const double int_k = std::round(k);
  const double kTolerance = 1e-8;
  return std::abs(k - int_k) < kTolerance;
}

// Tests that the Simulator invokes the DiscreteSystem's update method every
// 0.001 sec, and its publish method every 0.0025 sec, without missing any
// updates.
GTEST_TEST(SimulatorTest, DiscreteUpdateAndPublish) {
  DiscreteSystem system;
  int num_disc_updates = 0;
  system.set_update_callback([&](const Context<double>& context){
    ASSERT_TRUE(CheckSampleTime(context, system.update_period()));
    num_disc_updates++;
  });
  int num_publishes = 0;
  system.set_publish_callback([&](const Context<double>& context){
    ASSERT_TRUE(CheckSampleTime(context, system.publish_period()));
    num_publishes++;
  });

  drake::systems::Simulator<double> simulator(system);
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(0.5);
  EXPECT_EQ(500, num_disc_updates);
  // Publication occurs at 400Hz, and also at initialization.
  EXPECT_EQ(200 + 1, num_publishes);
}

// Tests that the order of events in a simulator time step is first update
// discrete state, then publish, then integrate.
GTEST_TEST(SimulatorTest, UpdateThenPublishThenIntegrate) {
  DiscreteSystem system;
  drake::systems::Simulator<double> simulator(system);
  enum EventType {
    kUpdate = 0,
    kPublish = 1,
    kIntegrate = 2
  };

  // Write down the order in which the DiscreteSystem is asked to compute
  // discrete updates, do publishes, or compute derivatives at each time step.
  std::map<int, std::vector<EventType>> events;
  system.set_update_callback(
      [&events, &simulator](const Context<double>& context) {
    events[simulator.get_num_steps_taken()].push_back(kUpdate);
  });
  system.set_publish_callback(
      [&events, &simulator](const Context<double>& context) {
    events[simulator.get_num_steps_taken()].push_back(kPublish);
  });
  system.set_derivatives_callback(
      [&events, &simulator](const Context<double>& context) {
    events[simulator.get_num_steps_taken()].push_back(kIntegrate);
  });

  // Run a simulation.
  simulator.set_publish_every_time_step(true);
  simulator.StepTo(0.5);

  // Check that all the update events precede all the publish events, and all
  // the publish events precede all the eval-derivatives events, for each
  // time step in the simulation.
  for (const auto& log : events) {
    ASSERT_GE(log.second.size(), 0u);
    EventType state = log.second[0];
    for (const EventType& event : log.second) {
      ASSERT_TRUE(event >= state);
      state = event;
    }
  }
}

// A basic sanity check that AutoDiff works.
GTEST_TEST(SimulatorTest, AutodiffBasic) {
  SpringMassSystem<AutoDiffXd> spring_mass(1., 1., 0.);
  Simulator<AutoDiffXd> simulator(spring_mass);
  simulator.Initialize();
  simulator.StepTo(1);
}

}  // namespace
}  // namespace systems
}  // namespace drake
