#include"drake/systems/analysis/runge_kutta2_integrator.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
  namespace systems {
    namespace {

      GTEST_TEST(IntegratorTest, MiscAPI) {
          // create the spring-mass system
          MySpringMassSystem spring_mass(1., 1., 0.);

          // create the integrator
          RungeKutta2Integrator<double> integrator(spring_mass);

          // set the accuracy
          integrator.set_accuracy(1e-6);
          integrator.request_initial_step_size_target(1e-8);

          EXPECT_EQ(integrator.get_target_accuracy(), 1e-6);
          EXPECT_EQ(integrator.get_initial_step_size_target(), 1e-8);
      }

      GTEST_TEST(IntegratorTest, ContextAccess) {
          // create the mass spring system
          MySpringMassSystem spring_mass(1., 1., 0.);

          // create a context
          auto context = spring_mass.CreateDefaultContext();

          // create the integrator
          RungeKutta2Integrator<double> integrator(spring_mass, context.get());
          integrator.get_mutable_context()->set_time(3.);
          EXPECT_EQ(integrator.get_context().get_time(), 3.);
          EXPECT_EQ(context->get_time(), 3.);
      }

      // Try a purely continuous system with no sampling.
      // d^2x/dt^2 = -kx/m
      // solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
      // where omega = sqrt(k/m)
      // x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
      // for t = 0, x(0) = c1, x'(0) = c2*omega
      GTEST_TEST(IntegratorTest, SpringMassStep) {
          const double kSpring = 300.0;  // N/m
          const double kMass = 2.0;      // kg

          // create the spring-mass system
          MySpringMassSystem spring_mass(kSpring, kMass, 0.);

          // create a context
          auto context = spring_mass.CreateDefaultContext();

          // create the integrator
          RungeKutta2Integrator<double> integrator(spring_mass, context.get());

          // setup the initial position and initial velocity
          const double kInitialPosition = 0.1;
          const double kInitialVelocity = 0.0;
          const double kOmega = std::sqrt(kSpring/kMass);

          // Set initial condition using the Simulator's internal Context.
          spring_mass.set_position(integrator.get_mutable_context(),
                                   kInitialPosition);

          // Take all the defaults.
          integrator.Initialize();

          // setup c1 and c2 for ODE constants
          const double C1 = kInitialPosition;
          const double C2 = kInitialVelocity/kOmega;

          // Integrate for 1 second.
          const double DT = 0.00078125;
          const double T_FINAL = 1.0;
          double t;
          for (t = 0.0; std::fabs(t - T_FINAL) > DT; t += DT)
            integrator.Step(DT);

          EXPECT_NEAR(context->get_time(), 1., DT);  // Should be exact.

          // get the final position
          const double kXFinal = context->get_state().
              continuous_state->get_state().GetAtIndex(0);

          // check the solution
          double true_sol = C1*std::cos(kOmega*t) + C2*std::sin(kOmega*t);
          EXPECT_NEAR(true_sol, kXFinal, 1e-5);
      }

    }  // namespace
  }  // namespace systems
}  // namespace drake
