#include <complex>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {

using systems::Context;
using systems::Simulator;

namespace multibody {
namespace {

// Fixture to verify a MultibodyPlant model for a spring-mass system with
// damping against its analytical solution.
class SpringMassSystemTest : public ::testing::Test {
 public:
  // Helper to create a MultibodyPlant model for a spring-mass system with
  // damping. The parameters of the model are:
  // - The mass of the body, in kilograms. Strictly positive.
  // - The period of the undamped system, in seconds. Strictly positive.
  // - The damping ratio, dimensionless. Positive, it can be zero.
  // This simple system follows a harmonic oscillator equation of the form:
  //   ẍ + 2ζω₀ẋ + ω₀²x = 0
  // where the undamped angular frequency of oscillation ω₀ and the damping
  // ratio are determined in terms of the body mass m, spring stiffness k and
  // damping coefficient c as:
  //   ω₀ = sqrt(k/m), rad/s.
  //   ζ = c/sqrt(m⋅k)/2
  void MakeSpringMassSystem(double mass, double period, double damping_ratio) {
    const double undamped_angular_frequency = 2.0 * M_PI / period;
    const double stiffness =
        mass * undamped_angular_frequency * undamped_angular_frequency;
    const double damping = 2.0 * damping_ratio * std::sqrt(mass * stiffness);

    // Since for this test the box only has a single translational degree of
    // freedom, the value of the rotational inertia is not important. Thus
    // we use an arbitrary rotational inertia value.
    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        mass, Vector3<double>::Zero(),
        mass * UnitInertia<double>::TriaxiallySymmetric(1.0));
    const RigidBody<double>& body = plant_.AddRigidBody("mass", M_B);
    slider_ = &plant_.AddJoint<PrismaticJoint>("Slider", plant_.world_body(),
                                               std::nullopt, body, std::nullopt,
                                               Vector3<double>::UnitX());
    plant_.AddForceElement<LinearSpringDamper>(
        plant_.world_body(), Vector3<double>::Zero(),
        body, Vector3<double>::Zero(),
        free_length_, stiffness, damping);
    plant_.Finalize();
  }

  // Returns the position x(t) of the body mass in the spring-mass system for
  // time `t = time` given the parameters of the system (period and
  // damping_ratio) and the initial position x0 and initial velocity v0 at
  // `t = 0`.
  // This method cannot compute the solution for damping_ratio = 1, i.e. the
  // critically damped system.
  double CalcAnalyticSolution(
      double period, double damping_ratio,
      double x0, double v0, double time) {
    DRAKE_DEMAND(damping_ratio != 1.0);
    using std::abs;
    using std::complex;
    using std::exp;
    using std::sqrt;
    constexpr complex<double> imaginary_unit(0.0, 1.0);
    using Vector2c = Vector2<complex<double>>;
    const double w0 = 2.0 * M_PI / period;
    // In general the damped frequency ω = ω₀⋅sqrt(1 - ζ²) is complex given the
    // damping ratio might be greater than one (overdamped oscillator).
    const complex<double> w =
        w0 * sqrt(complex<double>{1.0 - damping_ratio * damping_ratio});
    const double sigma = damping_ratio * w0;

    // The general solution to the harmonic oscillator equation
    // ẍ + 2ζω₀ẋ + ω₀²x = 0 has the form x(t) = c₁⋅exp(λ₁⋅t) + c₂⋅exp(λ₂⋅t)
    // where λ₁ and λ₂ are the two (conjugate) roots of the second order
    // polynomial λ² + 2ζω₀λ + ω₀² = 0.
    const complex<double> lambda1 = -sigma + imaginary_unit * w;
    const complex<double> lambda2 = -sigma - imaginary_unit * w;

    // The constants c₁ and c₂ are determined from the initial conditions
    // leading to the linear system of equations A⋅c = b with A and b defined
    // below.
    Matrix2<complex<double>> A;
    A << 1.0, 1.0, lambda1, lambda2;
    const Vector2c b(x0, v0);

    // Sove for c = [c₁, c₂].
    const Vector2c constants = A.lu().solve(b);

    // Compute x(t) = c₁⋅exp(λ₁⋅t) + c₂⋅exp(λ₂⋅t)
    const complex<double> x =
        constants(0) * exp(lambda1 * time) +
        constants(1) *  exp(lambda2 * time);

    // For real initial conditions the imaginary part of the solution should
    // remain zero at all times.
    DRAKE_DEMAND(abs(x.imag()) < 10 * std::numeric_limits<double>::epsilon());

    return x.real() + free_length_;
  }

 protected:
  MultibodyPlant<double> plant_{0.0};
  const PrismaticJoint<double>* slider_{nullptr};

  // Parameters of the case.
  const double free_length_ = 1.0;  // [m]
};

// Verify the solution for an undamped system, ζ = 0.
TEST_F(SpringMassSystemTest, UnDampedCase) {
  // Plant's parameters.
  const double mass = 1.0;             // Mass of the body, [kg].
  const double period = 1.0;           // Period of oscillation, [s].
  const double damping_ratio = 0.0;    // Damping ratio, dimensionless.
  const double amplitude = 0.5;        // Initial amplitude, [m].

  // Length of the simulation, in seconds.
  const double simulation_time = 0.2;

  // Integration accuracy.
  const double integration_accuracy = 1.0e-6;

  MakeSpringMassSystem(mass, period, damping_ratio);

  Simulator<double> simulator(plant_);
  Context<double> &context = simulator.get_mutable_context();
  slider_->set_translation(&context, free_length_ + amplitude);
  slider_->set_translation_rate(&context, 0.0);
  simulator.Initialize();
  simulator.get_mutable_integrator().set_target_accuracy(integration_accuracy);
  simulator.AdvanceTo(simulation_time);

  const double x_analytic = CalcAnalyticSolution(
      period, damping_ratio, amplitude, 0.0, simulation_time);

  EXPECT_NEAR(
      slider_->get_translation(context), x_analytic, integration_accuracy);
}

// Verify the solution for an uderdamped system, ζ < 1.
TEST_F(SpringMassSystemTest, UnderDampedCase) {
  // Plant's parameters.
  const double mass = 1.0;             // Mass of the body, [kg].
  const double period = 1.0;           // Period of oscillation, [s].
  const double damping_ratio = 0.1;    // Damping ratio, dimensionless.
  const double amplitude = 0.5;        // Initial amplitude, [m].

  // Length of the simulation, in seconds.
  const double simulation_time = 0.2;

  // Integration accuracy.
  const double integration_accuracy = 1.0e-6;

  MakeSpringMassSystem(mass, period, damping_ratio);

  Simulator<double> simulator(plant_);
  Context<double>& context = simulator.get_mutable_context();
  slider_->set_translation(&context, free_length_ + amplitude);
  slider_->set_translation_rate(&context, 0.0);
  simulator.Initialize();
  simulator.get_mutable_integrator().set_target_accuracy(integration_accuracy);
  simulator.AdvanceTo(simulation_time);

  const double x_analytic = CalcAnalyticSolution(
      period, damping_ratio, amplitude, 0.0, simulation_time);

  EXPECT_NEAR(
      slider_->get_translation(context), x_analytic, integration_accuracy);
}

// Verify the solution for an overdamped system, ζ > 1.
TEST_F(SpringMassSystemTest, OverDampedCase) {
  // Plant's parameters.
  const double mass = 1.0;             // Mass of the body, [kg].
  const double period = 1.0;           // Period of oscillation, [s].
  const double damping_ratio = 1.5;    // Damping ratio, dimensionless.
  const double amplitude = 0.5;        // Initial amplitude, [m].

  // Length of the simulation, in seconds.
  const double simulation_time = 0.2;

  // Integration accuracy.
  const double integration_accuracy = 1.0e-6;

  MakeSpringMassSystem(mass, period, damping_ratio);

  Simulator<double> simulator(plant_);
  Context<double>& context = simulator.get_mutable_context();
  slider_->set_translation(&context, free_length_ + amplitude);
  slider_->set_translation_rate(&context, 0.0);
  simulator.Initialize();
  simulator.get_mutable_integrator().set_target_accuracy(integration_accuracy);
  simulator.AdvanceTo(simulation_time);

  const double x_analytic = CalcAnalyticSolution(
      period, damping_ratio, amplitude, 0.0, simulation_time);

  EXPECT_NEAR(
      slider_->get_translation(context), x_analytic, integration_accuracy);
}

TEST_F(SpringMassSystemTest, Symbolic) {
  // Make the double-valued system.
  const double mass = 1.0;             // Mass of the body, [kg].
  const double period = 1.0;           // Period of oscillation, [s].
  const double damping_ratio = 0.0;    // Damping ratio, dimensionless.
  MakeSpringMassSystem(mass, period, damping_ratio);
  ASSERT_TRUE(is_symbolic_convertible(plant_));

  // Make the symbolic system.
  auto dut = MultibodyPlant<double>::ToSymbolic(plant_);
  auto context = dut->CreateDefaultContext();

  // Set the input and state to variables.
  using T = symbolic::Expression;
  const symbolic::Variable x("x");
  const symbolic::Variable v("v");
  dut->SetPositionsAndVelocities(context.get(), Vector2<T>(x, v));

  // Check the symbolic derivatives.  (For now, just check that xdd only
  // depends on x; we don't check its full expression.)
  const auto& derivatives = dut->EvalTimeDerivatives(*context);
  ASSERT_EQ(derivatives.size(), 2);
  EXPECT_PRED2(symbolic::test::ExprEqual, derivatives[0], v);
  EXPECT_EQ(derivatives[1].GetVariables(), symbolic::Variables{x});
}

}  // namespace
}  // namespace multibody
}  // namespace drake

