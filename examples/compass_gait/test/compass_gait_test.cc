#include "drake/examples/compass_gait/compass_gait.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace examples {
namespace compass_gait {

using std::cos;
using std::sin;
using std::sqrt;
using symbolic::Expression;
using symbolic::Variable;

namespace {

GTEST_TEST(CompassGaitTest, ScalarTypeTests) {
  CompassGait<double> cg;
  EXPECT_TRUE(is_autodiffxd_convertible(cg));
  EXPECT_TRUE(is_symbolic_convertible(cg));
}

GTEST_TEST(CompassGaitTest, TestEnergyConservedInSwing) {
  CompassGait<Expression> cg;

  auto context = cg.CreateDefaultContext();
  CompassGaitContinuousState<Expression>& state =
      cg.get_mutable_continuous_state(context.get());
  auto derivatives = cg.AllocateTimeDerivatives();

  // Set the state vector to be symbolic variables.
  state.SetToNamedVariables();

  // Compute the time-derivative of the energy symbolically.
  const Expression energy =
      cg.CalcKineticEnergy(*context) + cg.CalcPotentialEnergy(*context);
  cg.CalcTimeDerivatives(*context, derivatives.get());
  const Expression energy_dot =
      energy.Jacobian(GetVariableVector(state.CopyToVector())) *
      derivatives->CopyToVector();

  // Evaluate the time-derivative of energy at a random state, it should be
  // zero.
  symbolic::Environment e;
  e.insert(get_variable(state.stance()), 1.);
  e.insert(get_variable(state.swing()), 2.);
  e.insert(get_variable(state.stancedot()), 3.);
  e.insert(get_variable(state.swingdot()), 4.);

  EXPECT_NEAR(energy_dot.Evaluate(e), 0.0, 5e-14);
}

GTEST_TEST(CompassGaitTest, TestCollisionGuard) {
  const CompassGait<double> cg;
  auto context = cg.CreateDefaultContext();
  CompassGaitContinuousState<double>& state =
      cg.get_mutable_continuous_state(context.get());
  const CompassGaitParams<double>& params = cg.get_parameters(*context);

  const systems::WitnessFunction<double>* foot_collision;
  {
    std::vector<const systems::WitnessFunction<double>*> witness_functions;
    cg.GetWitnessFunctions(*context, &witness_functions);
    EXPECT_EQ(witness_functions.size(), 1);
    foot_collision = witness_functions[0];
  }

  // Foot is above the ground.
  state.set_stance(.1);
  state.set_swing(-.1);
  EXPECT_GT(cg.CalcWitnessValue(*context, *foot_collision), 0.);

  // Foot is below the ground, but stepping backward.
  state.set_stance(-.1);
  state.set_swing(.1);
  EXPECT_GT(cg.CalcWitnessValue(*context, *foot_collision), 0.);

  // Foot is exactly touching the ground.
  state.set_stance(.1 + params.slope());
  state.set_swing(-.1 + params.slope());
  EXPECT_NEAR(cg.CalcWitnessValue(*context, *foot_collision), 0., 1e-12);
}

double CalcAngularMomentum(const CompassGait<double>& cg,
                           const systems::Context<double>& context,
                           bool about_stance_foot) {
  const CompassGaitContinuousState<double>& state =
      cg.get_continuous_state(context);
  const CompassGaitParams<double>& params = cg.get_parameters(context);
  const double m = params.mass_leg();
  const double mh = params.mass_hip();
  const double a = params.length_leg() - params.center_of_mass_leg();
  const double b = params.center_of_mass_leg();
  const double l = params.length_leg();
  const double cst = cos(state.stance());
  const double csw = cos(state.swing());
  const double sst = sin(state.stance());
  const double ssw = sin(state.swing());
  const double vst = state.stancedot();
  const double vsw = state.swingdot();

  // Position and velocity of the mass in the stance leg.
  const Eigen::Vector2d p_m_stance{a * sst, a * cst};
  const Eigen::Vector2d v_m_stance{a * cst * vst, -a * sst * vst};

  // Position and velocity of the mass in the hip.
  const Eigen::Vector2d p_mh{l * sst, l * cst};
  const Eigen::Vector2d v_mh{l * cst * vst, -l * sst * vst};

  // Position and velocity of the mass in the stance leg.
  const Eigen::Vector2d p_m_swing = p_mh - Eigen::Vector2d{b * ssw, b * csw};
  const Eigen::Vector2d v_m_swing =
      v_mh - Eigen::Vector2d{b * csw * vsw, -b * ssw * vsw};

  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  if (!about_stance_foot) {
    origin = p_mh - Eigen::Vector2d{l * ssw, l * csw};
  }

  // Compute the out-of-plane component of the cross product of these planar
  // vectors.  (inputs are in x and z, output is y axis).
  const auto my_cross = [](const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
    return u(1) * v(0) - u(0) * v(1);
  };

  // ∑ r × mv.
  return my_cross(p_m_stance - origin, m * v_m_stance) +
         my_cross(p_mh - origin, mh * v_mh) +
         my_cross(p_m_swing - origin, m * v_m_swing);
}

GTEST_TEST(CompassGaitTest, TestCollisionDynamics) {
  // Check that angular momentum is conserved about the impact.
  // (This fact was not used explicitly in the derivation).
  const CompassGait<double> cg;
  auto context = cg.CreateDefaultContext();
  CompassGaitContinuousState<double>& state =
      cg.get_mutable_continuous_state(context.get());
  const CompassGaitParams<double>& params = cg.get_parameters(*context);

  const systems::WitnessFunction<double>* foot_collision;
  {
    std::vector<const systems::WitnessFunction<double>*> witness_functions;
    cg.GetWitnessFunctions(*context, &witness_functions);
    EXPECT_EQ(witness_functions.size(), 1);
    foot_collision = witness_functions[0];
  }

  // Foot is touching the ground.
  state.set_stance(.2 + params.slope());
  state.set_swing(-.2 + params.slope());
  state.set_stancedot(2.);
  state.set_swingdot(-1.);

  const double angular_momentum_before =
      CalcAngularMomentum(cg, *context, false);

  // Evaluate collision dynamics.
  auto next_context = cg.CreateDefaultContext();
  dynamic_cast<const systems::UnrestrictedUpdateEvent<double>*>(
      foot_collision->get_event())
      ->handle(*context, &next_context->get_mutable_state());

  const double angular_momentum_after =
      CalcAngularMomentum(cg, *next_context, true);
  EXPECT_NEAR(angular_momentum_before, angular_momentum_after, 1e-8);
}

}  // namespace
}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
