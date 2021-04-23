#include "drake/examples/compass_gait/compass_gait.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
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
  cg.get_input_port(0).FixValue(context.get(), Vector1<Expression>(0.0));
  CompassGaitContinuousState<Expression>& state =
      cg.get_mutable_continuous_state(context.get());

  // Set the state vector to be symbolic variables.
  state.SetToNamedVariables();

  // Compute the time-derivative of the energy symbolically.
  const Expression energy =
      cg.EvalKineticEnergy(*context) + cg.EvalPotentialEnergy(*context);
  const VectorX<Expression> derivatives =
      cg.EvalTimeDerivatives(*context).CopyToVector();
  const Expression energy_dot =
      energy.Jacobian(GetVariableVector(state.CopyToVector())) * derivatives;

  // Evaluate the time-derivative of energy at a few arbitrary states, it
  // should be zero.
  for (int i = 0; i < 4; i++) {
    symbolic::Environment e;
    e.insert(get_variable(state.stance()), i + 1.);
    e.insert(get_variable(state.swing()), i + 2.);
    e.insert(get_variable(state.stancedot()), i + 3.);
    e.insert(get_variable(state.swingdot()), i + 4.);

    EXPECT_NEAR(energy_dot.Evaluate(e), 0.0, 2e-13);
  }
}

GTEST_TEST(CompassGaitTest, TestHipTorque) {
  // Position the compass gait with the swing leg horizontal (straight
  // forward), and the center of mass over the foot.  This should be a fixed
  // point with the torque balancing the swing leg:
  //  hip_torque = mass_leg * gravity * center_of_mass_leg.
  // x position of the center of mass = 0 =>
  //  sin(stance) * (mass_leg * center_of_mass_leg
  //    + (mass_leg + mass_hip) * length_leg) = mass_leg * center_of_mass_leg

  CompassGait<double> cg;

  auto context = cg.CreateDefaultContext();
  const CompassGaitParams<double>& params = cg.get_parameters(*context);
  cg.get_input_port(0).FixValue(
      context.get(), Vector1<double>(params.mass_leg() * params.gravity() *
                                     params.center_of_mass_leg()));
  CompassGaitContinuousState<double>& state =
      cg.get_mutable_continuous_state(context.get());
  state.set_stance(std::asin(
      params.mass_leg() * params.center_of_mass_leg() /
      (params.mass_leg() * params.center_of_mass_leg() +
       (params.mass_leg() + params.mass_hip()) * params.length_leg())));
  state.set_swing(M_PI_2);
  state.set_stancedot(0.0);
  state.set_swingdot(0.0);

  const VectorX<double> derivatives =
      cg.EvalTimeDerivatives(*context).CopyToVector();

  EXPECT_TRUE(CompareMatrices(derivatives, Eigen::Vector4d::Zero(), 1e-15));
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
  EXPECT_NEAR(cg.CalcWitnessValue(*context, *foot_collision), 0., 1e-16);
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
  // Note: Stance can be any value > than slope that keeps the hip above ground.
  // Swing is set to put the swing toe in collision.  Velocities are arbitrary.
  const double kStanceRelativeToRamp = .2;
  state.set_stance(kStanceRelativeToRamp + params.slope());
  state.set_swing(-kStanceRelativeToRamp + params.slope());
  state.set_stancedot(2.);
  state.set_swingdot(-1.);

  const double angular_momentum_before =
      CalcAngularMomentum(cg, *context, false);

  // Evaluate collision dynamics.
  auto next_context = cg.CreateDefaultContext();
  dynamic_cast<const systems::UnrestrictedUpdateEvent<double>*>(
      foot_collision->get_event())
      ->handle(cg, *context, &next_context->get_mutable_state());

  const double angular_momentum_after =
      CalcAngularMomentum(cg, *next_context, true);
  EXPECT_NEAR(angular_momentum_before, angular_momentum_after, 5e-15);

  // Ensure that the toe moved forward.
  EXPECT_GT(cg.get_toe_position(*next_context), cg.get_toe_position(*context));
}

GTEST_TEST(CompassGaitTest, TestMinimalStateOutput) {
  const CompassGait<double> cg;
  auto context = cg.CreateDefaultContext();
  CompassGaitContinuousState<double>& state =
      cg.get_mutable_continuous_state(context.get());

  auto output = cg.get_minimal_state_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(output->get_value<systems::BasicVector<double>>());
  const systems::BasicVector<double>& minimal_state =
      output->get_value<systems::BasicVector<double>>();

  state.set_stance(1.);
  state.set_swing(2.);
  state.set_stancedot(3.);
  state.set_swingdot(4.);

  cg.get_minimal_state_output_port().Calc(*context, output.get());
  EXPECT_TRUE(CompareMatrices(minimal_state.CopyToVector(),
                              state.CopyToVector(), 1e-14));
  EXPECT_FALSE(
      cg.HasDirectFeedthrough(cg.get_minimal_state_output_port().get_index()));
}

GTEST_TEST(CompassGaitTest, TestFloatBaseOutput) {
  const CompassGait<double> cg;
  auto context = cg.CreateDefaultContext();
  CompassGaitContinuousState<double>& state =
      cg.get_mutable_continuous_state(context.get());
  const CompassGaitParams<double>& params = cg.get_parameters(*context);

  auto output = cg.get_floating_base_state_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(output->get_value<systems::BasicVector<double>>());
  const systems::BasicVector<double>& floating_base_state =
      output->get_value<systems::BasicVector<double>>();

  // Standing on the left with the hip straight above the toe.
  cg.set_left_leg_is_stance(true, &context->get_mutable_state());
  state.set_stance(0.);
  state.set_swing(1.);
  state.set_stancedot(2.);
  state.set_swingdot(3.);
  cg.set_toe_position(0., &context->get_mutable_state());
  Eigen::VectorXd expected(14);
  // clang-format off
  expected << 0., 0., params.length_leg(),     // x, y, z
              0., 0., 0.,                      // roll, pitch, yaw (of left)
              state.swing() - state.stance(),  // hip angle
              params.length_leg()*state.stancedot(), 0., 0.,  // velocities
              0., state.stancedot(), 0.,
              state.swingdot() - state.stancedot();
  // clang-format on

  cg.get_floating_base_state_output_port().Calc(*context, output.get());
  EXPECT_TRUE(
      CompareMatrices(floating_base_state.CopyToVector(), expected, 1e-14));

  // Now standing on the right with the hip straight above the toe.
  cg.set_left_leg_is_stance(false, &context->get_mutable_state());

  // clang-format off
  expected << 0., 0., params.length_leg(),     // x, y, z
      0., state.swing(), 0.,                   // roll, pitch, yaw (of left)
      state.stance() - state.swing(),          // -hip angle
      params.length_leg()*state.stancedot(), 0., 0.,  // velocities
      0., state.swingdot(), 0.,
      state.stancedot() - state.swingdot();
  // clang-format on

  cg.get_floating_base_state_output_port().Calc(*context, output.get());
  EXPECT_TRUE(
      CompareMatrices(floating_base_state.CopyToVector(), expected, 1e-14));
  EXPECT_FALSE(cg.HasDirectFeedthrough(
      cg.get_floating_base_state_output_port().get_index()));
}

}  // namespace
}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
