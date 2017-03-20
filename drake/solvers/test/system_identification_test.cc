#include "drake/solvers/system_identification.h"

#include <random>  // Used only with deterministic seeds!

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/polynomial.h"
#include "drake/common/trig_poly.h"

namespace drake {
namespace solvers {
namespace {

typedef SystemIdentification<double> SID;

GTEST_TEST(SystemIdentificationTest, LumpedSingle) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  Polynomiald a = Polynomiald("a");
  Polynomiald b = Polynomiald("b");
  Polynomiald c = Polynomiald("c");

  /* From the SystemIdentification.h doxygen */
  Polynomiald input = (a * x) + (b * x) + (a * c * y) + (a * c * y * y);

  std::set<Polynomiald::VarType> parameters = {
    a.GetSimpleVariable(),
    b.GetSimpleVariable(),
    c.GetSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomial(input, parameters);
  EXPECT_EQ(lump_map.size(), 2u);
  EXPECT_EQ(lump_map.count(a + b), 1u);
  EXPECT_EQ(lump_map.count(a * c), 1u);
}

GTEST_TEST(SystemIdentificationTest, LumpedMulti) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  Polynomiald a = Polynomiald("a");
  Polynomiald b = Polynomiald("b");
  Polynomiald c = Polynomiald("c");

  std::vector<Polynomiald> input = {
    (a * x) + (b * x) + (a * c * y),
    (a * c * y * y),
    2 * a,
    a};

  std::set<Polynomiald::VarType> parameters = {
    a.GetSimpleVariable(),
    b.GetSimpleVariable(),
    c.GetSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomials(input, parameters);

  // Note that we expect that 'a' and '2*a' will collapse to one lumped param.
  EXPECT_EQ(lump_map.size(), 3u);
  EXPECT_EQ(lump_map.count(a), 1u);
  EXPECT_EQ(lump_map.count(a + b), 1u);
  EXPECT_EQ(lump_map.count(a * c), 1u);

  // TODO(ggould-tri) The above code should be able to be more cleanly written
  // using gmock as something like:
  //
  // EXPECT_THAT(lump_map, ElementsAre(std::make_pair(a, _),
  //                                   std::make_pair((a + b), _),
  //                                   std::make_pair((a * c), _)));
  //
  // but the author could not get this working.
}

GTEST_TEST(SystemIdentificationTest, LumpedParameterRewrite) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  Polynomiald a = Polynomiald("a");
  Polynomiald b = Polynomiald("b");
  Polynomiald c = Polynomiald("c");

  std::vector<Polynomiald> input = {
    (a * x) + (b * x) + (3 * a * c * y),
    (a * x) + (2 * b * x) + (3 * a * c * y),
    (a * c * y * y),
    2 * a,
    a};

  std::set<Polynomiald::VarType> parameters = {
    a.GetSimpleVariable(),
    b.GetSimpleVariable(),
    c.GetSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomials(input, parameters);

  // A point for testing numeric stability.
  std::map<Polynomiald::VarType, double> eval_point = {
    {x.GetSimpleVariable(), 1},
    {y.GetSimpleVariable(), 2},
    {a.GetSimpleVariable(), 3},
    {b.GetSimpleVariable(), 5},
    {c.GetSimpleVariable(), 7},
  };
  // Compute the value of each lumped parameter at eval_point; store those
  // values into the eval_point (so that now it provides both lumped and
  // un-lumped values and can be used to evaluate either the original or the
  // rewritten polynomial).
  for (const auto& poly_var_pair : lump_map) {
    eval_point[poly_var_pair.second] =
        poly_var_pair.first.EvaluateMultivariate(eval_point);
  }

  for (const Polynomiald& poly : input) {
    Polynomiald rewritten =
        SID::RewritePolynomialWithLumpedParameters(poly, lump_map);

    // No non-lumped parameters should remain in rewritten.
    EXPECT_EQ(rewritten.GetVariables().count(a.GetSimpleVariable()), 0u);
    EXPECT_EQ(rewritten.GetVariables().count(b.GetSimpleVariable()), 0u);
    EXPECT_EQ(rewritten.GetVariables().count(c.GetSimpleVariable()), 0u);

    // Rewritten has the same or smaller number of variables and terms.
    EXPECT_LE(rewritten.GetVariables().size(), poly.GetVariables().size());
    EXPECT_LE(rewritten.GetMonomials().size(), poly.GetMonomials().size());

    // Rewriting in terms of lumped parameters should never change the
    // actual value of a polynomial at a particular point.
    EXPECT_EQ(poly.EvaluateMultivariate(eval_point),
              rewritten.EvaluateMultivariate(eval_point));

    // TODO(ggould-tri) The above tests do not ensure that the original and
    // rewritten polys are everywhere and always structurally identical, just
    // nearly always identical in their EvaluateMultivariate behaviour.
  }
}

GTEST_TEST(SystemIdentificationTest, BasicEstimateParameters) {
  const Polynomiald x = Polynomiald("x");
  const auto x_var = x.GetSimpleVariable();
  const Polynomiald y = Polynomiald("y");
  const auto y_var = y.GetSimpleVariable();
  const Polynomiald z = Polynomiald("z");
  const auto z_var = z.GetSimpleVariable();
  const Polynomiald a = Polynomiald("a");
  const auto a_var = a.GetSimpleVariable();
  const Polynomiald b = Polynomiald("b");
  const auto b_var = b.GetSimpleVariable();
  const Polynomiald c = Polynomiald("c");
  const auto c_var = c.GetSimpleVariable();

  /// Parameter estimation will try to make this Polynomial evaluate to zero:
  const Polynomiald poly = (a * x) + (b * x * x) + (c * y) - z;

  { // A very simple test case in which the error is zero.
    const std::vector<SID::PartialEvalType> sample_points {
      {{x_var, 1}, {y_var, 1}, {z_var, 3}},
      {{x_var, 1}, {y_var, 2}, {z_var, 4}},
      {{x_var, 2}, {y_var, 1}, {z_var, 7}},
      {{x_var, 2}, {y_var, 2}, {z_var, 8}}};

    const SID::PartialEvalType expected_params {
      {a_var, 1}, {b_var, 1}, {c_var, 1}};

    SID::PartialEvalType estimated_params;
    double error;
    std::tie(estimated_params, error) =
        SID::EstimateParameters(VectorXPoly::Constant(1, poly),
                                sample_points);

    EXPECT_LT(error, 1e-5);
    EXPECT_EQ(estimated_params.size(), 3u);
    for (const auto& var : {a_var, b_var, c_var}) {
      // `9 * error` here in case all of the RMS error was in a single term.
      EXPECT_NEAR(estimated_params[var], expected_params.at(var),
                  9 * error);
    }
  }

  { // Test with some error injected.
    const std::vector<SID::PartialEvalType> sample_points {
      {{x_var, 1}, {y_var, 1}, {z_var, 3.05}},
      {{x_var, 1}, {y_var, 2}, {z_var, 3.95}},
      {{x_var, 2}, {y_var, 1}, {z_var, 7.05}},
      {{x_var, 2}, {y_var, 2}, {z_var, 8.05}}};

    const SID::PartialEvalType expected_params {
      {a_var, 1}, {b_var, 1}, {c_var, 1}};

    SID::PartialEvalType estimated_params;
    double error;
    std::tie(estimated_params, error) =
        SID::EstimateParameters(VectorXPoly::Constant(1, poly),
                                sample_points);

    EXPECT_LT(error, 0.1);
    EXPECT_EQ(estimated_params.size(), 3u);
    for (const auto& var : {a_var, b_var, c_var}) {
      EXPECT_NEAR(estimated_params[var], expected_params.at(var), 4 * error);
    }
  }
}

/// Test to check parameter estimation for a basic spring-mass system.
///@{

struct State { double acceleration, velocity, position, force; };
static const double kMass = 1;
static const double kDamping = 0.1;
static const double kSpring = 2;
static const double kNoise = 0.01;
static const double kNoiseSeed = 1;

State AdvanceState(const State& previous, double input_force, double dt) {
  State next{};
  next.force = input_force;
  next.acceleration = (input_force -
                       previous.velocity * kDamping -
                       previous.position * kSpring) / kMass;
  next.velocity = previous.velocity +
      (dt * (previous.acceleration + next.acceleration) / 2);
  next.position = previous.position +
      (dt * (previous.velocity + next.velocity) / 2);
  return next;
}

std::vector<State> MakeTestData() {
  static const double kDt = 0.01;
  static const double kDuration1 = 1;
  static const double kInputForce1 = 1;
  static const double kDuration2 = 3;
  static const double kInputForce2 = 0;
  static const State kInitial {0, 0, 0, 0};

  std::vector<State> result { kInitial };
  State current = kInitial;
  double t = 0;
  while (t < kDuration1) {
    current = AdvanceState(current, kInputForce1, kDt);
    result.push_back(current);
    t += kDt;
  }
  while (t < kDuration1 + kDuration2) {
    current = AdvanceState(current, kInputForce2, kDt);
    result.push_back(current);
    t += kDt;
  }

  return result;
}

// TODO(ggould-tri) It is likely that much of the logic below will be
// boilerplate shared by all manipulator identification; it should eventually
// be pulled into a function of its own inside of system_identification.
GTEST_TEST(SystemIdentificationTest, SpringMassIdentification) {
  Polynomiald pos = Polynomiald("pos");
  auto pos_var = pos.GetSimpleVariable();
  Polynomiald velocity = Polynomiald("vel");
  auto velocity_var = velocity.GetSimpleVariable();
  Polynomiald acceleration = Polynomiald("acc");
  auto acceleration_var = acceleration.GetSimpleVariable();
  Polynomiald input_force = Polynomiald("f_in");
  auto input_force_var = input_force.GetSimpleVariable();
  Polynomiald mass = Polynomiald("m");
  auto mass_var = mass.GetSimpleVariable();
  Polynomiald damping = Polynomiald("b");
  auto damping_var = damping.GetSimpleVariable();
  Polynomiald spring = Polynomiald("k");
  auto spring_var = spring.GetSimpleVariable();

  // Code style violations here:
  // * Vector initializations use two statements on one line for clarity.
  // * The short names and upper/lower case here are conventional in the
  //   manipulator formulation.
  //
  // We write the manipulator as:
  //   H*vdot + C*v + g = B*u + f
  // Where f embodies any forces not appropriate to C.
  VectorXPoly v(1); v << velocity;
  VectorXPoly vdot(1); vdot << acceleration;
  VectorXPoly H(1); H << mass;
  VectorXPoly C(1); C << 0;
  VectorXPoly g(1); g << (spring * pos);
  VectorXPoly f(1); f << (velocity * damping);
  VectorXPoly B(1); B << 1;
  VectorXPoly u(1); u << input_force;

  const VectorXPoly manipulator_left = (H * vdot) + (C * v) + g;
  const VectorXPoly manipulator_right = (B * u) + f;

  std::default_random_engine noise_generator;
  noise_generator.seed(kNoiseSeed);
  std::uniform_real_distribution<double> noise_distribution(-kNoise, kNoise);
  auto noise = std::bind(noise_distribution, noise_generator);

  const std::vector<State> oracular_data = MakeTestData();
  std::vector<SID::PartialEvalType> measurements;
  for (const State& oracular_state : oracular_data) {
    SID::PartialEvalType measurement;
    measurement[pos_var] = oracular_state.position + noise();
    measurement[velocity_var] = oracular_state.velocity + noise();
    measurement[acceleration_var] = oracular_state.acceleration + noise();
    measurement[input_force_var] = oracular_state.force + noise();
    measurements.push_back(measurement);
  }

  SID::PartialEvalType estimated_params;
  double error;
  std::tie(estimated_params, error) =
      SID::EstimateParameters(manipulator_left - manipulator_right,
                              measurements);

  // Multiple layers of naive discrete-time numeric integration yields a very
  // high error value here, which almost all lands in the damping constant
  // because it is the smallest term in the equation of motion.
  //
  // The value for the error check here is an arbitrary empirical observation,
  // to catch changes that heavily regress accuracy.
  EXPECT_LT(error, 2e-2);

  EXPECT_EQ(estimated_params.size(), 3u);
  EXPECT_NEAR(estimated_params[mass_var], kMass, kNoise);
  EXPECT_NEAR(estimated_params[damping_var], kDamping,
              measurements.size() * error);
  EXPECT_NEAR(estimated_params[spring_var], kSpring, kNoise);
}

GTEST_TEST(SystemIdentificationTest, PendulaIdentification) {
  // Simulate two pendula that swing independently but are actuated with the
  // same torque.  The pendula have lengths l1 = 1, l2 = 2; their masses m1
  // and m2 are both 1.  Gravity is an earth-conventional -9.8.
  //
  // The comments about nomenclature from the previous test apply here as
  // well: Variable naming is conventional rather than style-conformant.

  const TrigPolyd theta1(Polynomiald("th", 1),
                         Polynomiald("s", 1), Polynomiald("c", 1));
  const TrigPolyd theta2(Polynomiald("th", 2),
                         Polynomiald("s", 2), Polynomiald("c", 2));
  VectorXTrigPoly q(2); q << theta1, theta2;

  const TrigPolyd theta1dot(Polynomiald("th.", 1),
                            Polynomiald("s.", 1), Polynomiald("c.", 1));
  const TrigPolyd theta2dot(Polynomiald("th.", 2),
                            Polynomiald("s.", 2), Polynomiald("c.", 2));
  VectorXTrigPoly qdot(2); qdot << theta1dot, theta2dot;

  const TrigPolyd theta1dotdot(Polynomiald("th..", 1),
                               Polynomiald("s..", 1), Polynomiald("c..", 1));
  const TrigPolyd theta2dotdot(Polynomiald("th..", 2),
                               Polynomiald("s..", 2), Polynomiald("c..", 2));
  VectorXTrigPoly qdotdot(2); qdotdot << theta1dotdot, theta2dotdot;

  const TrigPolyd l1(Polynomiald("l", 1));  //< Length of arm 1.
  const TrigPolyd l2(Polynomiald("l", 2));  //< Length of arm 2.
  const TrigPolyd m1(Polynomiald("m", 1));  //< Mass of arm 1.
  const TrigPolyd m2(Polynomiald("m", 2));  //< Mass of arm 2.

  const TrigPolyd gravity(Polynomiald("g"));  //< gravity
  const TrigPolyd tau(Polynomiald("tau"));  //< torque

  // The following matrices and vectors are the components of the Manipulator.
  Eigen::Matrix<TrigPolyd, 2, 2> H;  //< Inertia matrix.
  H << (m1 * l1 * l1), 0,
       0, (m2 * l2 * l2);
  Eigen::Matrix<TrigPolyd, 2, 2> C;  //< Coriolis matrix.
  C << 0, 0, 0, 0;
  Eigen::Matrix<TrigPolyd, 2, 1> g;  //< Field function (gravity).
  g << m1 * gravity * l1 * sin(theta1), m2 * gravity * l2 * sin(theta2);
  Eigen::Matrix<TrigPolyd, 2, 1> B;  //< Input transmission mapping.
  B << 1, 1;
  Eigen::Matrix<TrigPolyd, 2, 1> f;  //< Dissipative forces.
  f << 0, 0;

  VectorXTrigPoly u(1); u << tau;  //< Input signals.

  const VectorXTrigPoly manipulator_left = (H * qdotdot) + (C * qdot) + g;
  const VectorXTrigPoly manipulator_right = (B * u) + f;
  const VectorXTrigPoly to_estimate = manipulator_left - manipulator_right;

  // Create convenience variables for our q/qdot/u values.  Convenience vars
  // have an underscore prefix for slightly easier understandability.
  const TrigPolyd::VarType th1_var = theta1.poly().GetSimpleVariable();
  const TrigPolyd::VarType th2_var = theta2.poly().GetSimpleVariable();
  const TrigPolyd::VarType th1d_var =
      theta1dot.poly().GetSimpleVariable();
  const TrigPolyd::VarType th2d_var =
      theta2dot.poly().GetSimpleVariable();
  const TrigPolyd::VarType th1dd_var =
      theta1dotdot.poly().GetSimpleVariable();
  const TrigPolyd::VarType th2dd_var =
      theta2dotdot.poly().GetSimpleVariable();
  const TrigPolyd::VarType tau_var = tau.poly().GetSimpleVariable();

  const double kG = 9.8;
  const double kPi = 3.14159265;
  const double kPi2 = kPi / 2;

  const std::vector<typename SID::PartialEvalType> pendula_data = {
    {{tau_var, 0.},
     {th1_var, 0.}, {th1d_var, 0.}, {th1dd_var, 0.},
     {th2_var, 0.}, {th2d_var, 0.}, {th2dd_var, 0.}},
    {{tau_var, 0.},
     {th1_var, kPi2}, {th1d_var, 0.}, {th1dd_var, -kG},
     {th2_var, kPi2}, {th2d_var, 0.}, {th2dd_var, -0.25 * kG}},
    {{tau_var, 0.},
     {th1_var, -kPi}, {th1d_var, 0.}, {th1dd_var, 0.},
     {th2_var, -kPi}, {th2d_var, 0.}, {th2dd_var, 0.}},
    {{tau_var, 0.},
     {th1_var, -kPi2}, {th1d_var, 0.}, {th1dd_var, kG},
     {th2_var, -kPi2}, {th2d_var, 0.}, {th2dd_var, 0.25 * kG}},
    {{tau_var, 1.},
     {th1_var, 0.}, {th1d_var, 0.}, {th1dd_var, 1.},
     {th2_var, 0.}, {th2d_var, 0.}, {th2dd_var, 0.25}},
    {{tau_var, kG},
     {th1_var, kPi2}, {th1d_var, 0.}, {th1dd_var, 0.},
     {th2_var, kPi2}, {th2d_var, 0.}, {th2dd_var, 0.}},
    {{tau_var, 1.},
     {th1_var, -kPi}, {th1d_var, 0.}, {th1dd_var, 1.},
     {th2_var, -kPi}, {th2d_var, 0.}, {th2dd_var, 0.25}},
    {{tau_var, -kG},
     {th1_var, -kPi2}, {th1d_var, 0.}, {th1dd_var, 0.},
     {th2_var, -kPi2}, {th2d_var, 0.}, {th2dd_var, 0.}},
    };

  SID::SystemIdentificationResult result =
      SID::LumpedSystemIdentification(to_estimate, pendula_data);

  // Check result.rms_error.
  const double epsilon = 1e-5;  // Moderate, empirical epsilon for weak solvers.
  EXPECT_LT(result.rms_error, epsilon);
  const double max_per_term_error =
      result.rms_error *
      (result.lumped_parameters.size() * result.lumped_parameters.size());

  // Check result.lumped_parameters.
  Polynomiald mgl1 = (m1 * gravity * l1).poly();
  Polynomiald mgl2 = (m2 * gravity * l2).poly();
  Polynomiald mll1 = (m1 * l1 * l1).poly();
  Polynomiald mll2 = (m2 * l2 * l2).poly();
  EXPECT_EQ(result.lumped_parameters.size(), static_cast<size_t>(4));
  Polynomiald::VarType mgl1_var = result.lumped_parameters.at(mgl1);
  Polynomiald::VarType mgl2_var = result.lumped_parameters.at(mgl2);
  Polynomiald::VarType mll1_var = result.lumped_parameters.at(mll1);
  Polynomiald::VarType mll2_var = result.lumped_parameters.at(mll2);

  // Check result.lumped_polys.
  std::set<Polynomiald::VarType> expected_vars_1 = {
    mgl1_var, mll1_var, th1_var, th1dd_var, tau_var};
  EXPECT_EQ(result.lumped_polys[0].GetVariables(), expected_vars_1);
  std::set<Polynomiald::VarType> expected_vars_2 = {
    mgl2_var, mll2_var, th2_var, th2dd_var, tau_var};
  EXPECT_EQ(result.lumped_polys[1].GetVariables(), expected_vars_2);

  // Check result.lumped_parameter_values
  EXPECT_NEAR(result.lumped_parameter_values[mgl1_var], kG, max_per_term_error);
  EXPECT_NEAR(result.lumped_parameter_values[mgl2_var], kG, max_per_term_error);
  EXPECT_NEAR(result.lumped_parameter_values[mll1_var], 1, max_per_term_error);
  EXPECT_NEAR(result.lumped_parameter_values[mll2_var], 4, max_per_term_error);

  // Check result.partially_evaluated_polys.
  for (const auto& point : pendula_data) {
    EXPECT_NEAR(result.partially_evaluated_polys[0].EvaluateMultivariate(point),
                0, max_per_term_error);
    EXPECT_NEAR(result.partially_evaluated_polys[1].EvaluateMultivariate(point),
                0, max_per_term_error);
  }
}

///@}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
