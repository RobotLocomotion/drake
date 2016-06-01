#include "drake/solvers/system_identification.h"

#include <random>  // Used only with deterministic seeds!

#include "gtest/gtest.h"

#include "drake/util/Polynomial.h"

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
    a.getSimpleVariable(),
    b.getSimpleVariable(),
    c.getSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomial(input, parameters);
  EXPECT_EQ(lump_map.size(), 2);
  EXPECT_EQ(lump_map.count(a + b), 1);
  EXPECT_EQ(lump_map.count(a * c), 1);
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
    a.getSimpleVariable(),
    b.getSimpleVariable(),
    c.getSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomials(input, parameters);

  // Note that we expect that 'a' and '2*a' will collapse to one lumped param.
  EXPECT_EQ(lump_map.size(), 3);
  EXPECT_EQ(lump_map.count(a), 1);
  EXPECT_EQ(lump_map.count(a + b), 1);
  EXPECT_EQ(lump_map.count(a * c), 1);

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
    a.getSimpleVariable(),
    b.getSimpleVariable(),
    c.getSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomials(input, parameters);

  // A point for testing numeric stability.
  std::map<Polynomiald::VarType, double> eval_point = {
    {x.getSimpleVariable(), 1},
    {y.getSimpleVariable(), 2},
    {a.getSimpleVariable(), 3},
    {b.getSimpleVariable(), 5},
    {c.getSimpleVariable(), 7},
  };
  // Compute the value of each lumped parameter at eval_point; store those
  // values into the eval_point (so that now it provides both lumped and
  // un-lumped values and can be used to evaluate either the original or the
  // rewritten polynomial).
  for (const auto& poly_var_pair : lump_map) {
    eval_point[poly_var_pair.second] =
        poly_var_pair.first.evaluateMultivariate(eval_point);
  }

  for (const Polynomiald& poly : input) {
    Polynomiald rewritten =
        SID::RewritePolynomialWithLumpedParameters(poly, lump_map);

    // No non-lumped parameters should remain in rewritten.
    EXPECT_EQ(rewritten.getVariables().count(a.getSimpleVariable()), 0);
    EXPECT_EQ(rewritten.getVariables().count(b.getSimpleVariable()), 0);
    EXPECT_EQ(rewritten.getVariables().count(c.getSimpleVariable()), 0);

    // Rewritten has the same or smaller number of variables and terms.
    EXPECT_LE(rewritten.getVariables().size(), poly.getVariables().size());
    EXPECT_LE(rewritten.getMonomials().size(), poly.getMonomials().size());

    // Rewriting in terms of lumped parameters should never change the
    // actual value of a polynomial at a particular point.
    EXPECT_EQ(poly.evaluateMultivariate(eval_point),
              rewritten.evaluateMultivariate(eval_point));

    // TODO(ggould-tri) The above tests do not ensure that the original and
    // rewritten polys are everywhere and always structurally identical, just
    // nearly always identical in their evaluateMultivariate behaviour.
  }
}

// The current windows CI build has no solver for generic constraints.  The
// DISABLED_ logic below ensures that we still at least get compile-time
// checking of the test and resulting template instantiations.
#if !defined(WIN32) && !defined(WIN64)
#define BASIC_ESTIMATE_TEST_NAME BasicEstimateParameters
#else
#define BASIC_ESTIMATE_TEST_NAME DISABLED_BasicEstimateParameters
#endif

GTEST_TEST(SystemIdentificationTest, BASIC_ESTIMATE_TEST_NAME) {
  const Polynomiald x = Polynomiald("x");
  const auto x_var = x.getSimpleVariable();
  const Polynomiald y = Polynomiald("y");
  const auto y_var = y.getSimpleVariable();
  const Polynomiald z = Polynomiald("z");
  const auto z_var = z.getSimpleVariable();
  const Polynomiald a = Polynomiald("a");
  const auto a_var = a.getSimpleVariable();
  const Polynomiald b = Polynomiald("b");
  const auto b_var = b.getSimpleVariable();
  const Polynomiald c = Polynomiald("c");
  const auto c_var = c.getSimpleVariable();

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
    EXPECT_EQ(estimated_params.size(), 3);
    for (const auto& var : {a_var, b_var, c_var}) {
      EXPECT_NEAR(estimated_params[var], expected_params.at(var), 4 * error);
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
    EXPECT_EQ(estimated_params.size(), 3);
    for (const auto& var : {a_var, b_var, c_var}) {
      EXPECT_NEAR(estimated_params[var], expected_params.at(var), 4 * error);
    }
  }
}

#undef BASIC_ESTIMATE_TEST_NAME

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

// The current windows CI build has no solver for generic constraints.  The
// DISABLED_ logic below ensures that we still at least get compile-time
// checking of the test and resulting template instantiations.
#if !defined(WIN32) && !defined(WIN64)
#define IDENTIFICATION_TEST_NAME SpringMassIdentification
#else
#define IDENTIFICATION_TEST_NAME DISABLED_SpringMassIdentification
#endif

// TODO(ggould-tri) It is likely that much of the logic below will be
// boilerplate shared by all manipulator identification; it should eventually
// be pulled into a function of its own inside of system_identification.

GTEST_TEST(SystemIdentificationTest, IDENTIFICATION_TEST_NAME) {
  Polynomiald pos = Polynomiald("pos");
  auto pos_var = pos.getSimpleVariable();
  Polynomiald velocity = Polynomiald("vel");
  auto velocity_var = velocity.getSimpleVariable();
  Polynomiald acceleration = Polynomiald("acc");
  auto acceleration_var = acceleration.getSimpleVariable();
  Polynomiald input_force = Polynomiald("f_in");
  auto input_force_var = input_force.getSimpleVariable();
  Polynomiald mass = Polynomiald("m");
  auto mass_var = mass.getSimpleVariable();
  Polynomiald damping = Polynomiald("b");
  auto damping_var = damping.getSimpleVariable();
  Polynomiald spring = Polynomiald("k");
  auto spring_var = spring.getSimpleVariable();

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
  EXPECT_LT(error, 0.3);

  EXPECT_EQ(estimated_params.size(), 3);
  EXPECT_NEAR(estimated_params[mass_var], kMass, kNoise);
  EXPECT_NEAR(estimated_params[damping_var], kDamping,
              measurements.size() * error);
  EXPECT_NEAR(estimated_params[spring_var], kSpring, kNoise);
}
#undef IDENTIFICATION_TEST_NAME

///@}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
