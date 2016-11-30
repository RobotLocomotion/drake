#include "drake/common/trig_poly.h"

#include <sstream>
#include <map>

#include "gtest/gtest.h"

#include "drake/common/polynomial.h"

namespace drake {
namespace util {
namespace {

typedef std::map<TrigPolyd::VarType, double> MapType;
const double kPi = 3.1415926535897;

void TestSerializationContains(TrigPolyd dut, std::string expect) {
  std::stringstream test_stream;
  test_stream << dut;
  EXPECT_TRUE(test_stream.str().find(expect) != std::string::npos);
}

/// Tests that @p dut begins with @p expect followed by ' ' or end-of-string.
void TestSerializationFirstWord(TrigPolyd dut, std::string expect) {
  std::stringstream test_stream;
  test_stream << dut;
  EXPECT_EQ(test_stream.str().substr(0, expect.size()), expect);
  EXPECT_TRUE(test_stream.str().size() == expect.size() ||
              test_stream.str().substr(expect.size())[0] == ' ');
}

GTEST_TEST(TrigPolyTest, SmokeTest) {
  // Confirm that these conversions compile okay.
  TrigPolyd x(1.0);
  TrigPolyd y = 2.0;
  TrigPolyd z = 3;

  // Test something else.
  Polynomiald q("q");
  Polynomiald s("s");
  Polynomiald c("c");

  TrigPolyd p(q, s, c);

  // Require that serialization works, and that it contains definitions for
  // any sin_cos_map terms.
  TestSerializationFirstWord(p, "q1");
  TestSerializationFirstWord(sin(p), "s1");
  TestSerializationContains(sin(p), "s1=sin(q1)");
  TestSerializationFirstWord(cos(p), "c1");
  TestSerializationContains(cos(p), "c1=cos(q1)");

  // The following results could reasonably change if Polynomial changes its
  // monomial sorting or fixes #2216.  They are retained here to catch any
  // inadvertent changes to this behaviour.
  TestSerializationFirstWord(sin(p) * p * p + cos(p), "s1*q1^2+c1");
  TestSerializationFirstWord(sin(p + p), "s1*c1+c1*s1");
}

GTEST_TEST(TrigPolyTest, GetVariablesTest) {
  // Check that GetVariables() correctly returns all and only the base
  // variables of the expression and not the trig variables.
  Polynomiald q("q");
  Polynomiald s("s");
  Polynomiald c("c");
  TrigPolyd p(q, s, c);

  std::set<Polynomiald::VarType> expected_vars = {q.GetSimpleVariable(), };
  EXPECT_EQ(p.GetVariables(), expected_vars);
  EXPECT_EQ(sin(p).GetVariables(), expected_vars);
  EXPECT_EQ(cos(p).GetVariables(), expected_vars);
}

GTEST_TEST(TrigPolyTest, EvaluateMultivariateTest) {
  const TrigPolyd theta(Polynomiald("th"),
                        Polynomiald("sth"), Polynomiald("cth"));
  const TrigPolyd::VarType theta_var =
      theta.poly().GetSimpleVariable();

  // Check some basic evaluations.
  EXPECT_EQ(theta.EvaluateMultivariate(MapType {{theta_var, 1}}), 1);
  EXPECT_EQ(sin(theta).EvaluateMultivariate(MapType {{theta_var, 0}}), 0);
  EXPECT_EQ(cos(theta).EvaluateMultivariate(MapType {{theta_var, 0}}), 1);

  // Test that the pythagorean theorem is true for various angles.
  for (const double angle : {0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4}) {
    EXPECT_NEAR((sin(theta) * sin(theta) + cos(theta) * cos(theta) - 1)
                .EvaluateMultivariate(MapType {{theta_var, angle}}),
                0,
                1e-6);
  }

  // Test an arbitrary multivariate polynomial.
  const TrigPolyd phi(Polynomiald("phi"),
                      Polynomiald("sphi"), Polynomiald("cphi"));
  const TrigPolyd::VarType phi_var = phi.poly().GetSimpleVariable();
  const TrigPolyd multivariate = theta + phi * cos(theta) + sin(phi + theta);
  for (const double theta_value : {0., 1., kPi / 4, kPi / 2, -kPi / 4}) {
    for (const double phi_value : {0., 1., kPi / 4, kPi / 2, -kPi / 4}) {
      EXPECT_NEAR(multivariate.EvaluateMultivariate(
          MapType {{theta_var, theta_value}, {phi_var, phi_value}}),
                  (theta_value + (phi_value * std::cos(theta_value)) +
                   std::sin(phi_value + theta_value)),
                  1e-6) << "phi: " << phi_value << " theta: " << theta_value;
    }
  }
}

GTEST_TEST(TrigPolyTest, EvaluatePartialTest) {
  const TrigPolyd theta(Polynomiald("th"),
                        Polynomiald("sth"), Polynomiald("cth"));
  const TrigPolyd::VarType theta_var =
          theta.poly().GetSimpleVariable();
  const TrigPolyd phi(Polynomiald("phi"),
                      Polynomiald("sphi"), Polynomiald("cphi"));
  const TrigPolyd::VarType phi_var = phi.poly().GetSimpleVariable();
  const TrigPolyd multivariate = theta + phi * cos(theta) + sin(phi + theta);

  EXPECT_EQ(multivariate.EvaluatePartial(MapType {{theta_var, 0}}),
            phi + sin(phi));
  EXPECT_EQ(multivariate.EvaluatePartial(MapType {{phi_var, 0}}),
            theta + sin(theta));
  // TODO(#2216) This fails due to a known drake bug:
#if 0
  EXPECT_EQ(multivariate.evaluatePartial(MapType {{phi_var, 1}}),
            theta + cos(theta) + sin(theta + 1));
#endif
}

}  // anonymous namespace
}  // namespace util
}  // namespace drake
