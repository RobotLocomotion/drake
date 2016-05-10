#include <iostream>
#include <sstream>
#include <map>

#include "gtest/gtest.h"

#include "drake/util/Polynomial.h"
#include "drake/util/TrigPoly.h"

using namespace Eigen;
using namespace std;

namespace drake {
namespace util {
namespace {

typedef std::map<TrigPolyd::VarType, double> MapType;
const double kPi = 3.1415926535897;

void test_serialization(TrigPolyd dut, std::string expect) {
  std::stringstream test_stream;
  test_stream << dut;
  EXPECT_EQ(test_stream.str(), expect);
}

TEST(TrigPolyTest, SmokeTest) {
  // Confirm that these conversions compile okay.
  TrigPolyd x(1.0);
  TrigPolyd y = 2.0;
  TrigPolyd z = 3;

  // Test something else.
  Polynomiald q("q");
  Polynomiald s("s");
  Polynomiald c("c");

  TrigPolyd p(q, s, c);

  test_serialization(p, "q1");
  test_serialization(sin(p), "s1");
  test_serialization(cos(p), "c1");
  test_serialization(sin(p) * p * p + cos(p), "s1*q1^2+c1");
  test_serialization(sin(p + p), "s1*c1+c1*s1");
}

TEST(TrigPolyTest, EvaluateMultivariateTest) {
  const TrigPolyd theta(Polynomiald("th"),
                        Polynomiald("sth"), Polynomiald("cth"));
  const TrigPolyd::VarType theta_var =
      theta.getPolynomial().getSimpleVariable();

  // Check some basic evaluations.
  EXPECT_EQ(theta.evaluateMultivariate(MapType {{theta_var, 1}}), 1);
  EXPECT_EQ(sin(theta).evaluateMultivariate(MapType {{theta_var, 0}}), 0);
  EXPECT_EQ(cos(theta).evaluateMultivariate(MapType {{theta_var, 0}}), 1);

  // Test that the pythagorean theorem is true for various angles.
  for (const double angle : {0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4}) {
    EXPECT_NEAR((sin(theta) * sin(theta) + cos(theta) * cos(theta) - 1)
                .evaluateMultivariate(MapType {{theta_var, angle}}),
                0,
                1e-6);
  }

  // Test an arbitrary multivariate polynomial.
  const TrigPolyd phi(Polynomiald("phi"),
                      Polynomiald("sphi"), Polynomiald("cphi"));
  const TrigPolyd::VarType phi_var = phi.getPolynomial().getSimpleVariable();
  const TrigPolyd multivariate = theta + phi * cos(theta) + sin(phi + theta);
  for (const double theta_value : {0., 1., kPi / 4, kPi / 2, -kPi / 4}) {
    for (const double phi_value : {0., 1., kPi / 4, kPi / 2, -kPi / 4}) {
      EXPECT_NEAR(multivariate.evaluateMultivariate(
          MapType {{theta_var, theta_value}, {phi_var, phi_value}}),
                  (theta_value + (phi_value * std::cos(theta_value)) +
                   std::sin(phi_value + theta_value)),
                  1e-6) << "phi: " << phi_value << " theta: " << theta_value;
    }
  }
}

}  // anonymous namespace
}  // namespace test
}  // namespace drake
