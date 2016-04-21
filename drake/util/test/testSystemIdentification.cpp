#include "drake/util/Polynomial.h"
#include "drake/util/SystemIdentification.h"
#include "gtest/gtest.h"

namespace drake {
namespace util {
namespace {

typedef SystemIdentification<double> SID;

TEST(SystemIdentificationTest, LumpedSingle) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  Polynomiald a = Polynomiald("a");
  Polynomiald b = Polynomiald("b");
  Polynomiald c = Polynomiald("c");

  /* From the SystemIdentification.h doxygen */
  Polynomiald input = (a * x) + (b * x) + (a * c * y) + (a * c * y * y);

  std::set<Polynomiald::VarType> interests = {
    x.getSimpleVariable(),
    y.getSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomial(input, interests);
  EXPECT_EQ(lump_map.size(), 2);
  EXPECT_EQ(lump_map.count(a + b), 1);
  EXPECT_EQ(lump_map.count(a * c), 1);
}

TEST(SystemIdentificationTest, LumpedMulti) {
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

  std::set<Polynomiald::VarType> interests = {
    x.getSimpleVariable(),
    y.getSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomials(input, interests);

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

TEST(SystemIdentificationTest, LumpedParameterRewrite) {
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

  std::set<Polynomiald::VarType> interests = {
    x.getSimpleVariable(),
    y.getSimpleVariable()};
  SID::LumpingMapType lump_map =
      SID::GetLumpedParametersFromPolynomials(input, interests);

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

}  // anonymous namespace
}  // namespace util
}  // namespace drake
