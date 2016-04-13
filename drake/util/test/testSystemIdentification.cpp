#include <iostream>
#include "drake/util/Polynomial.h"
#include "drake/util/SystemIdentification.h"
#include "drake/util/testUtil.h"
#include "gtest/gtest.h"

namespace drake {
namespace util {

typedef SystemIdentification<double> SID;

TEST(SystemIdentificationTest, testLumpedSingle) {
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

TEST(SystemIdentificationTest, testLumpedMulti) {
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
}

}  // namespace test
}  // namespace drake
