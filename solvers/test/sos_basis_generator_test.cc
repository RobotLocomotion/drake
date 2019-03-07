#include "drake/solvers/sos_basis_generator.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/symbolic_monomial_util.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::symbolic::Monomial;

typedef std::set<Monomial,
                 drake::symbolic::GradedReverseLexOrder<std::less<Variable>>>
    MonomialSet;

MonomialSet VectorToSet(const drake::VectorX<Monomial>& x) {
  MonomialSet x_set;
  for (int i = 0; i < x.size(); i++) {
    x_set.insert(x(i));
  }
  return x_set;
}

class SosBasisGeneratorTest : public ::testing::Test {
 public:
  MonomialSet GetMonomialBasis(const symbolic::Polynomial& poly) {
    drake::VectorX<Monomial> basis = ConstructMonomialBasis(poly);
    MonomialSet basis_set = VectorToSet(basis);
    return basis_set;
  }

 protected:
  void SetUp() override { x_ = prog_.NewIndeterminates<3>(); }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
};

TEST_F(SosBasisGeneratorTest, MotzkinPoly) {
  const symbolic::Polynomial poly{pow(x_(0), 2) * pow(x_(1), 4) +
                                  pow(x_(0), 4) * pow(x_(1), 2) +
                                  pow(x_(0), 2) * pow(x_(1), 2) + +1};
  MonomialSet basis_ref;
  basis_ref.insert(Monomial());
  basis_ref.insert(Monomial(pow(x_(0), 1) * pow(x_(1), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 2) * pow(x_(1), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 1) * pow(x_(1), 2)));
  EXPECT_EQ(basis_ref, GetMonomialBasis(poly));
}

TEST_F(SosBasisGeneratorTest, Univariate) {
  const symbolic::Polynomial poly{1 + pow(x_(0), 1) + pow(x_(0), 2) +
                                  pow(x_(0), 8)};
  MonomialSet basis_ref;
  basis_ref.insert(Monomial());
  basis_ref.insert(Monomial(pow(x_(0), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 2)));
  basis_ref.insert(Monomial(pow(x_(0), 3)));
  basis_ref.insert(Monomial(pow(x_(0), 4)));
  EXPECT_EQ(basis_ref, GetMonomialBasis(poly));
}

TEST_F(SosBasisGeneratorTest, Empty) {
  const symbolic::Polynomial poly{pow(x_(0), 3)};
  MonomialSet basis_ref;
  EXPECT_EQ(basis_ref, GetMonomialBasis(poly));
}

TEST_F(SosBasisGeneratorTest, NewtonPolytopeEmptyInterior) {
  const symbolic::Polynomial poly{pow(x_(0), 2) * pow(x_(1), 2) +
                                  pow(x_(0), 3) * pow(x_(1), 3) +
                                  pow(x_(0), 4) * pow(x_(1), 4)};
  MonomialSet basis_ref;
  basis_ref.insert(Monomial(pow(x_(0), 1) * pow(x_(1), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 2) * pow(x_(1), 2)));
  EXPECT_EQ(basis_ref, GetMonomialBasis(poly));
}

TEST_F(SosBasisGeneratorTest, Singleton) {
  const symbolic::Polynomial poly{1 + pow(x_(0), 1)};
  MonomialSet basis_ref;
  basis_ref.insert(Monomial());
  EXPECT_EQ(basis_ref, GetMonomialBasis(poly));
}

}  // namespace
}  // namespace solvers
}  // namespace drake
