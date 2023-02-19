#undef EIGEN_NO_IO

#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/symbolic/chebyshev_basis_element.h"
#include "drake/common/symbolic/monomial_basis_element.h"
#include "drake/common/symbolic/polynomial.h"

namespace drake {
namespace symbolic {
class OstreamTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
};

TEST_F(OstreamTest, ChebyshevBasisElementEigenMatrix) {
  // Checks we can have an Eigen matrix of ChebyshevBasisElements without
  // compilation errors. No assertions in the test.
  Eigen::Matrix<ChebyshevBasisElement, 2, 2> M;
  M << ChebyshevBasisElement(), ChebyshevBasisElement({{var_x_, 1}}),
      ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}}),
      ChebyshevBasisElement({{var_y_, 2}});

  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::DerivedA>`
  std::ostringstream oss;
  oss << M;
}


// Checks we can have an Eigen matrix of Monomials without compilation
// errors. No assertions in the test.
TEST_F(OstreamTest, MonomialEigenMatrix) {
  Eigen::Matrix<Monomial, 2, 2> M;
  // M = | 1   x    |
  //     | y²  x²z³ |
  // clang-format off
  M << Monomial{}, Monomial{var_x_},
       Monomial{{{var_y_, 2}}}, Monomial{{{var_x_, 2}, {var_z_, 3}}};
  // clang-format on

  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::Monomial>`.
  std::ostringstream oss;
  oss << M;
}

// Checks we can have an Eigen matrix of Monomials without compilation
// errors. No assertions in the test.
TEST_F(OstreamTest, MonomialBasisElementEigenMatrix) {
  Eigen::Matrix<MonomialBasisElement, 2, 2> M;
  // M = | 1   x    |
  //     | y²  x²z³ |
  M << MonomialBasisElement{}, MonomialBasisElement{var_x_},
      MonomialBasisElement{{{var_y_, 2}}},
      MonomialBasisElement{{{var_x_, 2}, {var_z_, 3}}};

  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::MonomialBasisElement>`.
  std::ostringstream oss;
  oss << M;
}

}  // namespace symbolic
}  // namespace drake
