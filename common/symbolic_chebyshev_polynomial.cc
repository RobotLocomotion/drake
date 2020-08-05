// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
ChebyshevPolynomial::ChebyshevPolynomial(Variable var, int degree)
    : var_{std::move(var)}, degree_{degree} {
  DRAKE_DEMAND(degree_ >= 0);
}

namespace {
// We represent a Chebyshev polynomial as a map from monomial degree to its
// coefficient, so T₂(x) = 2x²-1 is represented by a vector
// Eigen::Vector3d(-1, 0, 2).
// In monomial_coeffs, each of its column represented one Chebyshev polynomial,
// At the start of the function:
// monomial_coeffs.col(*next_col_index) represents Tₙ₊₁(x),
// monomial_coeffs.col(*current_col_index) represents Tₙ(x)
// monomial_coeffs.col(*prev_col_index) represents Tₙ₋₁(x).
// At the end of this function, we increment n by 1, and
// monomial_coeffs.col(*next_col_index) represents Tₙ₊₂(x),
// monomial_coeffs.col(*current_col_index) represents Tₙ₊₁(x),
// monomial_coeffs.col(*prev_col_index) represents Tₙ(x).
void ToPolynomialImpl(Eigen::Matrix<int, Eigen::Dynamic, 3>* monomial_coeffs,
                      int* next_col_index, int* current_col_index,
                      int* prev_col_index) {
  // Use the recursive function Tₙ₊₂(x) = 2xTₙ₊₁(x) − Tₙ(x)
  // First put Tₙ₊₂(x) in the column prev_col_index, then point next_col_index
  // to Tₙ₊₂(x)
  monomial_coeffs->block(1, *prev_col_index, monomial_coeffs->rows() - 1, 1) =
      2 * monomial_coeffs->block(0, *next_col_index,
                                 monomial_coeffs->rows() - 1, 1);
  (*monomial_coeffs)(0, *prev_col_index) = 0;
  monomial_coeffs->col(*prev_col_index) -=
      monomial_coeffs->col(*current_col_index);
  const int tmp = *prev_col_index;
  *prev_col_index = *current_col_index;
  *current_col_index = *next_col_index;
  *next_col_index = tmp;
}
}  // namespace

Polynomial ChebyshevPolynomial::ToPolynomial() const {
  if (degree_ == 0) {
    // Return 1.
    return Polynomial(Monomial());
  } else if (degree_ == 1) {
    // Return x.
    return Polynomial(Monomial(var_, 1));
  } else if (degree_ == 2) {
    // Return 2x^2 - 1
    return Polynomial({{Monomial(var_, 2), 2}, {Monomial(), -1}});
  } else {
    // Use the recursion Tₙ₊₁(x) = 2xTₙ(x) − Tₙ₋₁(x)
    // We represent a Chebyshev polynomial as a map from monomial degree to its
    // coefficient, so T₂(x) = 2x²-1 is represented by a vector
    // Eigen::Vector3d(-1, 0, 2).
    Eigen::Matrix<int, Eigen::Dynamic, 3> monomial_coeffs =
        Eigen::Matrix<int, Eigen::Dynamic, 3>::Zero(degree_ + 1, 3);
    int next_col_index = 0;
    int current_col_index = 1;
    int prev_col_index = 2;
    // Set monomial_coeffs.col(0) to T₂(x) = 2x^2-1.
    monomial_coeffs(0, next_col_index) = -1;
    monomial_coeffs(2, next_col_index) = 2;
    // Set monomial_coeffs.col(1) to T₁(x) = x.
    monomial_coeffs(1, current_col_index) = 1;
    // Set monomial_coeffs.col(2) to T₀(x) = 1.
    monomial_coeffs(0, prev_col_index) = 1;
    // Now call ToPolynomialImpl for degree - 2 times,
    // monomial_coeffs.col(next_col_index) stores the monomial to coefficient
    // map for this Chebyshev polynomial.
    for (int i = 0; i < degree_ - 2; ++i) {
      ToPolynomialImpl(&monomial_coeffs, &next_col_index, &current_col_index,
                       &prev_col_index);
    }
    Polynomial::MapType monomial_to_coefficient_map;
    for (int i = 0; i <= degree_; ++i) {
      if (monomial_coeffs(i, next_col_index) != 0) {
        monomial_to_coefficient_map.emplace(Monomial(var_, i),
                                            monomial_coeffs(i, next_col_index));
      }
    }
    return Polynomial(monomial_to_coefficient_map);
  }
}

namespace {
// At the beginning of the function
// poly_prev_val stores Tₙ₋₁(x)
// poly_curr_val stores Tₙ(x)
// At the end of the function, we increment n by 1, and
// poly_prev_val stores Tₙ(x)
// poly_curr_val stores Tₙ₊₁(x)
void EvalImpl(double x_val, double* poly_curr_val, double* poly_prev_val) {
  const double old_poly_curr_val = *poly_curr_val;
  *poly_curr_val = 2 * x_val * (*poly_curr_val) - (*poly_prev_val);
  *poly_prev_val = old_poly_curr_val;
}
}  // namespace

double EvaluateChebyshevPolynomial(double var_val, int degree) {
  if (degree == 0) {
    return 1;
  } else if (degree == 1) {
    return var_val;
  } else {
    // Instead of using the equation
    // Tₙ(x) = cos(n*arccos(x)) when -1 <= x <= 1
    // Tₙ(x) = cosh(n*arccos(x)) when x >= 1
    // Tₙ(x) = (-1)ⁿcosh(n*arccos(-x)) when x <= -1
    // we compute the evaluation recursively. The main motivation is that the
    // recursive computation is numerically more stable than using arccos /
    // arccosh function.
    double poly_prev_val = 1;
    double poly_curr_val = var_val;
    for (int i = 0; i <= degree - 2; ++i) {
      EvalImpl(var_val, &poly_curr_val, &poly_prev_val);
    }
    return poly_curr_val;
  }
}

double ChebyshevPolynomial::Evaluate(double var_val) const {
  return EvaluateChebyshevPolynomial(var_val, degree_);
}

bool ChebyshevPolynomial::operator==(const ChebyshevPolynomial& other) const {
  if (degree() == 0 && other.degree() == 0) {
    return true;
  }
  return var().equal_to(other.var()) && degree() == other.degree();
}

bool ChebyshevPolynomial::operator!=(const ChebyshevPolynomial& other) const {
  return !(*this == other);
}

std::vector<std::pair<ChebyshevPolynomial, double>>
ChebyshevPolynomial::Differentiate() const {
  if (degree_ == 0) {
    // dT₀(x)/dx = 0, we return an empty vector.
    return std::vector<std::pair<ChebyshevPolynomial, double>>();
  }
  if (degree_ % 2 == 0) {
    // even degree Chebyshev polynomial, its derivative is
    // dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x), j is odd  and j <= n-1
    std::vector<std::pair<ChebyshevPolynomial, double>> derivative;
    derivative.reserve(degree_ / 2);
    for (int j = 1; j <= degree_ / 2; ++j) {
      derivative.emplace_back(ChebyshevPolynomial(var_, 2 * j - 1),
                              2 * degree_);
    }
    return derivative;
  } else {
    // Odd degree Chebyshev polynomial, its derivative is dTₙ(x)/dx = 2n ∑ⱼ
    // Tⱼ(x) - n, j is even and j <= n-1
    std::vector<std::pair<ChebyshevPolynomial, double>> derivative;
    derivative.reserve((degree_ + 1) / 2);
    // 2n*T₀(x) − n = n*T₀(x) since T₀(x)=1
    derivative.emplace_back(ChebyshevPolynomial(var_, 0), degree_);
    for (int j = 1; j < (degree_ + 1) / 2; ++j) {
      derivative.emplace_back(ChebyshevPolynomial(var_, 2 * j), 2 * degree_);
    }
    return derivative;
  }
}

std::ostream& operator<<(std::ostream& out, const ChebyshevPolynomial& p) {
  if (p.degree() == 0) {
    out << "T0()";
  } else {
    out << "T" << p.degree() << "(" << p.var() << ")";
  }
  return out;
}

bool ChebyshevPolynomial::operator<(const ChebyshevPolynomial& other) const {
  // First the special case if either or both lhs and rhs has degree 0.

  if (degree() == 0 || other.degree() == 0) {
    return degree() < other.degree();
  } else if (var().get_id() < other.var().get_id()) {
    return true;
  } else if (var() == other.var() && degree() < other.degree()) {
    return true;
  }
  return false;
}

}  // namespace symbolic
}  // namespace drake
