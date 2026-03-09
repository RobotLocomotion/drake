#include "drake/common/symbolic/rational_function.h"

#include <utility>

namespace drake {
namespace symbolic {

RationalFunction::RationalFunction()
    : numerator_{} /* zero polynomial */, denominator_{1} {}

RationalFunction::RationalFunction(Polynomial numerator, Polynomial denominator)
    : numerator_{std::move(numerator)}, denominator_{std::move(denominator)} {
  DRAKE_DEMAND(!denominator_.monomial_to_coefficient_map()
                    .empty() /* zero polynomial */);
  DRAKE_ASSERT_VOID(CheckIndeterminates());
}

RationalFunction::RationalFunction(const Polynomial& p)
    : RationalFunction(p, Polynomial(1)) {}

RationalFunction::RationalFunction(const Monomial& m)
    : RationalFunction(Polynomial(m)) {}

RationalFunction::RationalFunction(double c)
    : RationalFunction(Polynomial(c), Polynomial(1)) {}

bool RationalFunction::EqualTo(const RationalFunction& f) const {
  return numerator_.EqualTo(f.numerator()) &&
         denominator_.EqualTo(f.denominator());
}

double RationalFunction::Evaluate(const Environment& env) const {
  return numerator_.Evaluate(env) / denominator_.Evaluate(env);
}

Formula RationalFunction::operator==(const RationalFunction& f) const {
  return denominator_ * f.numerator() == numerator_ * f.denominator();
}

Formula RationalFunction::operator!=(const RationalFunction& f) const {
  return !(*this == f);
}

std::ostream& operator<<(std::ostream& os, const RationalFunction& f) {
  return os << fmt::to_string(f);
}

std::string to_string(const RationalFunction& f) {
  return fmt::format("({}) / ({})", f.numerator(), f.denominator());
}

void RationalFunction::CheckIndeterminates() const {
  const Variables vars1{intersect(numerator_.indeterminates(),
                                  denominator_.decision_variables())};
  const Variables vars2{intersect(numerator_.decision_variables(),
                                  denominator_.indeterminates())};
  if (!vars1.empty() || !vars2.empty()) {
    std::string err_msg{
        fmt::format("RationalFunction {} is invalid.\n", *this)};
    if (!vars1.empty()) {
      err_msg.append(fmt::format(
          "The following variable(s) are used as indeterminates in the "
          "numerator and decision variables in the denominator at the same "
          "time:\n{}.\n",
          vars1));
    }
    if (!vars2.empty()) {
      err_msg.append(
          fmt::format("The following variable(s) "
                      "are used as decision variables in the numerator and "
                      "indeterminates variables in the denominator at the same "
                      "time:\n{}.\n",
                      vars2));
    }
    throw std::logic_error(err_msg);
  }
}

Expression RationalFunction::ToExpression() const {
  return numerator_.ToExpression() / denominator_.ToExpression();
}

RationalFunction& RationalFunction::operator+=(const RationalFunction& f) {
  if (f.denominator().EqualTo(denominator_)) {
    numerator_ = numerator_ + f.numerator();
  } else {
    numerator_ = numerator_ * f.denominator() + denominator_ * f.numerator();
    denominator_ *= f.denominator();
  }
  return *this;
}

RationalFunction& RationalFunction::operator+=(const Polynomial& p) {
  numerator_ = p * denominator_ + numerator_;
  return *this;
}

RationalFunction& RationalFunction::operator+=(const Monomial& m) {
  numerator_ = m * denominator_ + numerator_;
  return *this;
}

RationalFunction& RationalFunction::operator+=(double c) {
  numerator_ = c * denominator_ + numerator_;
  return *this;
}

RationalFunction& RationalFunction::operator-=(const RationalFunction& f) {
  *this += -f;
  return *this;
}

RationalFunction& RationalFunction::operator-=(const Polynomial& p) {
  *this += -p;
  return *this;
}

RationalFunction& RationalFunction::operator-=(const Monomial& m) {
  *this += -m;
  return *this;
}

RationalFunction& RationalFunction::operator-=(double c) {
  return *this += -c;
}

RationalFunction& RationalFunction::operator*=(const RationalFunction& f) {
  numerator_ *= f.numerator();
  denominator_ *= f.denominator();
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator*=(const Polynomial& p) {
  numerator_ *= p;
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator*=(const Monomial& m) {
  numerator_ *= m;
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator*=(double c) {
  numerator_ *= c;
  return *this;
}

RationalFunction& RationalFunction::operator/=(const RationalFunction& f) {
  if (f.numerator().monomial_to_coefficient_map().empty()) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  numerator_ *= f.denominator();
  denominator_ *= f.numerator();
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator/=(const Polynomial& p) {
  if (p.monomial_to_coefficient_map().empty()) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  denominator_ *= p;
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator/=(const Monomial& m) {
  if (m.total_degree() == 0) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  denominator_ *= m;
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator/=(double c) {
  if (c == 0) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  denominator_ *= c;
  return *this;
}

RationalFunction operator-(RationalFunction f) {
  f.numerator_ *= -1;
  return f;
}

RationalFunction operator+(RationalFunction f1, const RationalFunction& f2) {
  return f1 += f2;
}

RationalFunction operator+(RationalFunction f, const Polynomial& p) {
  return f += p;
}

RationalFunction operator+(const Polynomial& p, RationalFunction f) {
  return f += p;
}

RationalFunction operator+(RationalFunction f, const Monomial& m) {
  return f += m;
}

RationalFunction operator+(const Monomial& m, RationalFunction f) {
  return f += m;
}

RationalFunction operator+(RationalFunction f, double c) {
  return f += c;
}

RationalFunction operator+(double c, RationalFunction f) {
  return f += c;
}

RationalFunction operator-(RationalFunction f1, const RationalFunction& f2) {
  return f1 -= f2;
}

RationalFunction operator-(RationalFunction f, const Polynomial& p) {
  return f -= p;
}

RationalFunction operator-(const Polynomial& p, const RationalFunction& f) {
  return -f + p;
}

RationalFunction operator-(RationalFunction f, const Monomial& m) {
  return f -= m;
}

RationalFunction operator-(const Monomial& m, RationalFunction f) {
  return -f + m;
}

RationalFunction operator-(RationalFunction f, double c) {
  return f -= c;
}

RationalFunction operator-(double c, RationalFunction f) {
  return f = -f + c;
}

RationalFunction operator*(RationalFunction f1, const RationalFunction& f2) {
  return f1 *= f2;
}

RationalFunction operator*(RationalFunction f, const Polynomial& p) {
  return f *= p;
}

RationalFunction operator*(const Polynomial& p, RationalFunction f) {
  return f *= p;
}

RationalFunction operator*(RationalFunction f, const Monomial& m) {
  return f *= m;
}

RationalFunction operator*(const Monomial& m, RationalFunction f) {
  return f *= m;
}

RationalFunction operator*(RationalFunction f, double c) {
  return f *= c;
}

RationalFunction operator*(double c, RationalFunction f) {
  return f *= c;
}

RationalFunction operator/(RationalFunction f1, const RationalFunction& f2) {
  return f1 /= f2;
}

RationalFunction operator/(RationalFunction f, const Polynomial& p) {
  return f /= p;
}

RationalFunction operator/(const Polynomial& p, const RationalFunction& f) {
  if (f.numerator().monomial_to_coefficient_map().empty()) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  return {p * f.denominator(), f.numerator()};
}

RationalFunction operator/(RationalFunction f, const Monomial& m) {
  return f /= m;
}

RationalFunction operator/(const Monomial& m, RationalFunction f) {
  if (f.numerator().monomial_to_coefficient_map().empty()) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  return {m * f.denominator(), f.numerator()};
}

RationalFunction operator/(RationalFunction f, double c) {
  return f /= c;
}

RationalFunction operator/(double c, const RationalFunction& f) {
  if (f.numerator().monomial_to_coefficient_map().empty()) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  return {c * f.denominator(), f.numerator()};
}

RationalFunction pow(const RationalFunction& f, int n) {
  if (n == 0) {
    return {Polynomial(1), Polynomial(1)};
  } else if (n >= 1) {
    return {pow(f.numerator(), n), pow(f.denominator(), n)};
  } else {
    // n < 0
    return {pow(f.denominator(), -n), pow(f.numerator(), -n)};
  }
}

void RationalFunction::SetIndeterminates(const Variables& new_indeterminates) {
  numerator_.SetIndeterminates(new_indeterminates);
  denominator_.SetIndeterminates(new_indeterminates);
}
}  // namespace symbolic
}  // namespace drake

// We must define this in the cc file so that symbolic_formula.h is fully
// defined (not just forward declared) when comparing.
namespace Eigen {
namespace numext {
template <>
bool equal_strict(const drake::symbolic::RationalFunction& x,
                  const drake::symbolic::RationalFunction& y) {
  return static_cast<bool>(x == y);
}
}  // namespace numext
}  // namespace Eigen
