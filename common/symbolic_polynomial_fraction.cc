// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
PolynomialFraction::PolynomialFraction()
    : numerator_{} /* zero polynomial */, denominator_{1} {}

PolynomialFraction::PolynomialFraction(const Polynomial& numerator,
                                       const Polynomial& denominator)
    : numerator_{numerator}, denominator_{denominator} {
  if (denominator_.EqualTo(Polynomial() /* zero polynomial */)) {
    throw std::invalid_argument(
        "PolynomialFraction: the denominator should not be 0.");
  }
  CheckInvariant();
}

bool PolynomialFraction::EqualTo(const PolynomialFraction& f) const {
  return numerator_.EqualTo(f.numerator()) &&
         denominator_.EqualTo(f.denominator());
}

std::ostream& operator<<(std::ostream& os, const PolynomialFraction& f) {
  os << "(" << f.numerator() << ") / (" << f.denominator() << ")";
  return os;
}

void PolynomialFraction::CheckInvariant() const {
  const Variables vars1{intersect(numerator_.indeterminates(),
                                  denominator_.decision_variables())};
  if (!vars1.empty()) {
    std::ostringstream oss;
    oss << "Polynomial fraction " << *this
        << " does not satisfy the invariant because the following variable(s) "
           "are used as indeterminates in the numerator and decision "
           "variables in the denominator at the same time:\n"
        << vars1 << ".";
    throw std::logic_error(oss.str());
  }
  const Variables vars2{intersect(numerator_.decision_variables(),
                                  denominator_.indeterminates())};
  if (!vars2.empty()) {
    std::ostringstream oss;
    oss << "Polynomial fraction " << *this
        << " does not satisfy the invariant because the following variable(s) "
           "are used as decision variables in the numerator and indeterminates"
           "variables in the denominator at the same time:\n"
        << vars2 << ".";
    throw std::logic_error(oss.str());
  }
}

PolynomialFraction& PolynomialFraction::operator+=(
    const PolynomialFraction& f) {
  const Polynomial denominator_old = denominator_;
  denominator_ = denominator_old * f.denominator();
  numerator_ = numerator_ * f.denominator() + denominator_old * f.numerator();
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator+=(const Polynomial& p) {
  numerator_ = p * denominator_ + numerator_;
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator+=(double c) {
  numerator_ = c * denominator_ + numerator_;
  return *this;
}

PolynomialFraction& PolynomialFraction::operator-=(
    const PolynomialFraction& f) {
  *this += -f;
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator-=(const Polynomial& p) {
  *this += -p;
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator-=(double c) {
  return *this += -c;
}

PolynomialFraction& PolynomialFraction::operator*=(
    const PolynomialFraction& f) {
  numerator_ *= f.numerator();
  denominator_ *= f.denominator();
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator*=(const Polynomial& p) {
  numerator_ *= p;
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator*=(double c) {
  numerator_ *= c;
  return *this;
}

PolynomialFraction& PolynomialFraction::operator/=(
    const PolynomialFraction& f) {
  if (f.numerator().EqualTo(Polynomial())) {
    throw std::logic_error("PolynomialFraction: operator/=: The divider is 0.");
  }
  numerator_ *= f.denominator();
  denominator_ *= f.numerator();
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator/=(const Polynomial& p) {
  if (p.EqualTo(Polynomial())) {
    throw std::logic_error("PolynomialFraction: operator/=: The divider is 0.");
  }
  denominator_ *= p;
  CheckInvariant();
  return *this;
}

PolynomialFraction& PolynomialFraction::operator/=(double c) {
  if (c == 0) {
    throw std::logic_error("PolynomialFraction: operator/=: The divider is 0.");
  }
  denominator_ *= c;
  return *this;
}

PolynomialFraction operator-(PolynomialFraction f) {
  return PolynomialFraction(-f.numerator(), f.denominator());
}

PolynomialFraction operator+(PolynomialFraction f1,
                             const PolynomialFraction& f2) {
  return f1 += f2;
}

PolynomialFraction operator+(PolynomialFraction f, const Polynomial& p) {
  return f += p;
}

PolynomialFraction operator+(const Polynomial& p, PolynomialFraction f) {
  return f += p;
}

PolynomialFraction operator+(PolynomialFraction f, double c) { return f += c; }

PolynomialFraction operator+(double c, PolynomialFraction f) { return f += c; }

PolynomialFraction operator-(PolynomialFraction f1,
                             const PolynomialFraction& f2) {
  return f1 -= f2;
}

PolynomialFraction operator-(PolynomialFraction f, const Polynomial& p) {
  return f -= p;
}

PolynomialFraction operator-(const Polynomial& p, PolynomialFraction f) {
  return f = -f + p;
}

PolynomialFraction operator-(PolynomialFraction f, double c) { return f -= c; }

PolynomialFraction operator-(double c, PolynomialFraction f) {
  return f = -f + c;
}

PolynomialFraction operator*(PolynomialFraction f1,
                             const PolynomialFraction& f2) {
  return f1 *= f2;
}

PolynomialFraction operator*(PolynomialFraction f, const Polynomial& p) {
  return f *= p;
}

PolynomialFraction operator*(const Polynomial& p, PolynomialFraction f) {
  return f *= p;
}

PolynomialFraction operator*(PolynomialFraction f, double c) { return f *= c; }

PolynomialFraction operator*(double c, PolynomialFraction f) { return f *= c; }

PolynomialFraction operator/(PolynomialFraction f1,
                             const PolynomialFraction& f2) {
  return f1 /= f2;
}

PolynomialFraction operator/(PolynomialFraction f, const Polynomial& p) {
  return f /= p;
}

PolynomialFraction operator/(const Polynomial& p, const PolynomialFraction& f) {
  return PolynomialFraction(p * f.denominator(), f.numerator());
}

PolynomialFraction operator/(PolynomialFraction f, double c) { return f /= c; }

PolynomialFraction operator/(double c, const PolynomialFraction& f) {
  return PolynomialFraction(c * f.denominator(), f.numerator());
}

PolynomialFraction pow(const PolynomialFraction& f, int n) {
  if (n == 0) {
    return PolynomialFraction(Polynomial(1), Polynomial(1));
  } else if (n >= 1) {
    return PolynomialFraction(pow(f.numerator(), n), pow(f.denominator(), n));
  } else {
    // n < 0
    return PolynomialFraction(pow(f.denominator(), -n), pow(f.numerator(), -n));
  }
}
}  // namespace symbolic
}  // namespace drake
