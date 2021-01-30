// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
RationalFunction::RationalFunction()
    : numerator_{} /* zero polynomial */, denominator_{1} {}

RationalFunction::RationalFunction(Polynomial numerator, Polynomial denominator)
    : numerator_{std::move(numerator)}, denominator_{std::move(denominator)} {
  if (denominator_.EqualTo(Polynomial() /* zero polynomial */)) {
    throw std::invalid_argument(
        "RationalFunction: the denominator should not be 0.");
  }
  DRAKE_ASSERT_VOID(CheckIndeterminates());
}

RationalFunction::RationalFunction(const Polynomial& p)
    : RationalFunction(p, Polynomial(1)) {}

RationalFunction::RationalFunction(double c)
    : RationalFunction(Polynomial(c), Polynomial(1)) {}

bool RationalFunction::EqualTo(const RationalFunction& f) const {
  return numerator_.EqualTo(f.numerator()) &&
         denominator_.EqualTo(f.denominator());
}

Formula RationalFunction::operator==(const RationalFunction& f) const {
  return denominator_ * f.numerator() == numerator_ * f.denominator();
}

Formula RationalFunction::operator!=(const RationalFunction& f) const {
  return !(*this == f);
}

std::ostream& operator<<(std::ostream& os, const RationalFunction& f) {
  os << "(" << f.numerator() << ") / (" << f.denominator() << ")";
  return os;
}

void RationalFunction::CheckIndeterminates() const {
  const Variables vars1{intersect(numerator_.indeterminates(),
                                  denominator_.decision_variables())};
  const Variables vars2{intersect(numerator_.decision_variables(),
                                  denominator_.indeterminates())};
  if (!vars1.empty() || !vars2.empty()) {
    std::ostringstream oss;
    oss << "RationalFunction " << *this << " is invalid.\n";
    if (!vars1.empty()) {
      oss << "The following variable(s) "
             "are used as indeterminates in the numerator and decision "
             "variables in the denominator at the same time:\n"
          << vars1 << ".\n";
    }
    if (!vars2.empty()) {
      oss << "The following variable(s) "
             "are used as decision variables in the numerator and "
             "indeterminates variables in the denominator at the same time:\n"
          << vars2 << ".\n";
    }
    throw std::logic_error(oss.str());
  }
}

RationalFunction& RationalFunction::operator+=(const RationalFunction& f) {
  numerator_ = numerator_ * f.denominator() + denominator_ * f.numerator();
  denominator_ *= f.denominator();
  return *this;
}

RationalFunction& RationalFunction::operator+=(const Polynomial& p) {
  numerator_ = p * denominator_ + numerator_;
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

RationalFunction& RationalFunction::operator-=(double c) { return *this += -c; }

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

RationalFunction& RationalFunction::operator*=(double c) {
  numerator_ *= c;
  return *this;
}

RationalFunction& RationalFunction::operator/=(const RationalFunction& f) {
  if (f.numerator().EqualTo(Polynomial())) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  numerator_ *= f.denominator();
  denominator_ *= f.numerator();
  DRAKE_ASSERT_VOID(CheckIndeterminates());
  return *this;
}

RationalFunction& RationalFunction::operator/=(const Polynomial& p) {
  if (p.EqualTo(Polynomial())) {
    throw std::logic_error("RationalFunction: operator/=: The divider is 0.");
  }
  denominator_ *= p;
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

RationalFunction operator+(RationalFunction f, double c) { return f += c; }

RationalFunction operator+(double c, RationalFunction f) { return f += c; }

RationalFunction operator-(RationalFunction f1, const RationalFunction& f2) {
  return f1 -= f2;
}

RationalFunction operator-(RationalFunction f, const Polynomial& p) {
  return f -= p;
}

RationalFunction operator-(const Polynomial& p, const RationalFunction& f) {
  return -f + p;
}

RationalFunction operator-(RationalFunction f, double c) { return f -= c; }

RationalFunction operator-(double c, RationalFunction f) { return f = -f + c; }

RationalFunction operator*(RationalFunction f1, const RationalFunction& f2) {
  return f1 *= f2;
}

RationalFunction operator*(RationalFunction f, const Polynomial& p) {
  return f *= p;
}

RationalFunction operator*(const Polynomial& p, RationalFunction f) {
  return f *= p;
}

RationalFunction operator*(RationalFunction f, double c) { return f *= c; }

RationalFunction operator*(double c, RationalFunction f) { return f *= c; }

RationalFunction operator/(RationalFunction f1, const RationalFunction& f2) {
  return f1 /= f2;
}

RationalFunction operator/(RationalFunction f, const Polynomial& p) {
  return f /= p;
}

RationalFunction operator/(const Polynomial& p, const RationalFunction& f) {
  return RationalFunction(p * f.denominator(), f.numerator());
}

RationalFunction operator/(RationalFunction f, double c) { return f /= c; }

RationalFunction operator/(double c, const RationalFunction& f) {
  return RationalFunction(c * f.denominator(), f.numerator());
}

RationalFunction pow(const RationalFunction& f, int n) {
  if (n == 0) {
    return RationalFunction(Polynomial(1), Polynomial(1));
  } else if (n >= 1) {
    return RationalFunction(pow(f.numerator(), n), pow(f.denominator(), n));
  } else {
    // n < 0
    return RationalFunction(pow(f.denominator(), -n), pow(f.numerator(), -n));
  }
}
}  // namespace symbolic
}  // namespace drake
