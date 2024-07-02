#include "drake/common/symbolic/ordered_monomial.h"
namespace drake {
namespace symbolic {

bool OrderedMonomial::operator<(const OrderedMonomial& m) const {
  DRAKE_THROW_UNLESS(m.GetVariables() == GetVariables());
  return DoLessThanComparison(m);
}

bool OrderedMonomial::operator<=(const OrderedMonomial& m) const {
  return (*this < m) || (*this == m);
}

bool OrderedMonomial::operator>(const OrderedMonomial& m) const {
  return !(*this <= m);
}

bool OrderedMonomial::operator>=(const OrderedMonomial& m) const {
  return !(*this < m);
}

std::unique_ptr<OrderedMonomial> OrderedMonomial::GetNextMonomial() const {
  return DoGetNextMonomial();
}

}  // namespace symbolic
}  // namespace drake
