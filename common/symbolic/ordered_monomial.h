#pragma once
#include <memory>
#include <optional>

#include "drake/common/symbolic/polynomial.h"

namespace drake {
namespace symbolic {

/**
 * An interface for defining monomials with an attached ordering. A monomial
 * ordering is a binary ordering on the exponents of the monomials such that 1.)
 * xᵝ < xᵞ is a total ordering 2.) If xᵝ < xᵞ then xᵝxᵟ < xᵟxᵞ 3.) xᵝ < xᵞ is a
 * well ordering. Every non-empty subset has a smallest element.
 */
class OrderedMonomial : public Monomial {
 public:
  // Inherit the monomial constructors.
  using Monomial::Monomial;

  virtual ~OrderedMonomial() {}

  /**
   * Compares two monomials according to the prescribed order. Throws
   * @std::exception if the variables of this monomial are not equal to the
   * variables of @p m
   */
  bool operator<(const OrderedMonomial& m) const;
  bool operator<=(const OrderedMonomial& m) const;
  bool operator>(const OrderedMonomial& m) const;
  bool operator>=(const OrderedMonomial& m) const;

  /**
   * Returns the next monomial according to the prescribed order. Specifically,
   * we return the unique monomial m such that this < m and there does not
   * exists any other monomial n such that this < n < m.
   *
   */
  std::unique_ptr<OrderedMonomial> GetNextMonomial() const;

  /**
   * Returns the previous monomial according to the prescribed order.
   * Specifically, we return the unique monomial m such that m < this and there
   * does not exists any other monomial n such that m < n < this.
   *
   * In some orders, it may not be possible to explicitly represent the previous
   * monomial. For example, in the lexicographic order with x < y, the previous
   * monomial for x³y is x to an arbitrarily high power. This is because xⁿ <
   * x³y for every n.
   *
   * In this case, this method return nullopt. Also return nullopt if called on
   * the constant monomial.
   *
   */
  std::optional<std::unique_ptr<OrderedMonomial>> MaybeGetPreviousMonomial()
      const;

 protected:
  /** Non-virtual interface implementation for DoGetNextMonomial().*/
  virtual std::unique_ptr<OrderedMonomial> DoGetNextMonomial() const = 0;
  /** Non-virtual interface implementation for DoMaybeGetPreviousMonomial().*/
  virtual std::optional<std::unique_ptr<OrderedMonomial>>
  DoMaybeGetPreviousMonomial() const = 0;
  /** Non-virtual interface implementation for the < operator.*/
  virtual bool DoLessThanComparison(const OrderedMonomial& m) const = 0;
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OrderedMonomial)
};

}  // namespace symbolic
}  // namespace drake
