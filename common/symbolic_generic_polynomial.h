#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>
#include <ostream>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents symbolic generic polynomials using a given basis (for example,
 * monomial basis, Chebyshev basis, etc). A generic symbolic polynomial keeps a
 * mapping from a basis element of indeterminates to its coefficient in a
 * symbolic expression. A generic polynomial `p` has to satisfy an invariant
 * such that `p.decision_variables() ∩ p.indeterminates() = ∅`. We have
 * CheckInvariant() method to check the invariant.
 *
 * Note that arithmetic operations (+,-,*) between a Polynomial and a Variable
 * are not provided. The problem is that Variable class has no intrinsic
 * information if a variable is a decision variable or an indeterminate while we
 * need this information to perform arithmetic operations over Polynomials.
 * We provide two instantiations of this template
 * - BasisElement = MonomialBasisElement
 * - BasisElement = ChebyshevBasisElement
 * @tparam BasisElement Must be a subclass of PolynomialBasisElement.
 */
template <typename BasisElement>
class GenericPolynomial {
 public:
  static_assert(
      std::is_base_of<PolynomialBasisElement, BasisElement>::value,
      "BasisElement should be a derived class of PolynomialBasisElement");
  /** Type of mapping from basis element to coefficient */
  using MapType = std::map<BasisElement, Expression>;

  /** Constructs a zero polynomial. */
  GenericPolynomial() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericPolynomial)

  /** Constructs a default value. This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit GenericPolynomial(std::nullptr_t)
      : GenericPolynomial<BasisElement>() {}

  /** Constructs a generic polynomial from a map, basis_element → coefficient.
   * For example
   * GenericPolynomial<MonomialBasiElement>({{MonomialBasisElement(x, 2), a},
   * {MonomialBasisElement(x, 3), a+b}}) constructs a polynomial ax²+(a+b)x³.*/
  explicit GenericPolynomial(MapType init);

  /** Constructs a generic polynomial from a single basis element @p m. Note
   * that all variables in `m` are considered as indeterminates. Namely the
   * constructed generic polynomial contains the map with a single key `m`, with
   * the coefficient being 1.
   */
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<BasisElement>s return a GenericPolynomial<BasisElement>.
  // NOLINTNEXTLINE(runtime/explicit)
  GenericPolynomial(const BasisElement& m);

  /** Returns the indeterminates of this generic polynomial. */
  const Variables& indeterminates() const { return indeterminates_; }

  /** Returns the decision variables of this generic polynomial. */
  const Variables& decision_variables() const { return decision_variables_; }

  /** Returns the map from each basis element to its coefficient. */
  const MapType& basis_element_to_coefficient_map() const {
    return basis_element_to_coefficient_map_;
  }

  /** Returns the highest degree of this generic polynomial in an indeterminate
   * @p v. */
  int Degree(const Variable& v) const;

  /** Returns the total degree of this generic polynomial. */
  int TotalDegree() const;

  /**
   * Differentiates this generic polynomial with respect to the variable @p x.
   * Note that a variable @p x can be either a decision variable or an
   * indeterminate.
   */
  GenericPolynomial<BasisElement> Differentiate(const Variable& x) const;

  /** Returns true if this generic polynomial and @p p are structurally equal.
   */
  bool EqualTo(const GenericPolynomial<BasisElement>& p) const;

 private:
  // Throws std::runtime_error if there is a variable appeared in both of
  // decision_variables() and indeterminates().
  void CheckInvariant() const;

  MapType basis_element_to_coefficient_map_;
  Variables indeterminates_;
  Variables decision_variables_;
};

template <typename BasisElement>
std::ostream& operator<<(std::ostream& os,
                         const GenericPolynomial<BasisElement>& p) {
  const typename GenericPolynomial<BasisElement>::MapType& map{
      p.basis_element_to_coefficient_map()};
  if (map.empty()) {
    return os << 0;
  }
  auto it = map.begin();
  os << it->second << "*" << it->first;
  for (++it; it != map.end(); ++it) {
    os << " + " << it->second << "*" << it->first;
  }
  return os;
}

extern template class GenericPolynomial<MonomialBasisElement>;
extern template class GenericPolynomial<ChebyshevBasisElement>;
}  // namespace symbolic
}  // namespace drake
