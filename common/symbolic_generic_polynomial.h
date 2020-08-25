#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <algorithm>
#include <functional>
#include <map>
#include <ostream>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents symbolic polynomials using a given basis (for example, monomial
 * basis, Chebyshev basis, etc). A symbolic polynomial keeps a mapping from a
 * basis element of indeterminates to its coefficient in a symbolic expression.
 * A polynomial `p` has to satisfy an invariant such that
 * `p.decision_variables() ∩ p.indeterminates() = ∅`. We have CheckInvariant()
 * method to check the invariant.
 *
 * Note that arithmetic operations (+,-,*) between a Polynomial and a Variable
 * are not provided. The problem is that Variable class has no intrinsic
 * information if a variable is a decision variable or an indeterminate while we
 * need this information to perform arithmetic operations over Polynomials.
 */
template <typename BasisElement>
class GenericPolynomial {
 public:
  using MapType = std::map<BasisElement, Expression>;

  /** Constructs a zero polynomial. */
  GenericPolynomial() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericPolynomial)

  /** Constructs a default value. This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit GenericPolynomial(std::nullptr_t)
      : GenericPolynomial<BasisElement>() {}

  /** Constructs a polynomial from a map, basis_element → Expression. */
  explicit GenericPolynomial(MapType init);

  /** Constructs a polynomial from a single basis element @p m. Note that all
   * variables in `m` are considered as indeterminates. Namely the constructed
   * polynomial contains the map with a single key `m`, with the coefficient
   * being 1.
   */
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<BasisElement>s return a GenericPolynomial<BasisElement>.
  // NOLINTNEXTLINE(runtime/explicit)
  GenericPolynomial(const BasisElement& m);

  /** Returns the indeterminates of this polynomial. */
  const Variables& indeterminates() const { return indeterminates_; }

  /** Returns the decision variables of this polynomial. */
  const Variables& decision_variables() const { return decision_variables_; }

  const MapType& basis_element_to_coefficient_map() const {
    return basis_element_to_coefficient_map_;
  }

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
}  // namespace symbolic
}  // namespace drake
