// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <algorithm>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

using std::accumulate;
using std::make_pair;
using std::map;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;

namespace drake {
namespace symbolic {
namespace {
template <typename BasisElement>
Variables GetIndeterminates(
    const typename GenericPolynomial<BasisElement>::MapType& m) {
  static_assert(
      std::is_base_of<PolynomialBasisElement, BasisElement>::value,
      "BasisElement should be a derived class of PolynomialBasisElement");
  Variables vars;
  for (const pair<const BasisElement, Expression>& p : m) {
    const BasisElement& m_i{p.first};
    vars += m_i.GetVariables();
  }
  return vars;
}

template <typename BasisElement>
Variables GetDecisionVariables(
    const typename GenericPolynomial<BasisElement>::MapType& m) {
  static_assert(
      std::is_base_of<PolynomialBasisElement, BasisElement>::value,
      "BasisElement should be a derived class of PolynomialBasisElement");
  Variables vars;
  for (const pair<const BasisElement, Expression>& p : m) {
    const Expression& e_i{p.second};
    vars += e_i.GetVariables();
  }
  return vars;
}
}  // namespace

template <typename BasisElement>
GenericPolynomial<BasisElement>::GenericPolynomial(MapType init)
    : basis_element_to_coefficient_map_{move(init)},
      indeterminates_{
          GetIndeterminates<BasisElement>(basis_element_to_coefficient_map_)},
      decision_variables_{GetDecisionVariables<BasisElement>(
          basis_element_to_coefficient_map_)} {
  static_assert(
      std::is_base_of<PolynomialBasisElement, BasisElement>::value,
      "BasisElement should be a derived class of PolynomialBasisElement");
  // DRAKE_ASSERT_VOID(CheckInvariant());
}

template <typename BasisElement>
void GenericPolynomial<BasisElement>::CheckInvariant() const {
  // TODO(hongkai.dai and soonho.kong): improves the computation time of
  // CheckInvariant(). See github issue
  // https://github.com/RobotLocomotion/drake/issues/10229
  Variables vars{intersect(decision_variables(), indeterminates())};
  if (!vars.empty()) {
    ostringstream oss;
    oss << "Polynomial " << *this
        << " does not satisfy the invariant because the following variable(s) "
           "are used as decision variables and indeterminates at the same "
           "time:\n"
        << vars << ".";
    throw runtime_error(oss.str());
  }
}

template class GenericPolynomial<MonomialBasisElement>;
template class GenericPolynomial<ChebyshevBasisElement>;
}  // namespace symbolic
}  // namespace drake
