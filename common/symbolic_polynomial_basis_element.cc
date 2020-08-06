// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <typeinfo>
#include <utility>

#include <fmt/format.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
PolynomialBasisElement::PolynomialBasisElement(
    const std::map<Variable, int>& var_to_degree_map) {
  total_degree_ = std::accumulate(
      var_to_degree_map.begin(), var_to_degree_map.end(), 0,
      [](const int degree, const std::pair<const Variable, int>& p) {
        return degree + p.second;
      });
  for (const auto& p : var_to_degree_map) {
    if (p.second > 0) {
      var_to_degree_map_.insert(p);
    } else if (p.second < 0) {
      throw std::logic_error(
          fmt::format("The degree for {} is negative.", p.first.get_name()));
    }
    // Ignore the entry if the degree == 0.
  }
}

Variables PolynomialBasisElement::GetVariables() const {
  Variables vars{};
  for (const auto& p : var_to_degree_map_) {
    vars += p.first;
  }
  return vars;
}

double PolynomialBasisElement::Evaluate(const Environment& env) const {
  return accumulate(
      var_to_degree_map().begin(), var_to_degree_map().end(), 1.0,
      [this, &env](const double v, const std::pair<const Variable, int>& p) {
        const Variable& var{p.first};
        const auto it = env.find(var);
        if (it == env.end()) {
          throw std::invalid_argument(
              fmt::format("Evaluate: {} is not in env", var.get_name()));
        } else {
          return v * this->DoEvaluate(it->second, p.second);
        }
      });
}

bool PolynomialBasisElement::operator==(
    const PolynomialBasisElement& other) const {
  return typeid(*this) == typeid(other) && EqualTo(other);
}

bool PolynomialBasisElement::EqualTo(
    const PolynomialBasisElement& other) const {
  if (var_to_degree_map_.size() != other.var_to_degree_map_.size()) {
    return false;
  }
  for (auto it1 = var_to_degree_map_.begin(),
            it2 = other.var_to_degree_map_.begin();
       it1 != var_to_degree_map_.end(); ++it1, ++it2) {
    const Variable& var1{it1->first};
    const Variable& var2{it2->first};
    const int degree1{it1->second};
    const int degree2{it2->second};
    if (!var1.equal_to(var2) || degree1 != degree2) {
      return false;
    }
  }
  return true;
}

bool PolynomialBasisElement::operator!=(
    const PolynomialBasisElement& other) const {
  return !(*this == other);
}

bool PolynomialBasisElement::lexicographical_compare(
    const PolynomialBasisElement& other) const {
  DRAKE_ASSERT(typeid(*this) == typeid(other));
  return std::lexicographical_compare(
      var_to_degree_map_.begin(), var_to_degree_map_.end(),
      other.var_to_degree_map_.begin(), other.var_to_degree_map_.end(),
      [](const std::pair<const Variable, int>& p1,
         const std::pair<const Variable, int>& p2) {
        const Variable& v1{p1.first};
        const int i1{p1.second};
        const Variable& v2{p2.first};
        const int i2{p2.second};
        if (v1.less(v2)) {
          // m2 does not have the variable v1 explicitly, so we treat it as if
          // it has (v1)⁰. That is, we need "return i1 < 0", but i1 should be
          // positive, so this case always returns false.
          return false;
        }
        if (v2.less(v1)) {
          // m1 does not have the variable v2 explicitly, so we treat it as
          // if it has (v2)⁰. That is, we need "return 0 < i2", but i2 should
          // be positive, so it always returns true.
          return true;
        }
        return i1 < i2;
      });
}
}  // namespace symbolic
}  // namespace drake
