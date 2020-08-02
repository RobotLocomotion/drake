// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.

#include <fmt/format.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
PolynomialBasis::PolynomialBasis(
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

Variables PolynomialBasis::GetVariables() const {
  Variables vars{};
  for (const auto& p : var_to_degree_map_) {
    vars += p.first;
  }
  return vars;
}

double PolynomialBasis::Evaluate(const Environment& env) const {
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

bool PolynomialBasis::operator==(const PolynomialBasis& other) const {
  return typeid(*this) == typeid(other) &&
         this->var_to_degree_map() == other.var_to_degree_map();
}

bool PolynomialBasis::operator!=(const PolynomialBasis& other) const {
  return !(*this == other);
}
}  // namespace symbolic
}  // namespace drake
