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
namespace {
std::map<Variable, int> ToVarToDegreeMap(
    const Eigen::Ref<const VectorX<Variable>>& vars,
    const Eigen::Ref<const Eigen::VectorXi>& exponents) {
  DRAKE_DEMAND(vars.size() == exponents.size());
  std::map<Variable, int> powers;
  for (int i = 0; i < vars.size(); ++i) {
    if (powers.count(vars[i]) > 0) {
      throw std::invalid_argument(fmt::format(
          "PolynomialBasisElement: {} is repeated", vars[i].get_name()));
    }
    if (exponents[i] > 0) {
      powers.emplace(vars[i], exponents[i]);
    } else if (exponents[i] < 0) {
      throw std::logic_error("The exponent is negative.");
    }
  }
  return powers;
}
}  // namespace

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

PolynomialBasisElement::PolynomialBasisElement(
    const Eigen::Ref<const VectorX<Variable>>& vars,
    const Eigen::Ref<const Eigen::VectorXi>& degrees)
    : PolynomialBasisElement(ToVarToDegreeMap(vars, degrees)) {}

int PolynomialBasisElement::degree(const Variable& v) const {
  auto it = var_to_degree_map_.find(v);
  if (it == var_to_degree_map_.end()) {
    return 0;
  } else {
    return it->second;
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

symbolic::Expression PolynomialBasisElement::ToExpression() const {
  return DoToExpression();
}

void PolynomialBasisElement::DoEvaluatePartial(
    const Environment& env, double* coeff,
    std::map<Variable, int>* new_basis_element) const {
  DRAKE_ASSERT(coeff != nullptr);
  DRAKE_ASSERT(new_basis_element != nullptr);
  DRAKE_ASSERT(new_basis_element->empty());
  *coeff = 1;
  for (const auto& [var, degree] : var_to_degree_map_) {
    auto it = env.find(var);
    if (it != env.end()) {
      *coeff *= DoEvaluate(it->second, degree);
    } else {
      new_basis_element->emplace(var, degree);
    }
  }
}

void PolynomialBasisElement::DoMergeBasisElementInPlace(
    const PolynomialBasisElement& other) {
  DRAKE_ASSERT(typeid(*this) == typeid(other));
  auto it1 = this->var_to_degree_map_.begin();
  auto it2 = other.var_to_degree_map_.begin();
  while (it1 != this->var_to_degree_map_.end() &&
         it2 != other.var_to_degree_map_.end()) {
    if (it1->first.equal_to(it2->first)) {
      it1->second += it2->second;
      it1++;
      it2++;
    } else if (it2->first.less(it1->first)) {
      this->var_to_degree_map_.insert(it1,
                                      std::make_pair(it2->first, it2->second));
      it2++;
    } else {
      // it1->first < it2->first.
      it1++;
    }
  }
  while (it2 != other.var_to_degree_map_.end()) {
    this->var_to_degree_map_.insert(this->var_to_degree_map_.end(),
                                    std::make_pair(it2->first, it2->second));
    it2++;
  }
  total_degree_ += other.total_degree_;
}
}  // namespace symbolic
}  // namespace drake
