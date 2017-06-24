#include "drake/solvers/bilinear_product_util.h"

#include <ostream>
#include <sstream>
namespace drake {
namespace solvers {
std::unordered_map<symbolic::Variable::Id, int> ConstructVarToIndexMap(const Eigen::Ref<const VectorXDecisionVariable>& x) {
  std::unordered_map<symbolic::Variable::Id, int> map;
  map.reserve(x.rows());
  for (int i = 0; i < x.rows(); ++i) {
    auto it = map.find(x(i).get_id());
    if (it != map.end()) {
      throw std::runtime_error("Input vector contains duplicate variable " + x(i).get_name());
    }
    map.emplace_hint(it, x(i).get_id(), i);
  }
  return map;
};

symbolic::Expression ReplaceBilinearTerms(
    const symbolic::Expression& e,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const MatrixXDecisionVariable>& W) {
  DRAKE_ASSERT(W.rows() == x.rows() && W.cols() == y.rows());
  const std::unordered_map<symbolic::Variable::Id, int> x_to_index_map = ConstructVarToIndexMap(x);
  const std::unordered_map<symbolic::Variable::Id, int> y_to_index_map = ConstructVarToIndexMap(y);

  symbolic::Variables vars{};
  for (int i = 0; i < x.rows(); ++i) {
    vars.insert(x(i));
  }
  for (int i = 0; i < y.rows(); ++i) {
    vars.insert(y(i));
  }
  symbolic::Polynomial p_bilinear(e, vars);

  auto map_bilinear = p_bilinear.monomial_to_coefficient_map();
  symbolic::Polynomial::MapType map_replaced;
  map_replaced.reserve(map_bilinear.size());
  for (const auto& p : map_bilinear) {
    std::map<symbolic::Variable, int> monomial_replace_map;
    // For each monomial, we decompose it into two parts, the part that contains
    // variables in `x` or `y`, and the part that does not. We call the first
    // part as "dependent monomial", the second part as "independent monomial".
    std::map<symbolic::Variable, int> dependent_monomial_map;
    std::map<symbolic::Variable, int> independent_monomial_map;
    int xy_total_degree{0};
    for (const auto& var_power : p.first.get_powers()) {
      const symbolic::Variable& var{var_power.first};
      if (x_to_index_map.find(var.get_id()) != x_to_index_map.end() || y_to_index_map.find(var.get_id()) != y_to_index_map.end()) {
        dependent_monomial_map.emplace(var_power.first, var_power.second);
        xy_total_degree += var_power.second;
      } else {
        independent_monomial_map.emplace(var_power.first, var_power.second);
      }
    }
    if (xy_total_degree > 2) {
      std::ostringstream oss;
      oss << "The term " << p.first << " has degree larger than 2 on the variables";
      throw std::runtime_error(oss.str());
    } else if (xy_total_degree < 2) {
      // Only linear or constant terms, do not need to replace the variables.
      auto it = map_replaced.find(p.first);
      if (it != map_replaced.end()) {
        it->second += p.second;
      } else {
        map_replaced.emplace_hint(it, p.first, p.second);
      }
    } else {
      // This monomial contains bilinear term in x and y.
      std::unordered_map<symbolic::Variable::Id, int>::const_iterator it_x_idx;
      std::unordered_map<symbolic::Variable::Id, int>::const_iterator it_y_idx;
      if (dependent_monomial_map.size() == 2) {
        // The dependent monomial is in the form of x * y, namely two different
        // variables multiplying together.
        auto dependent_monomial_map_it = dependent_monomial_map.begin();
        const symbolic::Variable &var1{dependent_monomial_map_it->first};
        ++dependent_monomial_map_it;
        const symbolic::Variable &var2{dependent_monomial_map_it->first};
        it_x_idx = x_to_index_map.find(var1.get_id());
        if (it_x_idx != x_to_index_map.end()) {
          it_y_idx = y_to_index_map.find(var2.get_id());
        } else {
          it_x_idx = x_to_index_map.find(var2.get_id());
          it_y_idx = y_to_index_map.find(var1.get_id());
        }
      } else {
        // The dependent monomial is in the form of x * x, the square of a variable.
        const symbolic::Variable& squared_var{dependent_monomial_map.begin()->first};
        it_x_idx = x_to_index_map.find(squared_var.get_id());
        it_y_idx = y_to_index_map.find(squared_var.get_id());
      }
      if (it_x_idx == x_to_index_map.end() || it_y_idx == y_to_index_map.end()) {
        std::ostringstream oss;
        oss << "Term " << p.first << " is bilinear, but x and y does not have the corresponding variables.";
        throw std::runtime_error(oss.str());
      }
      // w_xy is the symbolic variable representing the bilinear term x * y.
      const symbolic::Variable& w_xy{W(it_x_idx->second, it_y_idx->second)};
      // Multiply w_xy with the independent monomial
      auto it_w_in_independent_monomial = independent_monomial_map.find(w_xy);
      if (it_w_in_independent_monomial != independent_monomial_map.end()) {
        ++(it_w_in_independent_monomial->second);
      } else {
        independent_monomial_map.emplace_hint(it_w_in_independent_monomial, w_xy, 1);
      }
      map_replaced.emplace(symbolic::Monomial{independent_monomial_map}, p.second);
    }
  }
  symbolic::Expression poly_replaced{0};
  for (const auto& p_replaced : map_replaced) {
    poly_replaced += p_replaced.first.ToExpression() * p_replaced.second;
  }
  return poly_replaced;
}
}  // namespace solvers
}  // namespace drake