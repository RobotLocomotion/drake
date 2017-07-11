#include "drake/solvers/bilinear_product_util.h"

#include <ostream>
#include <sstream>
#include <unordered_map>

namespace drake {
namespace solvers {
/*
 * Returns the map that maps x(i).get_id() to i.
 */
std::unordered_map<symbolic::Variable::Id, int> ConstructVarToIndexMap(
    const Eigen::Ref<const VectorXDecisionVariable>& x) {
  std::unordered_map<symbolic::Variable::Id, int> map;
  map.reserve(x.rows());
  for (int i = 0; i < x.rows(); ++i) {
    auto it = map.find(x(i).get_id());
    if (it != map.end()) {
      throw std::runtime_error("Input vector contains duplicate variable " +
                               x(i).get_name());
    }
    map.emplace_hint(it, x(i).get_id(), i);
  }
  return map;
}

symbolic::Expression ReplaceBilinearTerms(
    const symbolic::Expression& e,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const MatrixXDecisionVariable>& W) {
  DRAKE_ASSERT(W.rows() == x.rows() && W.cols() == y.rows());
  const std::unordered_map<symbolic::Variable::Id, int> x_to_index_map =
      ConstructVarToIndexMap(x);
  const std::unordered_map<symbolic::Variable::Id, int> y_to_index_map =
      ConstructVarToIndexMap(y);

  // vars contains all the variables in x and y.
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
    std::unordered_map<symbolic::Variable, int, hash_value<symbolic::Variable>> monomial_map;
    int xy_total_degree{0};
    for (const auto& var_power : p.first.get_powers()) {
      monomial_map.emplace(var_power.first, var_power.second);
      xy_total_degree += var_power.second;
    }
    if (xy_total_degree > 2) {
      std::ostringstream oss;
      oss << "The term " << p.first
          << " has degree larger than 2 on the variables";
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
      if (monomial_map.size() == 2) {
        // The monomial is in the form of x * y, namely two different
        // variables multiplying together.
        auto monomial_map_it = monomial_map.begin();
        const symbolic::Variable &var1{monomial_map_it->first};
        ++monomial_map_it;
        const symbolic::Variable &var2{monomial_map_it->first};
        it_x_idx = x_to_index_map.find(var1.get_id());
        if (it_x_idx != x_to_index_map.end()) {
          // var1 is in x.
          it_y_idx = y_to_index_map.find(var2.get_id());
        } else {
          // var1 is in y.
          it_x_idx = x_to_index_map.find(var2.get_id());
          it_y_idx = y_to_index_map.find(var1.get_id());
        }
      } else {
        // The monomial is in the form of x * x, the square of a variable.
        const symbolic::Variable& squared_var{
            monomial_map.begin()->first};
        it_x_idx = x_to_index_map.find(squared_var.get_id());
        it_y_idx = y_to_index_map.find(squared_var.get_id());
      }
      if (it_x_idx == x_to_index_map.end() ||
          it_y_idx == y_to_index_map.end()) {
        // This error would happen, if we ask
        // ReplaceBilinearTerms(x(i) * x(j), x, y, W).
        std::ostringstream oss;
        oss << "Term " << p.first << " is bilinear, but x and y does not have "
                                     "the corresponding variables.";
        throw std::runtime_error(oss.str());
      }
      // w_xy is the symbolic variable representing the bilinear term x * y.
      const symbolic::Variable& w_xy{W(it_x_idx->second, it_y_idx->second)};
      map_replaced.emplace(symbolic::Monomial{w_xy},
                           p.second);
    }
  }
  symbolic::Polynomial poly_replaced{map_replaced};
  return poly_replaced.ToExpression();
}
}  // namespace solvers
}  // namespace drake