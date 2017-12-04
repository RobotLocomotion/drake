#include "drake/solvers/bilinear_product_util.h"

#include <ostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace drake {
namespace solvers {
using std::ostringstream;
using std::runtime_error;
using std::unordered_map;

using symbolic::Expression;
using symbolic::Monomial;
using symbolic::Variable;
using symbolic::Variables;

using MapVarToIndex = unordered_map<Variable::Id, int>;

/*
 * Returns the map that maps x(i).get_id() to i.
 */
MapVarToIndex ConstructVarToIndexMap(
    const Eigen::Ref<const VectorXDecisionVariable>& x) {
  MapVarToIndex map;
  map.reserve(x.rows());
  for (int i = 0; i < x.rows(); ++i) {
    const auto it = map.find(x(i).get_id());
    if (it != map.end()) {
      throw runtime_error("Input vector contains duplicate variable " +
                               x(i).get_name());
    }
    map.emplace_hint(it, x(i).get_id(), i);
  }
  return map;
}

Expression ReplaceBilinearTerms(
    const Expression& e,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& W) {
  DRAKE_ASSERT(W.rows() == x.rows() && W.cols() == y.rows());
  const MapVarToIndex x_to_index_map = ConstructVarToIndexMap(x);
  const MapVarToIndex y_to_index_map = ConstructVarToIndexMap(y);

  // p_bilinear is a polynomial with x and y as indeterminates.
  const Variables variables_in_x_y{Variables{x} + Variables{y}};
  const symbolic::Polynomial p_bilinear{e, variables_in_x_y};
  const auto& map_bilinear = p_bilinear.monomial_to_coefficient_map();
  symbolic::Polynomial poly;
  for (const auto& p : map_bilinear) {
    MapVarToIndex monomial_map;
    const int monomial_degree = p.first.total_degree();
    for (const auto& var_power : p.first.get_powers()) {
      monomial_map.emplace(var_power.first.get_id(), var_power.second);
    }
    if (monomial_degree > 2) {
      ostringstream oss;
      oss << "The term " << p.first
          << " has degree larger than 2 on the variables";
      throw runtime_error(oss.str());
    } else if (monomial_degree < 2) {
      // Only linear or constant terms, do not need to replace the variables.
      poly.AddProduct(p.second, p.first);
    } else {
      // This monomial contains bilinear term in x and y.
      MapVarToIndex::const_iterator it_x_idx;
      MapVarToIndex::const_iterator it_y_idx;
      if (monomial_map.size() == 2) {
        // The monomial is in the form of x * y, namely two different
        // variables multiplying together.
        auto monomial_map_it = monomial_map.begin();
        const Variable::Id var1_id{monomial_map_it->first};
        ++monomial_map_it;
        const Variable::Id var2_id{monomial_map_it->first};
        it_x_idx = x_to_index_map.find(var1_id);
        if (it_x_idx != x_to_index_map.end()) {
          // var1 is in x.
          it_y_idx = y_to_index_map.find(var2_id);
        } else {
          // var1 is in y.
          it_x_idx = x_to_index_map.find(var2_id);
          it_y_idx = y_to_index_map.find(var1_id);
        }
      } else {
        // The monomial is in the form of x * x, the square of a variable.
        const Variable::Id squared_var_id{monomial_map.begin()->first};
        it_x_idx = x_to_index_map.find(squared_var_id);
        it_y_idx = y_to_index_map.find(squared_var_id);
      }
      if (it_x_idx == x_to_index_map.end() ||
          it_y_idx == y_to_index_map.end()) {
        // This error would happen, if we ask
        // ReplaceBilinearTerms(x(i) * x(j), x, y, W).
        ostringstream oss;
        oss << "Term " << p.first << " is bilinear, but x and y does not have "
                                     "the corresponding variables.";
        throw runtime_error(oss.str());
      }
      // w_xy_expr is the symbolic expression representing the bilinear term x *
      // y.
      const symbolic::Expression& w_xy_expr{
          W(it_x_idx->second, it_y_idx->second)};
      if (is_variable(w_xy_expr)) {
        const symbolic::Variable w_xy = symbolic::get_variable(w_xy_expr);
        if (variables_in_x_y.include(w_xy)) {
          // Case: w_xy is an indeterminate.
          poly.AddProduct(p.second, symbolic::Monomial{w_xy});
        } else {
          // Case: w_xy is a decision variable.
          poly.AddProduct(w_xy * p.second, symbolic::Monomial{});
        }
      } else {
        if (intersect(w_xy_expr.GetVariables(), variables_in_x_y).size() != 0) {
          // w_xy_expr contains a variable in x or y.
          ostringstream oss;
          oss << "W(" + std::to_string(it_x_idx->second) + "," +
                     std::to_string(it_y_idx->second) + ")="
              << w_xy_expr << "contains variables in x or y.";
          throw std::runtime_error(oss.str());
        }
        poly.AddProduct(w_xy_expr * p.second, symbolic::Monomial{});
      }
    }
  }
  return poly.ToExpression();
}
}  // namespace solvers
}  // namespace drake
