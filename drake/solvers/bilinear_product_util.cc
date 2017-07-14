#include "drake/solvers/bilinear_product_util.h"

#include <ostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace drake {
namespace solvers {
/*
 * Returns the map that maps x(i).get_id() to i.
 */

using std::ostringstream;
using std::runtime_error;
using std::unordered_map;

using symbolic::Expression;
using symbolic::Monomial;
using symbolic::Variable;
using symbolic::Variables;

using MapVarToIndex = unordered_map<Variable::Id, int>;

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
    const Eigen::Ref<const MatrixXDecisionVariable>& W) {
  DRAKE_ASSERT(W.rows() == x.rows() && W.cols() == y.rows());
  const MapVarToIndex x_to_index_map = ConstructVarToIndexMap(x);
  const MapVarToIndex y_to_index_map = ConstructVarToIndexMap(y);

  // p_bilinear is a polynomial with x and y as indeterminates.
  const symbolic::Polynomial p_bilinear{e, Variables{x} + Variables{y}};

  const auto& map_bilinear = p_bilinear.monomial_to_coefficient_map();
  symbolic::Polynomial::MapType map_replaced;
  map_replaced.reserve(map_bilinear.size());
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
      const auto it = map_replaced.find(p.first);
      if (it != map_replaced.end()) {
        it->second += p.second;
      } else {
        map_replaced.emplace_hint(it, p.first, p.second);
      }
    } else {
      // This monomial contains bilinear term in x and y.
      MapVarToIndex::const_iterator it_x_idx;
      MapVarToIndex::const_iterator it_y_idx;
      if (monomial_map.size() == 2) {
        // The monomial is in the form of x * y, namely two different
        // variables multiplying together.
        auto monomial_map_it = monomial_map.begin();
        const auto var1_id{monomial_map_it->first};
        ++monomial_map_it;
        const auto var2_id{monomial_map_it->first};
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
        const auto squared_var_id{monomial_map.begin()->first};
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
      // w_xy is the symbolic variable representing the bilinear term x * y.
      const Variable& w_xy{W(it_x_idx->second, it_y_idx->second)};
      // TODO(soonho.kong): will improve the code below, such that we do not add
      // the monomial `w_xy`.
      map_replaced.emplace(Monomial{w_xy}, p.second);
    }
  }
  Expression e_replaced{0};
  for (const auto& p_replaced : map_replaced) {
    e_replaced += p_replaced.first.ToExpression() * p_replaced.second;
  }
  return e_replaced;
}
}  // namespace solvers
}  // namespace drake
