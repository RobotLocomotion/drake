#include "drake/geometry/optimization/cspace_separating_plane.h"

#include <utility>

#include "drake/common/symbolic/monomial_util.h"

namespace drake {
namespace geometry {
namespace optimization {

namespace {
template <typename T>
// typename std::enable_if_t<
//              std::is_same_v<typename T::value, symbolic::Variable> ||
//              std::is_same_v<typename T::value, double>>
void InitializeCoeffVects(const VectorX<T>& decision_variables,
                          Eigen::Matrix<T, 3, Eigen::Dynamic>* a_coeff,
                          VectorX<T>* b_coeff) {
  const int num_coeffs_per_poly = decision_variables.size() / 4;
  int var_count = 0;
  for (int i = 0; i < 3; ++i) {
    a_coeff->row(i) =
        decision_variables.segment(var_count, num_coeffs_per_poly);
    var_count += num_coeffs_per_poly;
  }
  *b_coeff = decision_variables.segment(var_count, num_coeffs_per_poly);
  var_count += num_coeffs_per_poly;
  DRAKE_DEMAND(var_count == decision_variables.size());
}

template <typename T>
void MakeCalcPlanePolynomial(
    const VectorX<symbolic::Variable>& vars_for_plane, const int plane_degree,
    const Eigen::MatrixBase<Eigen::Matrix<T, 3, Eigen::Dynamic>>& a_coeff,
    const Eigen::MatrixBase<VectorX<T>>& b_coeff,
    Vector3<symbolic::Polynomial>* a_val, symbolic::Polynomial* b_val) {
  const Eigen::Matrix<symbolic::Monomial, Eigen::Dynamic, 1> basis =
      symbolic::MonomialBasis(symbolic::Variables{vars_for_plane},
                              plane_degree);
  for (int i = 0; i < 3; ++i) {
    symbolic::Polynomial::MapType monomial_to_coeff_map;
    for (int j = 0; j < basis.size(); ++j) {
      monomial_to_coeff_map.emplace(basis(j), a_coeff(i, j));
    }
    (*a_val)(i) = symbolic::Polynomial(monomial_to_coeff_map);
  }
  symbolic::Polynomial::MapType monomial_to_coeff_map;
  for (int j = 0; j < basis.size(); ++j) {
    monomial_to_coeff_map.emplace(basis(j), b_coeff(j));
  }
  *b_val = symbolic::Polynomial(monomial_to_coeff_map);
}

 Eigen::VectorXd ComputeGradedRevLexEvaluatedPowers(
    const Eigen::Ref<const VectorX<double>>& values, const int degree) {
  const int num_vars = values.size();
  symbolic::Variables vars;
  symbolic::Environment evals;
  for (int i = 0; i < num_vars; ++i) {
    const symbolic::Variable cur_var{fmt::format("x{}", i)};
    vars.insert(cur_var);
    evals.insert(cur_var, values(i));
  }
  // Make a grevlex basis that we can now evaluate with the desired
  // values.
  Eigen::VectorX<symbolic::Monomial> basis =
      symbolic::MonomialBasis(vars, degree);
  Eigen::VectorXd ret{basis.size()};
  for (int i = 0; i < basis.size(); ++i) {
    ret(i) = basis(i).Evaluate(evals);
  }
  return ret;
}

}  // namespace

void CalcPlane(const VectorX<symbolic::Variable>& decision_variables,
               const VectorX<symbolic::Variable>& vars_for_plane,
               int plane_degree, Vector3<symbolic::Polynomial>* a_val,
               symbolic::Polynomial* b_val) {
  const int num_vars = vars_for_plane.size();
  const int num_coeffs_per_poly =
      symbolic::NChooseK(num_vars + plane_degree, plane_degree);
  DRAKE_DEMAND(decision_variables.size() == 4 * num_coeffs_per_poly);
  Eigen::Matrix<symbolic::Variable, 3, Eigen::Dynamic> a_coeff(
      3, num_coeffs_per_poly);
  VectorX<symbolic::Variable> b_coeff(num_coeffs_per_poly);
  InitializeCoeffVects(decision_variables, &a_coeff, &b_coeff);
  MakeCalcPlanePolynomial(vars_for_plane, plane_degree, a_coeff, b_coeff, a_val,
                          b_val);
}

void CalcPlane(const VectorX<double>& decision_variables,
               const VectorX<symbolic::Variable>& vars_for_plane,
               int plane_degree, Vector3<symbolic::Polynomial>* a_val,
               symbolic::Polynomial* b_val) {
  const int num_vars = vars_for_plane.size();
  const int num_coeffs_per_poly =
      symbolic::NChooseK(num_vars + plane_degree, plane_degree);
  DRAKE_DEMAND(decision_variables.size() == 4 * num_coeffs_per_poly);
  Eigen::Matrix<double, 3, Eigen::Dynamic> a_coeff(3,num_coeffs_per_poly);
  VectorX<double> b_coeff(num_coeffs_per_poly);
  InitializeCoeffVects(decision_variables, &a_coeff, &b_coeff);
  MakeCalcPlanePolynomial(vars_for_plane, plane_degree, a_coeff, b_coeff, a_val,
                          b_val);
}

 void CalcPlane(const VectorX<double>& decision_variables,
               const VectorX<double>& vars_for_plane, int plane_degree,
               Vector3<double>* a_val, double* b_val) {
  const int num_vars = vars_for_plane.size();
  const int num_coeffs_per_poly =
      symbolic::NChooseK(num_vars + plane_degree, plane_degree);
  DRAKE_DEMAND(decision_variables.size() == 4 * num_coeffs_per_poly);
  Eigen::Matrix<double, 3, Eigen::Dynamic> a_coeff(3,num_coeffs_per_poly);
  VectorX<double> b_coeff(num_coeffs_per_poly);
  InitializeCoeffVects(decision_variables, &a_coeff, &b_coeff);
  const Eigen::VectorXd evaluated_powers =
      ComputeGradedRevLexEvaluatedPowers(vars_for_plane, plane_degree);
  (*a_val) = a_coeff * evaluated_powers;
  (*b_val) = b_coeff.transpose() * evaluated_powers;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
