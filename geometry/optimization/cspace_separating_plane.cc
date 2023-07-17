#include "drake/geometry/optimization/cspace_separating_plane.h"

#include <utility>

#include "drake/common/symbolic/monomial_util.h"

namespace drake {
namespace geometry {
namespace optimization {

namespace {
template <typename T>
void InitializeCoeffVects(const VectorX<T>& decision_variables,
                          Eigen::Matrix<T, 3, Eigen::Dynamic>* a_coeff,
                          VectorX<T>* b_coeff) {
  static_assert(std::is_same_v<T, symbolic::Variable> ||
                std::is_same_v<T, double>);
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
void MakeCalcPlanePolynomial(const VectorX<symbolic::Variable>& vars_for_plane,
                             const int plane_degree,
                             const Eigen::Matrix<T, 3, Eigen::Dynamic>& a_coeff,
                             const VectorX<T>& b_coeff,
                             Vector3<symbolic::Polynomial>* a_val,
                             symbolic::Polynomial* b_val) {
  static_assert(std::is_same_v<T, symbolic::Variable> ||
                std::is_same_v<T, double>);
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
    const VectorX<double>& values, const int degree) {
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
  VectorX<symbolic::Monomial> basis = symbolic::MonomialBasis(vars, degree);
  Eigen::VectorXd ret{basis.size()};
  for (int i = 0; i < basis.size(); ++i) {
    ret(i) = basis(i).Evaluate(evals);
  }
  return ret;
}

template <typename T1, typename T2, typename T3>
void CalcPlaneImpl(const VectorX<T1>& decision_variables,
                   const VectorX<T2>& vars_for_plane, int plane_degree,
                   Vector3<T3>* a_val, T3* b_val) {
  const int num_vars = vars_for_plane.size();
  const int num_coeffs_per_poly =
      symbolic::NChooseK(num_vars + plane_degree, plane_degree);
  DRAKE_DEMAND(decision_variables.size() == 4 * num_coeffs_per_poly);
  Eigen::Matrix<T1, 3, Eigen::Dynamic> a_coeff(3, num_coeffs_per_poly);
  VectorX<T1> b_coeff(num_coeffs_per_poly);
  InitializeCoeffVects(decision_variables, &a_coeff, &b_coeff);
  if constexpr (std::is_same_v<T3, symbolic::Polynomial>) {
    MakeCalcPlanePolynomial(vars_for_plane, plane_degree, a_coeff, b_coeff,
                            a_val, b_val);
  } else {
    const Eigen::VectorXd evaluated_powers =
        ComputeGradedRevLexEvaluatedPowers(vars_for_plane, plane_degree);
    (*a_val) = a_coeff * evaluated_powers;
    (*b_val) = b_coeff.transpose() * evaluated_powers;
  }
}

}  // namespace

SeparatingPlaneOrder ToPlaneOrder(int plane_degree) {
  if (plane_degree == 1) {
    return SeparatingPlaneOrder::kAffine;
  } else {
    throw std::runtime_error(fmt::format(
        "ToPlaneOrder: plane_degree={}, only supports plane_degree = 1.",
        plane_degree));
  }
}

int ToPlaneDegree(SeparatingPlaneOrder plane_order) {
  switch (plane_order) {
    case SeparatingPlaneOrder::kAffine: {
      return 1;
    }
  }
  DRAKE_UNREACHABLE();
}

namespace internal {

void CalcPlane(const VectorX<symbolic::Variable>& decision_variables,
               const VectorX<symbolic::Variable>& vars_for_plane,
               int plane_degree, Vector3<symbolic::Polynomial>* a_val,
               symbolic::Polynomial* b_val) {
  CalcPlaneImpl(decision_variables, vars_for_plane, plane_degree, a_val, b_val);
}

void CalcPlane(const VectorX<double>& decision_variables,
               const VectorX<symbolic::Variable>& vars_for_plane,
               int plane_degree, Vector3<symbolic::Polynomial>* a_val,
               symbolic::Polynomial* b_val) {
  CalcPlaneImpl(decision_variables, vars_for_plane, plane_degree, a_val, b_val);
}

void CalcPlane(const VectorX<double>& decision_variables,
               const VectorX<double>& vars_for_plane, int plane_degree,
               Vector3<double>* a_val, double* b_val) {
  CalcPlaneImpl(decision_variables, vars_for_plane, plane_degree, a_val, b_val);
}

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
