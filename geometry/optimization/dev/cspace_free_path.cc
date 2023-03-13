#include "drake/geometry/optimization/dev/cspace_free_path.h"

namespace drake {
namespace geometry {
namespace optimization {

std::unordered_map<symbolic::Variable, symbolic::Polynomial>
initialize_path_map(CspaceFreePath* cspace_free_path,
                    unsigned int maximum_path_degree) {
  std::unordered_map<symbolic::Variable, symbolic::Polynomial> ret;
  Eigen::Matrix<symbolic::Monomial, Eigen::Dynamic, 1> basis =
      symbolic::MonomialBasis(symbolic::Variables{cspace_free_path->mu_},
                              maximum_path_degree);

  std::size_t i = 0;
  for (const auto& s_set_itr : cspace_free_path->get_s_set()) {
    // construct a dense polynomial
    symbolic::Polynomial::MapType path_monomial_to_coeff;
    for (unsigned int j = 0; j <= maximum_path_degree; ++j) {
      const symbolic::Variable cur_var{fmt::format("s_{}_{}", i, j)};
      path_monomial_to_coeff.emplace(basis(j), symbolic::Expression{cur_var});
    }
    ret.insert(
        {s_set_itr, symbolic::Polynomial(std::move(path_monomial_to_coeff))});
    ++i;
  }
  return ret;
}

CspaceFreePath::CspaceFreePath(const multibody::MultibodyPlant<double>* plant,
                               const geometry::SceneGraph<double>* scene_graph,
                               SeparatingPlaneOrder plane_order,
                               const Eigen::Ref<const Eigen::VectorXd>& q_star,
                               unsigned int maximum_path_degree,
                               const Options& options)
    : CspaceFreePolytope(plant, scene_graph, plane_order, q_star, options),
      mu_(symbolic::Variable("mu")),
      path_(initialize_path_map(this, maximum_path_degree)) {
  this->GeneratePathRationals();
}

void CspaceFreePath::GeneratePathRationals() {
  // plane_geometries_ currently has rationals in terms of the configuration
  // space variable. We replace each of these PlaneSeparatesGeometries with a
  // new one that has rationals in terms of the path variable.
  symbolic::Polynomial::SubstituteAndExpandCacheData cached_substitutions;

  // Add the auxilliary variables for matrix SOS constraints to the substitution
  // map.
  std::unordered_map<symbolic::Variable, symbolic::Polynomial>
      path_with_y_subs = path_;
  for (int i = 0; i < y_slack().size(); ++i) {
    path_with_y_subs.emplace(y_slack()(i), symbolic::Polynomial(y_slack()(i)));
  }

  auto generate_substituted_rationals =
      [&path_with_y_subs, &cached_substitutions](
          const std::vector<symbolic::RationalFunction>& rationals) {
        std::vector<symbolic::RationalFunction> path_rationals;
        path_rationals.reserve(rationals.size());
        for (const auto& rational : rationals) {
          const symbolic::Polynomial path_numerator{
              rational.numerator().SubstituteAndExpand(path_with_y_subs,
                                                       &cached_substitutions)};
          const symbolic::Polynomial path_denominator{
              rational.denominator().SubstituteAndExpand(
                  path_with_y_subs, &cached_substitutions)};
          path_rationals.emplace_back(path_numerator, path_denominator);
        }
        return path_rationals;
      };

  std::vector<PlaneSeparatesGeometries> path_plane_geometries;
  for (const auto& plane_geometry : this->get_mutable_plane_geometries()) {
    path_plane_geometries.emplace_back(
        generate_substituted_rationals(plane_geometry.positive_side_rationals),
        generate_substituted_rationals(plane_geometry.negative_side_rationals),
        plane_geometry.plane_index);
  }
  this->get_mutable_plane_geometries() = std::move(path_plane_geometries);
}

ParametrizdPolynomialPositiveOnUnitInterval::
    ParametrizdPolynomialPositiveOnUnitInterval(
        const symbolic::Polynomial& poly,
        const symbolic::Variable& interval_variable,
        const symbolic::Variables& parameters,
        const std::optional<const symbolic::Variables>& auxillary_variables)
    : p_(poly), parameters_(parameters) {
  int d = poly.Degree(interval_variable);
  const solvers::MathematicalProgram::NonnegativePolynomial type =
      solvers::MathematicalProgram::NonnegativePolynomial::kSos;

  auto make_lambda = [&d, &interval_variable, &type, this]() {
    auto [lambda, Q_lambda] =
        this->psatz_variables_and_psd_constraints_.NewSosPolynomial(
            {interval_variable}, 2 * d, type, "Sl");
    return lambda;
  };

  auto make_nu = [&d, &interval_variable, &type, this]() {
    const int deg{d % 2 == 0 ? std::max(2 * d - 2, 0) : 2 * d};
    auto [nu, Q_nu] = psatz_variables_and_psd_constraints_.NewSosPolynomial(
        {interval_variable}, deg, type, "Sv");
    return nu;
  };
  if(auxillary_variables.has_value())
  {

  }
  else {
    Polynomial lambda{make_lambda()};
    Polynomial nu{make_nu()};
    if (d % 2 == 0) {
      p_ -= lambda + nu * interval_variable *
                         (symbolic::Polynomial(1, {interval_variable}) -
                          interval_variable);
    }
    else {
      p_ -= lambda * interval_variable +
            nu * (symbolic::Polynomial(1, {interval_variable}) -
                  interval_variable);
    }
  }
  // Constructs the multiplier polynomials and their associated Gram matrices as
  // well as the polynomial p_.
//  if (d > 0) {
//    auto [lambda, Q_lambda] =
//        psatz_variables_and_psd_constraints_.NewSosPolynomial(
//            {interval_variable}, 2 * d, type, "Sl");
//    if (d % 2 == 0) {
//      auto [nu, Q_nu] = psatz_variables_and_psd_constraints_.NewSosPolynomial(
//          {interval_variable}, 2 * d - 2, type, "Sv");
//      p_ -= lambda + nu * interval_variable *
//                         (symbolic::Polynomial(1, {interval_variable}) -
//                          interval_variable);
//    } else {
//      auto [nu, Q_nu] = psatz_variables_and_psd_constraints_.NewSosPolynomial(
//          {interval_variable}, 2 * d, type, "Sv");
//      p_ -= lambda * interval_variable +
//            nu * (symbolic::Polynomial(1, {interval_variable}) -
//                  interval_variable);
//    }
//  }
  // TODO(Alexandre.Amice) remove this once we use the auxillary variables.
  if (auxillary_variables.has_value()) {
    std::cout << "has value" << std::endl;
  }
}

void ParametrizdPolynomialPositiveOnUnitInterval::
    AddPositivityConstraintToProgram(solvers::MathematicalProgram* prog,
                                     symbolic::Environment env) {
  DRAKE_DEMAND(env.size() == parameters_.size());
  for (const auto& parameter : parameters_) {
    DRAKE_DEMAND(env.find(parameter) != env.cend());
  }
  prog->AddEqualityConstraintBetweenPolynomials(p_.EvaluatePartial(env),
                                                symbolic::Polynomial());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
