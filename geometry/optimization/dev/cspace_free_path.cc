#include "drake/geometry/optimization/dev/cspace_free_path.h"

#include <vector>
#include <iostream>
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

PlaneSeparatesGeometriesOnPath::PlaneSeparatesGeometriesOnPath(
    const PlaneSeparatesGeometries& plane_geometries,
    const symbolic::Variable& mu,
    const std::unordered_map<symbolic::Variable, symbolic::Polynomial>&
        path_with_y_subs,
    const symbolic::Variables& indeterminates,
    symbolic::Polynomial::SubstituteAndExpandCacheData* cached_substitutions)
    : plane_index{plane_geometries.plane_index} {
  auto substitute_and_create_condition =
      [this, &cached_substitutions, &path_with_y_subs, &indeterminates, &mu](
          const symbolic::RationalFunction& rational, bool positive_side) {
        symbolic::Variables parameters;
        for (const auto& var : rational.numerator().indeterminates()) {
          parameters.insert(path_with_y_subs.at(var).decision_variables());
        }
        symbolic::Polynomial path_numerator{
            rational.numerator().SubstituteAndExpand(path_with_y_subs,
                                                     cached_substitutions)};

        // The current y_slacks along with mu.
        symbolic::Variables cur_indeterminates{
            intersect(indeterminates, rational.numerator().indeterminates())};
        cur_indeterminates.insert(mu);

        path_numerator.SetIndeterminates(cur_indeterminates);
        if (positive_side) {
          positive_side_conditions.emplace_back(path_numerator, mu, parameters);
        } else {
          negative_side_conditions.emplace_back(path_numerator, mu, parameters);
        }
      };

  for (const auto& rational : plane_geometries.positive_side_rationals) {
    substitute_and_create_condition(rational, true);
  }
  for (const auto& rational : plane_geometries.negative_side_rationals) {
    substitute_and_create_condition(rational, false);
  }
}

CspaceFreePath::CspaceFreePath(const multibody::MultibodyPlant<double>* plant,
                               const geometry::SceneGraph<double>* scene_graph,
                               SeparatingPlaneOrder plane_order,
                               const Eigen::Ref<const Eigen::VectorXd>& q_star,
                               unsigned int maximum_path_degree,
                               const Options& options)
    : CspaceFreePolytope(plant, scene_graph, plane_order, q_star, options),
      mu_(symbolic::Variable("mu")),
      max_degree_(static_cast<int>(maximum_path_degree)),
      path_(initialize_path_map(this, maximum_path_degree)) {
  this->GeneratePathRationals();
}

void CspaceFreePath::GeneratePathRationals() {
  // plane_geometries_ currently has rationals in terms of the configuration
  // space variable. We create PlaneSeparatesGeometriesOnPath objects which are
  // in terms of the path variable and can be used to construct the
  // certification program once a path is chosen.
  symbolic::Polynomial::SubstituteAndExpandCacheData cached_substitutions;

  // Add the auxilliary variables for matrix SOS constraints to the substitution
  // map.
  std::unordered_map<symbolic::Variable, symbolic::Polynomial>
      path_with_y_subs = path_;
  symbolic::Variables indeterminates{mu_};
  for (int i = 0; i < y_slack().size(); ++i) {
    path_with_y_subs.emplace(y_slack()(i), symbolic::Polynomial(y_slack()(i)));
    indeterminates.insert(y_slack()(i));
  }

  for (const auto& plane_geometry : this->get_plane_geometries()) {
    plane_geometries_on_path_.emplace_back(plane_geometry, mu_,
                                           path_with_y_subs, indeterminates,
                                           &cached_substitutions);
  }
}

//[[nodiscard]] CspaceFreePolytope::SeparationCertificateProgram
//CspaceFreePath::MakeIsGeometrySeparableOnPathProgram(
//    const SortedPair<geometry::GeometryId>& geometry_pair,
//    const std::unordered_map<const symbolic::Variable,
//                             const Polynomial<double>>& path) const {
//  // Fail fast as building the programs can be expensive.
//  for (const auto& [var, cur_path] : path) {
//    DRAKE_DEMAND(cur_path.is_univariate());
//    DRAKE_DEMAND(cur_path.GetDegree() <= static_cast<int>(max_degree_));
//  }
//
//  int plane_index{get_separating_plane_index(geometry_pair)};
//  if (plane_index < 0) {
//    throw std::runtime_error(fmt::format(
//        "GetIsGeometrySeparableProgram(): geometry pair ({}, {}) does not need "
//        "a separation certificate",
//        get_scene_graph().model_inspector().GetName(geometry_pair.first()),
//        get_scene_graph().model_inspector().GetName(geometry_pair.second())));
//  }
//
//  return ConstructPlaneSearchProgramOnPath(
//      plane_geometries_on_path_.at(plane_index), path);
//}

[[nodiscard]] CspaceFreePolytope::SeparationCertificateProgram
CspaceFreePath::MakeIsGeometrySeparableOnPathProgram(
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const VectorX<Polynomiald>& path) const {
  // Fail fast as building the program can be expensive.
  int plane_index{get_separating_plane_index(geometry_pair)};
  if (plane_index < 0) {
    throw std::runtime_error(fmt::format(
        "GetIsGeometrySeparableProgram(): geometry pair ({}, {}) does not need "
        "a separation certificate",
        get_scene_graph().model_inspector().GetName(geometry_pair.first()),
        get_scene_graph().model_inspector().GetName(geometry_pair.second())));
  }

  DRAKE_DEMAND(rational_forward_kin().s().rows() == path.rows());
  std::unordered_map<symbolic::Variable, Polynomial<double>> cspace_var_to_path;
  for (int i = 0; i < path.rows(); ++i) {
    DRAKE_DEMAND(path(i).is_univariate());
    DRAKE_DEMAND(path(i).GetDegree() <= static_cast<int>(max_degree_));
    cspace_var_to_path.emplace(rational_forward_kin().s()(i), path(i));
  }

  return ConstructPlaneSearchProgramOnPath(
      plane_geometries_on_path_.at(plane_index), cspace_var_to_path);
}

[[nodiscard]] CspaceFreePolytope::SeparationCertificateProgram
CspaceFreePath::ConstructPlaneSearchProgramOnPath(
    const PlaneSeparatesGeometriesOnPath& plane_geometries_on_path,
    const std::unordered_map<symbolic::Variable,
                             Polynomial<double>>& path) const {
  SeparationCertificateProgram ret;

  // construct the parameter to value map
  symbolic::Environment param_eval_map;
  for (const auto& [config_space_var, eval_path] : path) {
    const symbolic::Polynomial symbolic_path{path_.at(config_space_var)};
    const std::vector<Polynomial<double>::Monomial> monomials =
        eval_path.GetMonomials();
    for (const auto& [mu_monom, mu_var_coeff] :
         symbolic_path.monomial_to_coefficient_map()) {
      // Find the monomial with the matching degree. If it doesn't exist
      // evaluate it to 0.
      const auto evaled_monom_iter = std::find_if(
          monomials.begin(), monomials.end(), [&mu_monom](const auto& m) {
            return m.GetDegree() == mu_monom.total_degree();
          });
      const double mu_var_coeff_eval{evaled_monom_iter == monomials.end()
                                         ? 0
                                         : evaled_monom_iter->coefficient};
      param_eval_map.insert(*mu_var_coeff.GetVariables().begin(),
                            mu_var_coeff_eval);
    }
  }

  // Now add the separation conditions to the program
  for (const auto& condition :
       plane_geometries_on_path.positive_side_conditions) {
    condition.AddPositivityConstraintToProgram(param_eval_map, ret.prog.get());
  }
  for (const auto& condition :
       plane_geometries_on_path.negative_side_conditions) {
    condition.AddPositivityConstraintToProgram(param_eval_map, ret.prog.get());
  }

  return ret;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
