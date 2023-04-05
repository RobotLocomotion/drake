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

//        std::cout << rational.numerator().indeterminates() << std::endl;
//        std::cout << path_numerator.indeterminates() << std::endl;
//        std::cout << std::endl;
        path_numerator.SetIndeterminates(indeterminates);
        if (positive_side) {
          positive_side_conditions.emplace_back(path_numerator,
                                                mu,
                                                parameters);
        }
        else {
          negative_side_conditions.emplace_back(path_numerator,
                                                mu,
                                                parameters);
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
      path_(initialize_path_map(this, maximum_path_degree)) {
  this->GeneratePathRationals();
}

void CspaceFreePath::GeneratePathRationals() {
  // plane_geometries_ currently has rationals in terms of the configuration
  // space variable. We create PlaneSeparatesGeometriesOnPath objects which can
  // be used to construct the program once a path is chosen.
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

  for (const auto& plane_geometry : this->get_mutable_plane_geometries()) {
    plane_geometries_on_path_.emplace_back(plane_geometry, mu_,
                                           path_with_y_subs, indeterminates,
                                           &cached_substitutions);
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
