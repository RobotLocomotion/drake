#include "drake/geometry/optimization/dev/cspace_free_path.h"

#include "drake/common/symbolic/monomial_util.h"

namespace drake {
namespace geometry {
namespace optimization {

namespace {}

CspaceFreePath::CspaceFreePath(const multibody::MultibodyPlant<double>* plant,
                               const geometry::SceneGraph<double>* scene_graph,
                               SeparatingPlaneOrder plane_order,
                               const Eigen::Ref<const Eigen::VectorXd>& q_star,
                               unsigned int maximum_path_degree,
                               const Options& options)
    : CspaceFreePolytope(plant, scene_graph, plane_order, q_star, options),
      mu_(symbolic::Variable("mu")),
      path_(([maximum_path_degree,
              this]() -> std::unordered_map<symbolic::Variable,
                                            symbolic::Polynomial> {
        std::unordered_map<symbolic::Variable, symbolic::Polynomial> ret;
        Eigen::Matrix<symbolic::Monomial, Eigen::Dynamic, 1> basis =
            symbolic::MonomialBasis(symbolic::Variables{mu_},
                                    maximum_path_degree);

        std::size_t i = 0;
        for (const auto& s_set_itr : this->get_s_set()) {
          // construct a dense polynomial
          symbolic::Polynomial::MapType path_monomial_to_coeff;
          for (unsigned int j = 0; j < maximum_path_degree; ++j) {
            const symbolic::Variable cur_var{fmt::format("s_{}_{}", i, j)};
            path_monomial_to_coeff.emplace(basis(j),
                                           symbolic::Expression{cur_var});
          }
          ret.insert({s_set_itr, symbolic::Polynomial(path_monomial_to_coeff)});
          ++i;
        }
        return ret;
      })()) {
  this->GeneratePathRationals();
}

void CspaceFreePath::GeneratePathRationals() {
  // plane_geometries_ currently has rationals in terms of the configuration
  // space variable. We replace each of these PlaneSeparatesGeometries with a
  // new one that has rationals in terms of the path variable.
  //  auto substitute_path_
  std::map<symbolic::Monomial, symbolic::Polynomial,
           symbolic::internal::CompareMonomial>
      cached_substitutions;

  // add the auxilliary variables for matrix SOS constraints to the substitution
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
  for (const auto& plane_geometry : plane_geometries_) {
    path_plane_geometries.emplace_back(
        generate_substituted_rationals(plane_geometry.positive_side_rationals),
        generate_substituted_rationals(plane_geometry.negative_side_rationals),
        plane_geometry.plane_index);
  }
  plane_geometries_ = std::move(path_plane_geometries);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
