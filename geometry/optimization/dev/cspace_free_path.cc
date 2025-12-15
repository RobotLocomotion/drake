#include "drake/geometry/optimization/dev/cspace_free_path.h"

#include <vector>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/geometry/optimization/cspace_free_internal.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

namespace drake {
namespace geometry {
namespace optimization {
std::unordered_map<symbolic::Variable, symbolic::Polynomial>
initialize_path_map(
    CspaceFreePath* cspace_free_path, int maximum_path_degree,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& s_variables) {
  std::unordered_map<symbolic::Variable, symbolic::Polynomial> ret;
  const VectorX<symbolic::Monomial> basis = symbolic::MonomialBasis(
      symbolic::Variables{cspace_free_path->mu_}, maximum_path_degree);

  for (int i = 0; i < s_variables.size(); ++i) {
    const symbolic::Variable s{s_variables(i)};
    // construct a dense polynomial
    symbolic::Polynomial::MapType path_monomial_to_coeff;
    for (int j = 0; j <= maximum_path_degree; ++j) {
      const symbolic::Variable cur_var{fmt::format("s_{}(mu_{})_coeff", i, j)};
      path_monomial_to_coeff.emplace(basis(j), symbolic::Expression{cur_var});
    }
    ret.insert({s, symbolic::Polynomial(std::move(path_monomial_to_coeff))});
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

CspaceFreePath::SeparationCertificateResult::~SeparationCertificateResult() =
    default;

CspaceFreePath::SeparationCertificateProgram::~SeparationCertificateProgram() =
    default;

CspaceFreePath::CspaceFreePath(const multibody::MultibodyPlant<double>* plant,
                               const geometry::SceneGraph<double>* scene_graph,
                               const Eigen::Ref<const Eigen::VectorXd>& q_star,
                               int maximum_path_degree, int plane_degree)
    : rational_forward_kin_(plant),
      scene_graph_{DRAKE_DEREF(scene_graph)},
      q_star_{q_star},
      link_geometries_{internal::GetCollisionGeometries(*plant, *scene_graph)},
      plane_degree_{plane_degree},
      mu_(symbolic::Variable("mu")),
      max_degree_(maximum_path_degree),
      path_(initialize_path_map(this, maximum_path_degree,
                                rational_forward_kin_.s())) {
  // collision_pairs maps each pair of body to the pair of collision geometries
  // on that pair of body.
  std::map<SortedPair<multibody::BodyIndex>,
           std::vector<std::pair<const CIrisCollisionGeometry*,
                                 const CIrisCollisionGeometry*>>>
      collision_pairs;
  int num_collision_pairs = internal::GenerateCollisionPairs(
      rational_forward_kin_.plant(), scene_graph_, link_geometries_,
      &collision_pairs);

  const int num_coeffs_per_poly = plane_degree + 1;
  separating_planes_.reserve(num_collision_pairs);
  for (const auto& [link_pair, geometry_pairs] : collision_pairs) {
    for (const auto& geometry_pair : geometry_pairs) {
      // Generate the separating plane for this collision pair.
      Vector3<symbolic::Polynomial> a;
      symbolic::Polynomial b;
      VectorX<symbolic::Variable> plane_decision_vars{4 * num_coeffs_per_poly};
      for (int i = 0; i < plane_decision_vars.rows(); ++i) {
        plane_decision_vars(i) =
            symbolic::Variable(fmt::format("plane_var{}", i));
      }
      CalcPlane<symbolic::Variable, symbolic::Variable, symbolic::Polynomial>(
          plane_decision_vars, Vector1<symbolic::Variable>(mu_), plane_degree,
          &a, &b);

      // Compute the expressed body for this plane
      const multibody::BodyIndex expressed_body =
          multibody::internal::FindBodyInTheMiddleOfChain(
              rational_forward_kin_.plant(), link_pair.first(),
              link_pair.second());
      separating_planes_.emplace_back(a, b, geometry_pair.first,
                                      geometry_pair.second, expressed_body,
                                      plane_degree_, plane_decision_vars);

      map_geometries_to_separating_planes_.emplace(
          SortedPair<geometry::GeometryId>(geometry_pair.first->id(),
                                           geometry_pair.second->id()),
          static_cast<int>(separating_planes_.size()) - 1);
    }
  }

  for (int i = 0; i < 3; ++i) {
    y_slack_(i) = symbolic::Variable("y" + std::to_string(i));
  }

  std::vector<std::unique_ptr<CSpaceSeparatingPlane<symbolic::Variable>>>
      separating_planes_ptrs;
  separating_planes_ptrs.reserve(separating_planes_.size());
  for (const auto& plane : separating_planes_) {
    separating_planes_ptrs.push_back(
        std::make_unique<CSpaceSeparatingPlane<symbolic::Variable>>(plane));
  }
  // Generate the rationals for the separating planes. At this point, the plane
  // components are a function of mu, but the plane_geometries will still be in
  // terms of the s variable.
  std::vector<PlaneSeparatesGeometries> plane_geometries;
  internal::GenerateRationals(separating_planes_ptrs, y_slack_, q_star_,
                              rational_forward_kin_, &plane_geometries);
  GeneratePathRationals(plane_geometries);
}

void CspaceFreePath::GeneratePathRationals(
    const std::vector<PlaneSeparatesGeometries>& plane_geometries) {
  // plane_geometries_ currently has rationals in terms of the configuration
  // space variable. We create PlaneSeparatesGeometriesOnPath objects which are
  // in terms of the path variable and can be used to construct the
  // certification program once a path is chosen.
  symbolic::Polynomial::SubstituteAndExpandCacheData cached_substitutions;

  // Add the auxilliary variables for matrix SOS constraints to the substitution
  // map.
  std::unordered_map<symbolic::Variable, symbolic::Polynomial>
      path_with_y_subs = path_;
  path_with_y_subs.emplace(mu_, symbolic::Polynomial(mu_));
  symbolic::Variables indeterminates{mu_};
  for (int i = 0; i < y_slack_.size(); ++i) {
    path_with_y_subs.emplace(y_slack_(i), symbolic::Polynomial(y_slack_(i)));
    indeterminates.insert(y_slack_(i));
  }
  for (const auto& plane_geometry : plane_geometries) {
    plane_geometries_on_path_.emplace_back(plane_geometry, mu_,
                                           path_with_y_subs, indeterminates,
                                           &cached_substitutions);
  }
}

[[nodiscard]] CspaceFreePath::SeparationCertificateProgram
CspaceFreePath::MakeIsGeometrySeparableOnPathProgram(
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const VectorX<Polynomiald>& path) const {
  // Fail fast as building the program can be expensive.
  int plane_index{GetSeparatingPlaneIndex(geometry_pair)};
  if (plane_index < 0) {
    throw std::runtime_error(fmt::format(
        "GetIsGeometrySeparableProgram(): geometry pair ({}, {}) does not need "
        "a separation certificate",
        scene_graph_.model_inspector().GetName(geometry_pair.first()),
        scene_graph_.model_inspector().GetName(geometry_pair.second())));
  }

  DRAKE_DEMAND(rational_forward_kin_.s().rows() == path.rows());
  // Now we convert the vector of common::Polynomial to a map from the
  // configuration space variable s to symbolic::Polynomial in mu.
  std::unordered_map<symbolic::Variable, symbolic::Polynomial>
      cspace_var_to_sym_path;
  for (int i = 0; i < path.rows(); ++i) {
    DRAKE_DEMAND(path(i).is_univariate());
    DRAKE_DEMAND(path(i).GetDegree() <= static_cast<int>(max_degree_));
    symbolic::Polynomial::MapType sym_path_map;
    for (const auto& monom : path(i).GetMonomials()) {
      sym_path_map.insert(
          {symbolic::Monomial(mu_, monom.GetDegree()), monom.coefficient});
    }
    cspace_var_to_sym_path.emplace(rational_forward_kin_.s()(i),
                                   symbolic::Polynomial{sym_path_map});
  }

  return ConstructPlaneSearchProgramOnPath(
      plane_geometries_on_path_.at(plane_index), cspace_var_to_sym_path);
}

[[nodiscard]] CspaceFreePath::SeparationCertificateProgram
CspaceFreePath::ConstructPlaneSearchProgramOnPath(
    const PlaneSeparatesGeometriesOnPath& plane_geometries_on_path,
    const std::unordered_map<symbolic::Variable, symbolic::Polynomial>& path)
    const {
  SeparationCertificateProgram ret{path, plane_geometries_on_path.plane_index};
  ret.prog->AddIndeterminate(mu_);
  ret.prog->AddIndeterminates(this->y_slack());

  // construct the parameter to value map
  symbolic::Environment param_eval_map;
  for (const auto& [config_space_var, eval_path] : path) {
    const symbolic::Polynomial symbolic_path{path_.at(config_space_var)};
    for (const auto& [mu_monom, mu_var_coeff] :
         symbolic_path.monomial_to_coefficient_map()) {
      // Find the monomial with the matching degree. If it doesn't exist
      // evaluate it to 0.
      const auto evaled_monom_iter =
          eval_path.monomial_to_coefficient_map().find(mu_monom);
      const double mu_var_coeff_eval{
          evaled_monom_iter == eval_path.monomial_to_coefficient_map().end()
              ? 0
              : evaled_monom_iter->second.Evaluate()};
      param_eval_map.insert(*mu_var_coeff.GetVariables().begin(),
                            mu_var_coeff_eval);
    }
  }

  // Now add the separation conditions to the program
  for (const auto& condition :
       plane_geometries_on_path.positive_side_conditions) {
    condition.AddPositivityConstraintToProgram(param_eval_map,
                                               ret.prog.get_mutable());
  }
  for (const auto& condition :
       plane_geometries_on_path.negative_side_conditions) {
    condition.AddPositivityConstraintToProgram(param_eval_map,
                                               ret.prog.get_mutable());
  }
  return ret;
}

CspaceFreePath::SeparationCertificateResult
CspaceFreePath::SolveSeparationCertificateProgram(
    const CspaceFreePath::SeparationCertificateProgram& certificate_program,
    const FindSeparationCertificateOptions& options) const {
  CspaceFreePath::SeparationCertificateResult result{};
  internal::SolveSeparationCertificateProgramBase(
      certificate_program, options,
      separating_planes_[certificate_program.plane_index], &result);
  return result;
}

int CspaceFreePath::GetSeparatingPlaneIndex(
    const SortedPair<geometry::GeometryId>& pair) const {
  return (!map_geometries_to_separating_planes_.contains(pair))
             ? -1
             : map_geometries_to_separating_planes_.at(pair);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
