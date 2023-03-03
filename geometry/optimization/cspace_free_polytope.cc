#include "drake/geometry/optimization/cspace_free_polytope.h"

#include <set>

#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {
// Returns true if all the bodies on the kinematics chain from `start` to `end`
// are welded together (namely all the mobilizers in between are welded).
[[nodiscard]] bool ChainIsWelded(const multibody::MultibodyPlant<double>& plant,
                                 multibody::BodyIndex start,
                                 multibody::BodyIndex end) {
  const std::vector<multibody::internal::MobilizerIndex> mobilizers =
      multibody::internal::FindMobilizersOnPath(plant, start, end);
  if (mobilizers.size() == 0) {
    return true;
  }
  const multibody::internal::MultibodyTree<double>& tree =
      multibody::internal::GetInternalTree(plant);
  for (const auto& mobilizer : mobilizers) {
    if (tree.get_mobilizer(mobilizer).num_positions() != 0) {
      return false;
    }
  }
  return true;
}

// For all the geometries `link1_geometries` on link1, and `link2_geometries` on
// link2, return them as pairs if they are not filtered.
[[nodiscard]] std::vector<
    std::pair<const CIrisCollisionGeometry*, const CIrisCollisionGeometry*>>
GetLinkCollisionPairs(
    const multibody::MultibodyPlant<double>& plant,
    const SceneGraph<double>& scene_graph, multibody::BodyIndex link1,
    multibody::BodyIndex link2,
    const std::vector<std::unique_ptr<CIrisCollisionGeometry>>&
        link1_geometries,
    const std::vector<std::unique_ptr<CIrisCollisionGeometry>>&
        link2_geometries) {
  std::vector<
      std::pair<const CIrisCollisionGeometry*, const CIrisCollisionGeometry*>>
      ret;
  if (ChainIsWelded(plant, link1, link2)) {
    // Two links cannot collide if there are only welded joints between them as
    // they cannot move relative to each other. Return an empty vector.
    return ret;
  }
  const auto& model_inspector = scene_graph.model_inspector();
  for (const auto& geometry1 : link1_geometries) {
    for (const auto& geometry2 : link2_geometries) {
      if (!model_inspector.CollisionFiltered(geometry1->id(),
                                             geometry2->id())) {
        ret.emplace_back(geometry1.get(), geometry2.get());
      }
    }
  }
  return ret;
}

struct BodyPair {
  BodyPair(multibody::BodyIndex m_body1, multibody::BodyIndex m_body2)
      : body1{m_body1}, body2{m_body2} {}
  bool operator==(const BodyPair& other) const {
    return body1 == other.body1 && body2 == other.body2;
  }

  multibody::BodyIndex body1;
  multibody::BodyIndex body2;
};

struct BodyPairHash {
  size_t operator()(const BodyPair& p) const { return p.body1 * 100 + p.body2; }
};

// Returns the number of y_slack variables in `rational`.
// Not all y_slack necessarily appear in `rational`.
int GetNumYInRational(const symbolic::RationalFunction& rational,
                      const Vector3<symbolic::Variable>& y_slack) {
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    if (rational.numerator().indeterminates().find(y_slack(i)) !=
        rational.numerator().indeterminates().end()) {
      ++count;
    }
  }
  return count;
}

/**
 The monomials in the numerator of body point position has the form ∏ᵢ pow(tᵢ,
 dᵢ) * ∏ⱼ pow(xⱼ, cⱼ), where tᵢ is the configuration variable tᵢ = tan(Δθᵢ/2)
 for the revolute joint, dᵢ = 0, 1, or 2, on the kinematic chain between the
 pair of bodies; xⱼ is the prismatic joint displacement on the same kinematic
 chain, cⱼ = 0 or 1. When we use SeparatingPlaneOrder = kAffine, the monomial
 basis would include all the monomials in the form ∏ᵢ pow(sᵢ, nᵢ) with nᵢ = 0
 or 1, s ∈ {tᵢ, xⱼ}. Let's denote this monomial basis as m(s).

 When we have non-polytopic collision geometries, we will also impose a matrix
 of rational functions being positive semidefinite. This requires a matrix-sos
 constraint with slack variable y. We will need the monomial basis in the form
 yᵢ*m(s) with i=0,1,2.

 @param[in/out] map_body_to_monomial_basis_array stores all the monomials basis
 already computed.
 @param[out] monomial_basis_array The monomial basis array for this pair of
 body, monomial_basis_array = [m(s), y₀*m(s), y₁*m(s), y₂*m(s)]
 */
void FindMonomialBasisArray(
    const multibody::RationalForwardKinematics& rational_forward_kin,
    const Vector3<symbolic::Variable>& y_slack,
    const SortedPair<multibody::BodyIndex>& body_pair,
    std::unordered_map<SortedPair<multibody::BodyIndex>,
                       std::array<VectorX<symbolic::Monomial>, 4>>*
        map_body_to_monomial_basis_array,
    std::array<VectorX<symbolic::Monomial>, 4>* monomial_basis_array) {
  auto body_pair_it = map_body_to_monomial_basis_array->find(body_pair);
  if (body_pair_it == map_body_to_monomial_basis_array->end()) {
    const std::vector<multibody::internal::MobilizerIndex> mobilizer_indices =
        multibody::internal::FindMobilizersOnPath(rational_forward_kin.plant(),
                                                  body_pair.first(),
                                                  body_pair.second());
    const auto& tree =
        multibody::internal::GetInternalTree(rational_forward_kin.plant());
    symbolic::Variables s_set;
    for (const auto& mobilizer_index : mobilizer_indices) {
      const auto& mobilizer = tree.get_mobilizer(mobilizer_index);
      if ((mobilizer.can_rotate() && !mobilizer.can_translate()) ||
          (mobilizer.can_translate() && !mobilizer.can_rotate())) {
        // This is a revolute or prismatic joint.
        s_set.insert(
            rational_forward_kin
                .s()[rational_forward_kin
                         .map_mobilizer_to_s_index()[mobilizer_index]]);
      } else if (mobilizer.num_velocities() > 0) {
        throw std::runtime_error(
            "FindMonomialBasis: we only support revolute, prismatic or weld "
            "mobilizers.");
      }
    }
    if (s_set.empty()) {
      // No s variable. The monomial basis is just [1].
      (*monomial_basis_array)[0].resize(1);
      (*monomial_basis_array)[0](0) = symbolic::Monomial();
    } else {
      (*monomial_basis_array)[0] =
          symbolic::CalcMonomialBasisOrderUpToOne(s_set);
    }
    // monomial_basis_array[i+1] = y(i) * monomial_basis_array[0]
    for (int i = 0; i < 3; ++i) {
      const symbolic::Monomial yi(y_slack(i));
      (*monomial_basis_array)[i + 1].resize((*monomial_basis_array)[0].rows());
      for (int j = 0; j < (*monomial_basis_array)[0].rows(); ++j) {
        (*monomial_basis_array)[i + 1](j) = yi * (*monomial_basis_array)[0](j);
      }
    }
    map_body_to_monomial_basis_array->emplace(body_pair, *monomial_basis_array);
  } else {
    *monomial_basis_array = body_pair_it->second;
  }
}

}  // namespace

CspaceFreePolytope::CspaceFreePolytope(
    const multibody::MultibodyPlant<double>* plant,
    const geometry::SceneGraph<double>* scene_graph,
    SeparatingPlaneOrder plane_order,
    const Eigen::Ref<const Eigen::VectorXd>& q_star, const Options& options)
    : rational_forward_kin_(plant),
      scene_graph_{*scene_graph},
      link_geometries_{GetCollisionGeometries(*plant, *scene_graph)},
      plane_order_{plane_order},
      s_set_{rational_forward_kin_.s()},
      q_star_{q_star},
      with_cross_y_{options.with_cross_y} {
  // Create separating planes.
  // collision_pairs maps each pair of body to the pair of collision geometries
  // on that pair of body.
  std::map<SortedPair<multibody::BodyIndex>,
           std::vector<std::pair<const CIrisCollisionGeometry*,
                                 const CIrisCollisionGeometry*>>>
      collision_pairs;
  int num_collision_pairs = 0;
  for (const auto& [link1, geometries1] : link_geometries_) {
    for (const auto& [link2, geometries2] : link_geometries_) {
      if (link1 < link2) {
        auto it = collision_pairs.emplace_hint(
            collision_pairs.end(),
            SortedPair<multibody::BodyIndex>(link1, link2),
            GetLinkCollisionPairs(rational_forward_kin_.plant(), scene_graph_,
                                  link1, link2, geometries1, geometries2));
        num_collision_pairs += static_cast<int>(it->second.size());
      }
    }
  }
  separating_planes_.reserve(num_collision_pairs);
  const symbolic::Monomial monomial_one{};
  for (const auto& [link_pair, geometry_pairs] : collision_pairs) {
    for (const auto& geometry_pair : geometry_pairs) {
      Vector3<symbolic::Polynomial> a;
      symbolic::Polynomial b;
      VectorX<symbolic::Variable> plane_decision_vars;
      switch (plane_order) {
        case SeparatingPlaneOrder::kAffine: {
          const VectorX<symbolic::Variable> s_for_plane =
              rational_forward_kin_.s();
          plane_decision_vars.resize(4 * s_for_plane.rows() + 4);
          for (int i = 0; i < plane_decision_vars.rows(); ++i) {
            plane_decision_vars(i) =
                symbolic::Variable(fmt::format("plane_var{}", i));
          }
          CalcPlane<symbolic::Variable, symbolic::Variable,
                    symbolic::Polynomial>(plane_decision_vars, s_for_plane,
                                          plane_order_, &a, &b);
        }
      }
      const multibody::BodyIndex expressed_body =
          multibody::internal::FindBodyInTheMiddleOfChain(
              rational_forward_kin_.plant(), link_pair.first(),
              link_pair.second());
      separating_planes_.emplace_back(a, b, geometry_pair.first,
                                      geometry_pair.second, expressed_body,
                                      plane_order_, plane_decision_vars);

      map_geometries_to_separating_planes_.emplace(
          SortedPair<geometry::GeometryId>(geometry_pair.first->id(),
                                           geometry_pair.second->id()),
          static_cast<int>(separating_planes_.size()) - 1);
    }
  }

  for (int i = 0; i < 3; ++i) {
    y_slack_(i) = symbolic::Variable("y" + std::to_string(i));
  }

  s_lower_ = rational_forward_kin_.ComputeSValue(
      rational_forward_kin_.plant().GetPositionLowerLimits(), q_star_);
  s_upper_ = rational_forward_kin_.ComputeSValue(
      rational_forward_kin_.plant().GetPositionUpperLimits(), q_star_);
  CalcSBoundsPolynomial();

  this->GenerateRationals();

  this->CalcMonomialBasis();
}

// Find the redundant inequalities in C * s <= d, s_lower <= s <= s_upper.
void CspaceFreePolytope::FindRedundantInequalities(
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
    const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
    double tighten, std::unordered_set<int>* C_redundant_indices,
    std::unordered_set<int>* s_lower_redundant_indices,
    std::unordered_set<int>* s_upper_redundant_indices) const {
  C_redundant_indices->clear();
  s_lower_redundant_indices->clear();
  s_upper_redundant_indices->clear();
  // We aggregate the constraint {C*s<=d, s_lower <= s <= s_upper} as C̅s ≤
  // d̅
  const int ns = s_lower.rows();
  Eigen::MatrixXd C_bar(C.rows() + 2 * ns, ns);
  Eigen::VectorXd d_bar(d.rows() + 2 * ns);
  C_bar << C, Eigen::MatrixXd::Identity(ns, ns),
      -Eigen::MatrixXd::Identity(ns, ns);
  d_bar << d, s_upper, -s_lower;
  const HPolyhedron hpolyhedron(C_bar, d_bar);
  const std::set<int> redundant_indices = hpolyhedron.FindRedundant(-tighten);
  C_redundant_indices->reserve(redundant_indices.size());
  s_lower_redundant_indices->reserve(redundant_indices.size());
  s_upper_redundant_indices->reserve(redundant_indices.size());
  for (const int index : redundant_indices) {
    if (index < C.rows()) {
      C_redundant_indices->emplace_hint(C_redundant_indices->end(), index);
    } else if (index < C.rows() + ns) {
      s_upper_redundant_indices->emplace_hint(s_upper_redundant_indices->end(),
                                              index - C.rows());
    } else {
      s_lower_redundant_indices->emplace_hint(s_lower_redundant_indices->end(),
                                              index - C.rows() - ns);
    }
  }
}

void CspaceFreePolytope::CalcSBoundsPolynomial() {
  const int s_size = rational_forward_kin_.s().rows();
  s_minus_s_lower_.resize(s_size);
  s_upper_minus_s_.resize(s_size);
  const symbolic::Monomial monomial_one{};
  const auto& s = rational_forward_kin_.s();
  for (int i = 0; i < s_size; ++i) {
    s_minus_s_lower_(i) = symbolic::Polynomial(symbolic::Polynomial::MapType(
        {{symbolic::Monomial(s(i)), 1}, {monomial_one, -s_lower_(i)}}));
    s_upper_minus_s_(i) = symbolic::Polynomial(symbolic::Polynomial::MapType(
        {{monomial_one, s_upper_(i)}, {symbolic::Monomial(s(i)), -1}}));
  }
}

template <typename T>
VectorX<symbolic::Polynomial> CspaceFreePolytope::CalcDminusCs(
    const Eigen::Ref<const MatrixX<T>>& C,
    const Eigen::Ref<const VectorX<T>>& d) const {
  // Now build the polynomials d(i) - C.row(i) * s
  const auto& s = rational_forward_kin_.s();
  DRAKE_DEMAND(C.rows() == d.rows() && C.cols() == static_cast<int>(s.size()));
  const symbolic::Monomial monomial_one{};
  symbolic::Polynomial::MapType d_minus_Cs_poly_map;
  VectorX<symbolic::Monomial> s_monomials(s.rows());
  for (int i = 0; i < s.rows(); ++i) {
    s_monomials(i) = symbolic::Monomial(s(i));
  }
  VectorX<symbolic::Polynomial> d_minus_Cs(d.rows());
  for (int i = 0; i < C.rows(); ++i) {
    for (int j = 0; j < static_cast<int>(s.rows()); ++j) {
      auto it = d_minus_Cs_poly_map.find(s_monomials(j));
      if (it == d_minus_Cs_poly_map.end()) {
        d_minus_Cs_poly_map.emplace_hint(it, s_monomials(j), -C(i, j));
      } else {
        it->second = -C(i, j);
      }
    }
    auto it = d_minus_Cs_poly_map.find(monomial_one);
    if (it == d_minus_Cs_poly_map.end()) {
      d_minus_Cs_poly_map.emplace_hint(it, monomial_one, d(i));
    } else {
      it->second = d(i);
    }
    d_minus_Cs(i) = symbolic::Polynomial(d_minus_Cs_poly_map);
  }
  return d_minus_Cs;
}


void CspaceFreePolytope::GenerateRationals() {
  // There can be multiple geometries on the same pair, hence the body pose will
  // be reused. We use this map to store the body pose to avoid redundant
  // computation.
  std::unordered_map<
      BodyPair,
      multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>,
      BodyPairHash>
      body_pair_to_X_AB_multilinear;
  for (int plane_index = 0;
       plane_index < static_cast<int>(separating_planes_.size());
       ++plane_index) {
    const auto& separating_plane = separating_planes_[plane_index];
    // Compute X_AB for both side of the geometries.
    std::vector<symbolic::RationalFunction> positive_side_rationals;
    std::vector<symbolic::RationalFunction> negative_side_rationals;
    for (const PlaneSide plane_side :
         {PlaneSide::kPositive, PlaneSide::kNegative}) {
      const CIrisCollisionGeometry* link_geometry =
          separating_plane.geometry(plane_side);

      const BodyPair expressed_to_link(separating_plane.expressed_body,
                                       link_geometry->body_index());
      auto it = body_pair_to_X_AB_multilinear.find(expressed_to_link);
      if (it == body_pair_to_X_AB_multilinear.end()) {
        std::tie(it, std::ignore) = body_pair_to_X_AB_multilinear.emplace(
            expressed_to_link,
            rational_forward_kin_.CalcBodyPoseAsMultilinearPolynomial(
                q_star_, link_geometry->body_index(),
                separating_plane.expressed_body));
      }
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear = it->second;
      auto& rationals = plane_side == PlaneSide::kPositive
                            ? positive_side_rationals
                            : negative_side_rationals;
      link_geometry->OnPlaneSide(separating_plane.a, separating_plane.b,
                                 X_AB_multilinear, rational_forward_kin_,
                                 plane_side, y_slack_, &rationals);
    }
    // For a non-polytopic geometry (sphere, capsule, etc) to be on the positive
    // side of the plane, we require that aᵀ*p_AS + b ≥ r|a|. To avoid a trivial
    // solution (a = b = 0), we also include the constraint that aᵀ*p_AS + b ≥ 1
    // (see ImplementGeometry for the Sphere type). This second constraint is
    // redundant if the negative side geometry is a polytope or if the negative
    // side geometry is also non-polytopic, and we have included the constraint
    // aᵀ*p_AS2 + b ≤ -1. We avoid adding the redundant constraint to reduce the
    // program size. We know that the redundant constraints can only be the
    // rationals without y_slack variable in non-polytopic geometries.
    if (separating_plane.positive_side_geometry->type() ==
            CIrisGeometryType::kPolytope &&
        separating_plane.negative_side_geometry->type() ==
            CIrisGeometryType::kPolytope) {
      plane_geometries_.emplace_back(positive_side_rationals,
                                     negative_side_rationals, plane_index);
    } else if (separating_plane.positive_side_geometry->type() ==
                   CIrisGeometryType::kPolytope &&
               separating_plane.negative_side_geometry->type() !=
                   CIrisGeometryType::kPolytope) {
      // Do not add the negative side rationals that have no y_slack variable.
      std::vector<symbolic::RationalFunction> negative_side_rationals_with_y;
      for (int i = 0; i < static_cast<int>(negative_side_rationals.size());
           ++i) {
        if (GetNumYInRational(negative_side_rationals[i], y_slack_) > 0) {
          negative_side_rationals_with_y.push_back(
              std::move(negative_side_rationals[i]));
        }
      }
      plane_geometries_.emplace_back(
          positive_side_rationals, negative_side_rationals_with_y, plane_index);
    } else if (separating_plane.positive_side_geometry->type() !=
                   CIrisGeometryType::kPolytope &&
               separating_plane.negative_side_geometry->type() ==
                   CIrisGeometryType::kPolytope) {
      // Do not add the positive side rationals that have no y_slack variables.
      std::vector<symbolic::RationalFunction> positive_side_rationals_with_y;
      for (int i = 0; i < static_cast<int>(positive_side_rationals.size());
           ++i) {
        if (GetNumYInRational(positive_side_rationals[i], y_slack_) > 0) {
          positive_side_rationals_with_y.push_back(
              std::move(positive_side_rationals[i]));
        }
      }
      plane_geometries_.emplace_back(positive_side_rationals_with_y,
                                     negative_side_rationals, plane_index);
    } else {
      // Both sides are non-polytopic, we only need the rationals without y from
      // one side, we choose the positive side. For the negative side, we only
      // add the rationals that contain y as indeterminates.
      std::vector<symbolic::RationalFunction> negative_side_rationals_with_y;
      for (int i = 0; i < static_cast<int>(negative_side_rationals.size());
           ++i) {
        if (GetNumYInRational(negative_side_rationals[i], y_slack_) > 0) {
          negative_side_rationals_with_y.push_back(
              std::move(negative_side_rationals[i]));
        }
      }
      plane_geometries_.emplace_back(
          positive_side_rationals, negative_side_rationals_with_y, plane_index);
    }
    DRAKE_DEMAND(plane_geometries_[plane_index].plane_index == plane_index);
  }
}

void CspaceFreePolytope::CalcMonomialBasis() {
  for (int plane_index = 0;
       plane_index < static_cast<int>(separating_planes_.size());
       ++plane_index) {
    const auto& plane = separating_planes_[plane_index];
    for (const auto collision_geometry :
         {plane.positive_side_geometry, plane.negative_side_geometry}) {
      const SortedPair<multibody::BodyIndex> body_pair(
          plane.expressed_body, collision_geometry->body_index());
      std::array<VectorX<symbolic::Monomial>, 4> monomial_basis_array;
      FindMonomialBasisArray(rational_forward_kin_, y_slack_, body_pair,
                             &map_body_to_monomial_basis_array_,
                             &monomial_basis_array);
    }
  }
}

std::map<multibody::BodyIndex,
         std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph) {
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
      ret;
  const auto& inspector = scene_graph.model_inspector();

  for (multibody::BodyIndex body_index{0}; body_index < plant.num_bodies();
       ++body_index) {
    const std::optional<geometry::FrameId> frame_id =
        plant.GetBodyFrameIdIfExists(body_index);
    if (frame_id.has_value()) {
      const auto geometry_ids =
          inspector.GetGeometries(frame_id.value(), geometry::Role::kProximity);
      for (const auto& geometry_id : geometry_ids) {
        auto collision_geometry = std::make_unique<CIrisCollisionGeometry>(
            &(inspector.GetShape(geometry_id)), body_index, geometry_id,
            inspector.GetPoseInFrame(geometry_id));
        auto body_it = ret.find(body_index);
        if (body_it == ret.end()) {
          std::vector<std::unique_ptr<CIrisCollisionGeometry>> body_geometries;
          body_geometries.push_back(std::move(collision_geometry));
          ret.emplace_hint(body_it, body_index, std::move(body_geometries));
        } else {
          body_it->second.push_back(std::move(collision_geometry));
        }
      }
    }
  }
  return ret;
}

// Explicit instantiation
template VectorX<symbolic::Polynomial> CspaceFreePolytope::CalcDminusCs<double>(
    const Eigen::Ref<const Eigen::MatrixXd>&,
    const Eigen::Ref<const Eigen::VectorXd>&) const;
template VectorX<symbolic::Polynomial>
CspaceFreePolytope::CalcDminusCs<symbolic::Variable>(
    const Eigen::Ref<const MatrixX<symbolic::Variable>>&,
    const Eigen::Ref<const VectorX<symbolic::Variable>>&) const;

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
