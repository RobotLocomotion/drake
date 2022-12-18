#include "drake/geometry/optimization/dev/cspace_free_polytope.h"

#include <chrono>
#include <future>
#include <limits>
#include <list>
#include <optional>
#include <set>
#include <string>
#include <thread>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/geometry/optimization/dev/collision_geometry.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
const double kInf = std::numeric_limits<double>::infinity();
namespace {
// Returns true if all the bodies on the kinematics chain from `start` to `end`
// are welded together (namely all the mobilizer in between are welded).
[[nodiscard]] bool ChainIsWeld(const multibody::MultibodyPlant<double>& plant,
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
    std::pair<const CollisionGeometry*, const CollisionGeometry*>>
GetLinkCollisionPairs(
    const multibody::MultibodyPlant<double>& plant,
    const SceneGraph<double>& scene_graph, multibody::BodyIndex link1,
    multibody::BodyIndex link2,
    const std::vector<std::unique_ptr<CollisionGeometry>>& link1_geometries,
    const std::vector<std::unique_ptr<CollisionGeometry>>& link2_geometries) {
  std::vector<std::pair<const CollisionGeometry*, const CollisionGeometry*>>
      ret;
  if (ChainIsWeld(plant, link1, link2)) {
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

/**
 The monomials in the numerator of body point position has the form ∏ᵢ pow(tᵢ,
 dᵢ) * ∏ⱼ pow(xⱼ, cⱼ), where tᵢ is the configuration variable tᵢ = tan(Δθᵢ/2)
 for the revolute joint, dᵢ = 0, 1, 2, on the kinematic chain between the pair
 of bodies; xⱼ is the prismatic joint displacement on the same kinematic chain,
 cⱼ = 0 or 1. When we use SeparatingPlaneOrder = kAffine, the monomial basis
 would include all the monomials in the form ∏ᵢ pow(sᵢ, mᵢ) with mᵢ = 0 or 1.

 @param[in/out] map_body_to_monomial_basis stores all the monomials basis
 already computed.
 @param[out] monomial_basis Te monomial basis for this pair of body.
 */
void FindMonomialBasis(
    const multibody::RationalForwardKinematics& rational_forward_kin,
    const SortedPair<multibody::BodyIndex>& body_pair,
    std::unordered_map<SortedPair<multibody::BodyIndex>,
                       VectorX<symbolic::Monomial>>* map_body_to_monomial_basis,
    VectorX<symbolic::Monomial>* monomial_basis) {
  auto it = map_body_to_monomial_basis->find(body_pair);
  if (it == map_body_to_monomial_basis->end()) {
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
      monomial_basis->resize(1);
      (*monomial_basis)(0) = symbolic::Monomial();
    } else {
      *monomial_basis = symbolic::CalcMonomialBasisOrderUpToOne(s_set);
    }
    it = map_body_to_monomial_basis->emplace_hint(it, body_pair,
                                                  *monomial_basis);
  } else {
    *monomial_basis = it->second;
  }
}

// TODO(hongkai.dai): move this function to a header file for general usage.
template <typename T>
void SymmetricMatrixFromLowerTriangularPart(
    int rows, const Eigen::Ref<const VectorX<T>>& lower_triangle,
    MatrixX<T>* mat) {
  mat->resize(rows, rows);
  DRAKE_DEMAND(lower_triangle.rows() == rows * (rows + 1) / 2);
  int count = 0;
  for (int j = 0; j < rows; ++j) {
    (*mat)(j, j) = lower_triangle(count++);
    for (int i = j + 1; i < rows; ++i) {
      (*mat)(i, j) = lower_triangle(count);
      (*mat)(j, i) = lower_triangle(count);
      count++;
    }
  }
}

// Computes the Lagrangian, and add the constraints on the Lagrangian Gram
// matrix to the program. If it is redundant, then set the Lagrangian to zero
// and constrain the Gram matrix to zero; otherwise constrain the Gram matrix to
// be PSD.
void AddLagrangian(
    solvers::MathematicalProgram* prog,
    const VectorX<symbolic::Monomial>& monomial_basis,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& gram_lower,
    bool redundant, symbolic::Polynomial* lagrangian,
    MatrixX<symbolic::Variable>* lagrangian_gram) {
  SymmetricMatrixFromLowerTriangularPart<symbolic::Variable>(
      monomial_basis.rows(), gram_lower, lagrangian_gram);
  DRAKE_DEMAND(gram_lower.rows() ==
               monomial_basis.rows() * (monomial_basis.rows() + 1) / 2);
  if (redundant) {
    // Lagrangian is 0.
    *lagrangian = symbolic::Polynomial();
    prog->AddBoundingBoxConstraint(0, 0, gram_lower);
  } else {
    *lagrangian = symbolic::CalcPolynomialWLowerTriangularPart(monomial_basis,
                                                               gram_lower);
    prog->AddPositiveSemidefiniteConstraint(*lagrangian_gram);
  }
}

// Checks if a future has completed execution.
// This function is taken from monte_carlo.cc. It will be used in the "thread
// pool" implementation (which doesn't use the openMP).
template <typename T>
bool IsFutureReady(const std::future<T>& future) {
  // future.wait_for() is the only method to check the status of a future
  // without waiting for it to complete.
  const std::future_status status =
      future.wait_for(std::chrono::milliseconds(1));
  return (status == std::future_status::ready);
}

[[nodiscard]] CspaceFreePolytope::SeparationCertificateResult GetSolution(
    int plane_index, const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const CspaceFreePolytope::SeparationCertificate& certificate,
    const std::optional<symbolic::Variable>& separating_margin,
    const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
    const solvers::MathematicalProgramResult& result) {
  CspaceFreePolytope::SeparationCertificateResult ret;
  ret.plane_index = plane_index;
  ret.C = C;
  ret.d = d;
  ret.positive_side_lagrangians.reserve(
      certificate.positive_side_lagrangians.size());
  for (const auto& lagrangians : certificate.positive_side_lagrangians) {
    ret.positive_side_lagrangians.push_back(lagrangians.GetSolution(result));
  }
  ret.negative_side_lagrangians.reserve(
      certificate.negative_side_lagrangians.size());
  for (const auto& lagrangians : certificate.negative_side_lagrangians) {
    ret.negative_side_lagrangians.push_back(lagrangians.GetSolution(result));
  }
  ret.unit_length_lagrangians.reserve(
      certificate.unit_length_lagrangians.size());
  for (const auto& lagrangians : certificate.unit_length_lagrangians) {
    ret.unit_length_lagrangians.push_back(lagrangians.GetSolution(result));
  }
  if (separating_margin.has_value()) {
    ret.separating_margin.emplace(
        result.GetSolution(separating_margin.value()));
  }
  for (int i = 0; i < 3; ++i) {
    ret.a(i) = result.GetSolution(a(i));
  }
  ret.b = result.GetSolution(b);
  return ret;
}
}  // namespace

CspaceFreePolytope::CspaceFreePolytope(
    const multibody::MultibodyPlant<double>* plant,
    const geometry::SceneGraph<double>* scene_graph,
    SeparatingPlaneOrder plane_order,
    const Eigen::Ref<const Eigen::VectorXd>& q_star)
    : rational_forward_kin_(plant),
      scene_graph_{*scene_graph},
      link_geometries_{GetCollisionGeometries(*plant, *scene_graph)},
      plane_order_{plane_order},
      s_set_{rational_forward_kin_.s()},
      q_star_{q_star} {
  // Create separating planes.
  // collision_pairs maps each pair of body to the pair of collision geometries
  // on that pair of body.
  std::map<SortedPair<multibody::BodyIndex>,
           std::vector<
               std::pair<const CollisionGeometry*, const CollisionGeometry*>>>
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
  // Currently we only need to impose the constraint that a 3D vector to have
  // unit length, which requires a 3 + 1 = 4 dimensional y_slack.
  y_slack_.resize(4);
  for (int i = 0; i < 4; ++i) {
    y_slack_(i) = symbolic::Variable("y" + std::to_string(i));
  }

  s_lower_ = rational_forward_kin_.ComputeSValue(
      rational_forward_kin_.plant().GetPositionLowerLimits(), q_star_);
  s_upper_ = rational_forward_kin_.ComputeSValue(
      rational_forward_kin_.plant().GetPositionUpperLimits(), q_star_);
  CalcSBoundsPolynomial();
}

std::vector<PlaneSeparatesGeometries> CspaceFreePolytope::GenerateRationals(
    const CspaceFreePolytope::FilteredCollsionPairs& filtered_collision_pairs,
    bool search_separating_margin) const {
  std::vector<PlaneSeparatesGeometries> ret;
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
    if (filtered_collision_pairs.count(
            SortedPair(separating_plane.positive_side_geometry->id(),
                       separating_plane.negative_side_geometry->id())) == 0) {
      // Compute X_AB for both side of the geometries.
      std::vector<symbolic::RationalFunction> positive_side_rationals;
      std::vector<symbolic::RationalFunction> negative_side_rationals;
      std::vector<VectorX<symbolic::Polynomial>> unit_length_vectors;
      std::optional<symbolic::Variable> separating_margin = std::nullopt;
      if (search_separating_margin) {
        separating_margin.emplace(symbolic::Variable("margin"));
      }
      for (const PlaneSide plane_side :
           {PlaneSide::kPositive, PlaneSide::kNegative}) {
        const CollisionGeometry* link_geometry =
            separating_plane.geometry(plane_side);

        const BodyPair expressed_to_link(separating_plane.expressed_body,
                                         link_geometry->body_index());
        auto it = body_pair_to_X_AB_multilinear.find(expressed_to_link);
        if (it == body_pair_to_X_AB_multilinear.end()) {
          it = body_pair_to_X_AB_multilinear.emplace_hint(
              it, expressed_to_link,
              rational_forward_kin_.CalcBodyPoseAsMultilinearPolynomial(
                  q_star_, link_geometry->body_index(),
                  separating_plane.expressed_body));
        }
        const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
            X_AB_multilinear = it->second;
        std::optional<VectorX<symbolic::Polynomial>> unit_length_vec;
        auto& rationals = plane_side == PlaneSide::kPositive
                              ? positive_side_rationals
                              : negative_side_rationals;
        link_geometry->OnPlaneSide(separating_plane.a, separating_plane.b,
                                   X_AB_multilinear, rational_forward_kin_,
                                   separating_margin, plane_side, &rationals,
                                   &unit_length_vec);

        if (unit_length_vec.has_value()) {
          if (unit_length_vectors.empty()) {
            unit_length_vectors.push_back(unit_length_vec.value());
          } else {
            // The unit length vector for the positive side geometry might be
            // the same as the unit length vector for the negative side
            // geometry, for example if both geometries are spheres. So we check
            // if unit_length_vec is the same as the one in unit_length_vectors;
            // if they are the same, then we don't add it.
            bool existing_unit_length_vec = false;
            for (const auto& vec : unit_length_vectors) {
              bool match = true;
              if (vec.rows() == unit_length_vec->rows()) {
                for (int i = 0; i < vec.rows(); ++i) {
                  if (!vec(i).EqualTo((*unit_length_vec)(i))) {
                    match = false;
                    break;
                  }
                }
              } else {
                match = false;
              }
              if (match) {
                existing_unit_length_vec = true;
                break;
              }
            }
            if (!existing_unit_length_vec) {
              unit_length_vectors.push_back(unit_length_vec.value());
            }
          }
        }
      }
      ret.emplace_back(positive_side_rationals, negative_side_rationals,
                       unit_length_vectors, plane_index, separating_margin);
    }
  }
  return ret;
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

CspaceFreePolytope::SeparatingPlaneLagrangians
CspaceFreePolytope::SeparatingPlaneLagrangians::GetSolution(
    const solvers::MathematicalProgramResult& result) const {
  CspaceFreePolytope::SeparatingPlaneLagrangians ret(this->polytope.rows(),
                                                     this->s_lower.rows());
  for (int i = 0; i < this->polytope.rows(); ++i) {
    ret.polytope(i) = result.GetSolution(this->polytope(i));
  }
  for (int i = 0; i < this->s_lower.rows(); ++i) {
    ret.s_lower(i) = result.GetSolution(this->s_lower(i));
    ret.s_upper(i) = result.GetSolution(this->s_upper(i));
  }
  return ret;
}

CspaceFreePolytope::UnitLengthLagrangians
CspaceFreePolytope::UnitLengthLagrangians::GetSolution(
    const solvers::MathematicalProgramResult& result) const {
  CspaceFreePolytope::UnitLengthLagrangians ret(this->polytope.rows(),
                                                this->s_lower.rows());
  for (int i = 0; i < this->polytope.rows(); ++i) {
    ret.polytope(i) = result.GetSolution(this->polytope(i));
  }
  for (int i = 0; i < this->s_lower.rows(); ++i) {
    ret.s_lower(i) = result.GetSolution(this->s_lower(i));
    ret.s_upper(i) = result.GetSolution(this->s_upper(i));
  }
  ret.y_square = result.GetSolution(this->y_square);
  return ret;
}

std::unordered_map<SortedPair<multibody::BodyIndex>,
                   VectorX<symbolic::Monomial>>
CspaceFreePolytope::CalcMonomialBasis(
    const std::vector<PlaneSeparatesGeometries>& plane_geometries) const {
  std::unordered_map<SortedPair<multibody::BodyIndex>,
                     VectorX<symbolic::Monomial>>
      ret;
  for (const auto& plane_geometries_pair : plane_geometries) {
    const auto& plane = separating_planes_[plane_geometries_pair.plane_index];
    for (const auto collision_geometry :
         {plane.positive_side_geometry, plane.negative_side_geometry}) {
      const SortedPair<multibody::BodyIndex> body_pair(
          plane.expressed_body, collision_geometry->body_index());
      VectorX<symbolic::Monomial> monomial_basis;
      FindMonomialBasis(rational_forward_kin_, body_pair, &ret,
                        &monomial_basis);
    }
  }
  return ret;
}

CspaceFreePolytope::v
CspaceFreePolytope::ConstructPlaneSearchProgram(
    const PlaneSeparatesGeometries& plane_geometries,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::unordered_set<int>& C_redundant_indices,
    const std::unordered_set<int>& s_lower_redundant_indices,
    const std::unordered_set<int>& s_upper_redundant_indices,
    const std::unordered_map<SortedPair<multibody::BodyIndex>,
                             VectorX<symbolic::Monomial>>&
        map_body_to_monomial_basis) const {
  SeparationCertificate ret;
  ret.prog->AddIndeterminates(rational_forward_kin_.s());
  const auto& plane = separating_planes_[plane_geometries.plane_index];
  ret.prog->AddDecisionVariables(plane.decision_variables);
  if (plane_geometries.separating_margin.has_value()) {
    ret.prog->AddDecisionVariables(Vector1<symbolic::Variable>(
        plane_geometries.separating_margin.value()));
  }
  VectorX<symbolic::Monomial> positive_side_monomial_basis;
  VectorX<symbolic::Monomial> negative_side_monomial_basis;

  auto add_geometry =
      [&plane, &d_minus_Cs, &C_redundant_indices, &s_lower_redundant_indices,
       &s_upper_redundant_indices, &map_body_to_monomial_basis, this](
          solvers::MathematicalProgram* prog,
          const std::vector<symbolic::RationalFunction>& rationals,
          multibody::BodyIndex body,
          std::vector<SeparatingPlaneLagrangians>* search_plane_lagrangians) {
        const VectorX<symbolic::Monomial> monomial_basis =
            map_body_to_monomial_basis.at(
                SortedPair<multibody::BodyIndex>(plane.expressed_body, body));
        const int num_grams =
            rationals.size() * (1 + d_minus_Cs.rows() +
                                this->rational_forward_kin_.s().rows() * 2);
        const int gram_size =
            (monomial_basis.rows() + 1) * monomial_basis.rows() / 2;
        const int num_gram_vars = gram_size * num_grams;
        auto gram_vars = prog->NewContinuousVariables(num_gram_vars, "Gram");

        int gram_var_count = 0;
        search_plane_lagrangians->reserve(rationals.size());
        MatrixX<symbolic::Variable> gram(gram_size, gram_size);
        for (int i = 0; i < static_cast<int>(rationals.size()); ++i) {
          search_plane_lagrangians->emplace_back(
              d_minus_Cs.rows(), this->rational_forward_kin_.s().rows());
          auto& lagrangians = search_plane_lagrangians->back();
          for (int j = 0; j < d_minus_Cs.rows(); ++j) {
            AddLagrangian(prog, monomial_basis,
                          gram_vars.segment(gram_var_count, gram_size),
                          C_redundant_indices.count(j) > 0,
                          &(lagrangians.polytope(j)), &gram);
            gram_var_count += gram_size;
          }
          for (int j = 0; j < this->rational_forward_kin_.s().rows(); ++j) {
            AddLagrangian(prog, monomial_basis,
                          gram_vars.segment(gram_var_count, gram_size),
                          s_lower_redundant_indices.count(j) > 0,
                          &(lagrangians.s_lower(j)), &gram);
            gram_var_count += gram_size;
            AddLagrangian(prog, monomial_basis,
                          gram_vars.segment(gram_var_count, gram_size),
                          s_upper_redundant_indices.count(j) > 0,
                          &(lagrangians.s_upper(j)), &gram);
            gram_var_count += gram_size;
          }
          const symbolic::Polynomial poly =
              rationals[i].numerator() - lagrangians.polytope.dot(d_minus_Cs) -
              lagrangians.s_lower.dot(this->s_minus_s_lower_) -
              lagrangians.s_upper.dot(this->s_upper_minus_s_);
          // Add the constraint that poly is sos.
          // Use the gram variable gram_vars that has been allocated.
          SymmetricMatrixFromLowerTriangularPart<symbolic::Variable>(
              monomial_basis.rows(),
              gram_vars.segment(gram_var_count, gram_size), &gram);
          prog->AddPositiveSemidefiniteConstraint(gram);
          const symbolic::Polynomial poly_sos =
              symbolic::CalcPolynomialWLowerTriangularPart(
                  monomial_basis, gram_vars.segment(gram_var_count, gram_size));
          gram_var_count += gram_size;
          prog->AddEqualityConstraintBetweenPolynomials(poly, poly_sos);
        }
        DRAKE_DEMAND(gram_var_count == num_gram_vars);
      };
  add_geometry(ret.prog.get(), plane_geometries.positive_side_rationals,
               plane.positive_side_geometry->body_index(),
               &ret.positive_side_lagrangians);
  add_geometry(ret.prog.get(), plane_geometries.negative_side_rationals,
               plane.negative_side_geometry->body_index(),
               &ret.negative_side_lagrangians);
  if (!plane_geometries.unit_length_vectors.empty()) {
    ret.prog->AddIndeterminates(y_slack_);
    for (const auto& vec : plane_geometries.unit_length_vectors) {
      ret.unit_length_lagrangians.push_back(AddUnitLengthConstraint(
          ret.prog.get(), vec, d_minus_Cs, C_redundant_indices,
          s_lower_redundant_indices, s_upper_redundant_indices));
    }
  }

  if (plane_geometries.separating_margin.has_value()) {
    // Add the constraint that margin >= 0 and maximize margin.
    ret.prog->AddBoundingBoxConstraint(
        0, kInf, plane_geometries.separating_margin.value());
    ret.prog->AddLinearCost(-Vector1d(1), 0,
                            Vector1<symbolic::Variable>(
                                plane_geometries.separating_margin.value()));
  }
  return ret;
}

CspaceFreePolytope::UnitLengthLagrangians
CspaceFreePolytope::AddUnitLengthConstraint(
    solvers::MathematicalProgram* prog,
    const VectorX<symbolic::Polynomial>& unit_length_vec,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::unordered_set<int>& C_redundant_indices,
    const std::unordered_set<int>& s_lower_redundant_indices,
    const std::unordered_set<int>& s_upper_redundant_indices) const {
  CspaceFreePolytope::UnitLengthLagrangians ret(d_minus_Cs.rows(),
                                                this->s_minus_s_lower_.rows());
  // Currently we only handle unit_length_vec.rows() == 3, and each polynomial
  // in unit_length_vec has degree <= 1.
  // TODO(hongkai.dai): handle unit_length_vec.rows() != 3.
  if (unit_length_vec.rows() != 3) {
    throw std::runtime_error(
        "AddUnitLengthConstraint(): unit_length_vec should have 3 rows.");
  }
  // TODO(hongkai.dai): handle higher degree unit length vector.
  for (int i = 0; i < unit_length_vec.rows(); ++i) {
    // We set the Lagrangian multiplier degrees based on the assumption that
    // unit_length_vec is an affine polynomial of s.
    if (unit_length_vec(i).TotalDegree() > 1) {
      throw std::runtime_error(fmt::format(
          "AddUnitLengthConstraint(): unit_length_vec({}) has degree >1", i));
    }
  }
  // Compute p(y, s) = yᵀ * ⌈ 1   a(s)ᵀ⌉ * y
  //                        ⌊a(s)    I ⌋
  symbolic::Polynomial::MapType p_map;
  for (int i = 0; i < 1 + unit_length_vec.rows(); ++i) {
    p_map.emplace(symbolic::Monomial(y_slack_(i), 2), 1);
  }
  symbolic::Polynomial p{p_map};
  for (int i = 0; i < unit_length_vec.rows(); ++i) {
    p += 2 * unit_length_vec(i) *
         symbolic::Polynomial(symbolic::Monomial(y_slack_(i + 1)));
  }
  const int num_grams =
      1 + d_minus_Cs.rows() + 2 * this->s_minus_s_lower_.rows();
  // Since p(y, s) is quadratic in y and linear in s, with the highest-degree
  // monomial in the form yᵢ²sⱼ, we know that to make p(y, s) - λ₁(y, s)ᵀ(d−Cs)
  // − λ₂(y, s)ᵀ(s−s_lower) − λ₃(y, s)ᵀ(s_upper−s) - ν₄(y, s)(1−yᵀy) being sos,
  // we can set λ₁(y, s) λ₂(y, s) and λ₃(y, s) to be quadratic polynomials of y
  // and ν₄(y, s) to be an affine polynomial of s. These Lagrangian multipliers
  // degrees meet the minimum requirement to match the degree of p(y, s).
  VectorX<symbolic::Monomial> monomial_basis(2 + unit_length_vec.rows() + 1);
  monomial_basis(0) = symbolic::Monomial();
  for (int i = 0; i < 1 + unit_length_vec.rows(); ++i) {
    monomial_basis(1 + i) = symbolic::Monomial(y_slack_(i));
  }
  const int gram_size = monomial_basis.rows() * (monomial_basis.rows() + 1) / 2;
  const int num_gram_vars = gram_size * num_grams;
  const VectorX<symbolic::Variable> gram_vars =
      prog->NewContinuousVariables(num_gram_vars, "Gram");
  int gram_var_count = 0;
  MatrixX<symbolic::Variable> gram(monomial_basis.rows(),
                                   monomial_basis.rows());
  for (int i = 0; i < d_minus_Cs.rows(); ++i) {
    AddLagrangian(prog, monomial_basis,
                  gram_vars.segment(gram_var_count, gram_size),
                  C_redundant_indices.count(i) > 0, &(ret.polytope(i)), &gram);
    gram_var_count += gram_size;
  }
  for (int i = 0; i < this->s_minus_s_lower_.rows(); ++i) {
    AddLagrangian(
        prog, monomial_basis, gram_vars.segment(gram_var_count, gram_size),
        s_lower_redundant_indices.count(i) > 0, &(ret.s_lower(i)), &gram);
    gram_var_count += gram_size;
    AddLagrangian(
        prog, monomial_basis, gram_vars.segment(gram_var_count, gram_size),
        s_upper_redundant_indices.count(i) > 0, &(ret.s_upper(i)), &gram);
    gram_var_count += gram_size;
  }
  // This Lagrangian is just an affine polynomial of s, it is for the algebraic
  // set {y | yᵀy=1}.
  ret.y_square = prog->NewFreePolynomial(s_set_, 1);
  // Compute polynomial 1 - yᵀy
  symbolic::Polynomial::MapType one_minus_y_square_map;
  one_minus_y_square_map.emplace(symbolic::Monomial(), 1);
  for (int i = 0; i < 1 + unit_length_vec.rows(); ++i) {
    one_minus_y_square_map.emplace(symbolic::Monomial(y_slack_(i), 2), -1);
  }
  const symbolic::Polynomial one_minus_y_square{one_minus_y_square_map};
  const symbolic::Polynomial poly = p - ret.polytope.dot(d_minus_Cs) -
                                    ret.s_lower.dot(this->s_minus_s_lower_) -
                                    ret.s_upper.dot(this->s_upper_minus_s_) -
                                    ret.y_square * one_minus_y_square;
  // Now constrain poly to be sos.
  // `poly` is affine in `s`, specifically it only contains monomials in the
  // form of yᵢ²sⱼ, yᵢ², yᵢ, sⱼ, 1. Hence to make `poly` a sos polynomial, we
  // only need to consider the monomial basis [1, y], and cancel out the terms
  // that are linear in s.
  SymmetricMatrixFromLowerTriangularPart<symbolic::Variable>(
      monomial_basis.rows(), gram_vars.segment(gram_var_count, gram_size),
      &gram);
  prog->AddPositiveSemidefiniteConstraint(gram);
  const symbolic::Polynomial poly_sos =
      symbolic::CalcPolynomialWLowerTriangularPart(
          monomial_basis, gram_vars.segment(gram_var_count, gram_size));
  gram_var_count += gram_size;
  DRAKE_DEMAND(gram_var_count == num_gram_vars);
  prog->AddEqualityConstraintBetweenPolynomials(poly, poly_sos);

  return ret;
}

std::vector<std::optional<CspaceFreePolytope::SeparationCertificateResult>>
CspaceFreePolytope::FindSeparationCertificateGivenPolytope(
    const std::vector<PlaneSeparatesGeometries>& plane_geometries,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const std::unordered_map<SortedPair<multibody::BodyIndex>,
                             VectorX<symbolic::Monomial>>&
        map_body_to_monomial_basis,
    const FindSeparationCertificateGivenPolytopeOptions& options) const {
  const VectorX<symbolic::Polynomial> d_minus_Cs = this->CalcDminusCs(C, d);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  this->FindRedundantInequalities(
      C, d, this->s_lower_, this->s_upper_, 0., &C_redundant_indices,
      &s_lower_redundant_indices, &s_upper_redundant_indices);

  std::vector<std::optional<bool>> is_success(plane_geometries.size(),
                                              std::nullopt);
  std::vector<std::optional<SeparationCertificateResult>> ret(
      plane_geometries.size(), std::nullopt);

  // This lambda function formulates and solves a smal SOS program for each pair
  // of geometries.
  auto solve_small_sos =
      [this, &plane_geometries, &C, &d, &d_minus_Cs, &C_redundant_indices,
       &s_lower_redundant_indices, &s_upper_redundant_indices,
       &map_body_to_monomial_basis, &options, &is_success,
       &ret](int plane_count) {
        auto certificate = this->ConstructPlaneSearchProgram(
            plane_geometries[plane_count], d_minus_Cs, C_redundant_indices,
            s_lower_redundant_indices, s_upper_redundant_indices,
            map_body_to_monomial_basis);
        auto solver = solvers::MakeSolver(options.solver_id);
        solvers::MathematicalProgramResult result;
        solver->Solve(*(certificate.prog), std::nullopt, std::nullopt, &result);
        if (result.is_success()) {
          const int plane_index = plane_geometries[plane_count].plane_index;
          ret[plane_count].emplace(
              GetSolution(plane_index, C, d, certificate,
                          plane_geometries[plane_count].separating_margin,
                          separating_planes_[plane_index].a,
                          separating_planes_[plane_index].b, result));
          is_success[plane_count].emplace(true);
        } else {
          ret[plane_count].reset();
          is_success[plane_count].emplace(false);
        }
        return plane_count;
      };

  const int num_threads =
      options.num_threads > 0
          ? options.num_threads
          : static_cast<int>(std::thread::hardware_concurrency());
  // We implement the "thread pool" idea here, by following
  // MonteCarloSimulationParallel class. This implementation doesn't use openMP
  // library.
  std::list<std::future<int>> active_operations;
  // Keep track of how many SOS have been dispatched already.
  int sos_dispatched = 0;
  // If any SOS is infeasible, then we don't dispatch any more SOS and report
  // failure.
  bool terminate = false;
  while ((active_operations.size() > 0 ||
          sos_dispatched < static_cast<int>(plane_geometries.size())) &&
         !terminate) {
    // Check for completed operations.
    for (auto operation = active_operations.begin();
         operation != active_operations.end();) {
      if (IsFutureReady(*operation)) {
        // This call to future.get() is necessary to propagate any exception
        // thrown during SOS setup/solve.
        const int plane_count = operation->get();
        drake::log()->debug("SOS {}/{} completed, is_success {}", plane_count,
                            plane_geometries.size(),
                            is_success[plane_count].value());
        if (!(is_success[plane_count].value()) &&
            options.terminate_at_failure) {
          terminate = true;
          break;
        }
        // Erase returned iterator to the next node in the list.
        operation = active_operations.erase(operation);
      } else {
        // Advance to next node in the list.
        ++operation;
      }
    }

    // Dispatch new SOS.
    while (static_cast<int>(active_operations.size()) < num_threads &&
           sos_dispatched < static_cast<int>(plane_geometries.size())) {
      active_operations.emplace_back(std::async(
          std::launch::async, std::move(solve_small_sos), sos_dispatched));
      drake::log()->debug("SOS {}/{} dispatched", sos_dispatched,
                          plane_geometries.size());
      ++sos_dispatched;
    }

    // Wait a bit before checking for completion.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (std::all_of(is_success.begin(), is_success.end(),
                  [](std::optional<bool> flag) {
                    return flag.has_value() && flag.value();
                  })) {
    if (options.verbose) {
      drake::log()->info("Found Lagrangian multipliers and separating planes");
    }
  } else {
    if (options.verbose) {
      std::string bad_pairs;
      const auto& inspector = scene_graph_.model_inspector();
      for (int plane_count = 0;
           plane_count < static_cast<int>(plane_geometries.size());
           ++plane_count) {
        const int plane_index = plane_geometries[plane_count].plane_index;
        if (is_success[plane_count].has_value() &&
            !(is_success[plane_count].value())) {
          bad_pairs.append(fmt::format(
              "({}, {})\n",
              inspector.GetName(
                  separating_planes_[plane_index].positive_side_geometry->id()),
              inspector.GetName(separating_planes_[plane_index]
                                    .negative_side_geometry->id())));
        }
      }

      drake::log()->warn(fmt::format(
          "Cannot find Lagrangian multipliers and separating planes for \n{}",
          bad_pairs));
    }
  }
  return ret;
}

std::map<multibody::BodyIndex, std::vector<std::unique_ptr<CollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph) {
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CollisionGeometry>>>
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
        auto collision_geometry = std::make_unique<CollisionGeometry>(
            &(inspector.GetShape(geometry_id)), body_index, geometry_id,
            inspector.GetPoseInFrame(geometry_id));
        auto body_it = ret.find(body_index);
        if (body_it == ret.end()) {
          std::vector<std::unique_ptr<CollisionGeometry>> body_geometries;
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
