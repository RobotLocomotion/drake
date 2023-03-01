#include "drake/geometry/optimization/cspace_free_polytope.h"

#include <future>
#include <limits>
#include <list>
#include <set>
#include <string>
#include <thread>

#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {
const double kInf = std::numeric_limits<double>::infinity();
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

// Return the total size of the lower triangular variables in the Gram
// matrices.
int GetGramVarSize(
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
    bool with_cross_y, int num_y) {
  auto gram_lower_size = [](int gram_rows) {
    return gram_rows * (gram_rows + 1) / 2;
  };
  if (num_y == 0) {
    // We only need to use monomial_basis_array[0].
    return gram_lower_size(monomial_basis_array[0].rows());
  } else {
    // We will use the monomials that contain y for the psd_mat.
    // We will denote monomial_basis_array[0] as m(s), and
    // monomial_basis_array[i+1] as  yᵢ * m(s).
    if (with_cross_y) {
      // The monomials basis we use are [m(s); y₀*m(s), ..., yₙ * m(s)] where n
      // = num_y - 1.
      int gram_rows = monomial_basis_array[0].rows();
      for (int i = 0; i < num_y; ++i) {
        gram_rows += monomial_basis_array[i + 1].rows();
      }
      return gram_lower_size(gram_rows);
    } else {
      // Use multiple monomial basis, each monomials basis is [m(s); yᵢ*m(s)].
      int ret = 0;
      for (int i = 0; i < num_y; ++i) {
        ret += gram_lower_size(monomial_basis_array[0].rows() +
                               monomial_basis_array[i + 1].rows());
      }
      return ret;
    }
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

// TODO(hongkai.dai): move this change to MathematicalProgram.
void AddPsdConstraint(solvers::MathematicalProgram* prog,
                      const MatrixX<symbolic::Variable>& X) {
  DRAKE_DEMAND(X.rows() == X.cols());
  if (X.rows() == 1) {
    prog->AddBoundingBoxConstraint(0, kInf, X(0, 0));
  } else if (X.rows() == 2) {
    prog->AddRotatedLorentzConeConstraint(
        Vector3<symbolic::Variable>(X(0, 0), X(1, 1), X(0, 1)));
  } else {
    prog->AddPositiveSemidefiniteConstraint(X);
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

// Given the monomial_basis_array, compute the sos polynomial.
// monomial_basis_array contains [m(s), y₀*m(s), y₁*m(s), y₂*m(s)].
//
// If num_y == 0, then the sos polynomial is just
// m(s)ᵀ * X * m(s)
// where X is a Gram matrix, `grams` is a length-1 vector containing X.
//
// If num_y != 0 and with_cross_y = true, then the sos polynomial is
// ⌈    m(s)⌉ᵀ * Y * ⌈    m(s)⌉
// | y₀*m(s)|        | y₀*m(s)|
// |   ...  |        |   ...  |
// ⌊ yₙ*m(s)⌋        ⌊ yₙ*m(s)⌋
// where n = num_y-1. Y is a Gram matrix, `grams` is a length-1 vector
// containing Y.
//
// if num_y != 0 and with_cross_y = false, then the sos polynomial is
// ∑ᵢ ⌈    m(s)⌉ᵀ * Zᵢ * ⌈    m(s)⌉
//    ⌊ yᵢ*m(s)⌋         ⌊ yᵢ*m(s)⌋
// where Zᵢ is a Gram matrix, i = 0, ..., num_y-1.  `grams` is a vector of
// length `num_y`, and grams[i] = Zᵢ
struct GramAndMonomialBasis {
  GramAndMonomialBasis(
      const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
      bool with_cross_y, int num_y) {
    this->gram_var_size =
        GetGramVarSize(monomial_basis_array, with_cross_y, num_y);
    if (num_y == 0) {
      // We only need to use monomial_basis_array[0].
      this->grams.emplace_back(monomial_basis_array[0].rows(),
                               monomial_basis_array[0].rows());
      this->monomial_basis.push_back(monomial_basis_array[0]);
    } else {
      // We will use the monomials that contain y for the psd_mat.
      // We will denote monomial_basis_array[0] as m(s), and
      // monomial_basis_array[i+1] as  yᵢ * m(s).
      if (with_cross_y) {
        // The monomials basis we use is [m(s); y₀*m(s), ..., yₙ * m(s)] where
        // n = num_y - 1.
        int gram_rows = monomial_basis_array[0].rows();
        for (int i = 0; i < num_y; ++i) {
          gram_rows += monomial_basis_array[i + 1].rows();
        }
        this->grams.emplace_back(gram_rows, gram_rows);
        this->monomial_basis.emplace_back(gram_rows);
        this->monomial_basis[0].topRows(monomial_basis_array[0].rows()) =
            monomial_basis_array[0];
        gram_rows = monomial_basis_array[0].rows();
        for (int i = 0; i < num_y; ++i) {
          this->monomial_basis[0].segment(gram_rows,
                                          monomial_basis_array[i + 1].rows()) =
              monomial_basis_array[i + 1];
          gram_rows += monomial_basis_array[i + 1].rows();
        }
      } else {
        // Use multiple monomial bases, each monomial basis is [m(s); yᵢ*m(s)].
        for (int i = 0; i < num_y; ++i) {
          const int gram_rows = monomial_basis_array[0].rows() +
                                monomial_basis_array[i + 1].rows();
          this->grams.emplace_back(gram_rows, gram_rows);
          this->monomial_basis.emplace_back(gram_rows);
          this->monomial_basis.back().topRows(monomial_basis_array[0].rows()) =
              monomial_basis_array[0];
          this->monomial_basis.back().bottomRows(
              monomial_basis_array[i + 1].rows()) = monomial_basis_array[i + 1];
        }
      }
    }
  }

  // Add the constraint that the polynomial represented by this Gram and
  // monomial basis is sos.
  // @param is_zero_poly If true, then constrain all the Gram matrices to be
  // zero.
  void AddSos(solvers::MathematicalProgram* prog,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& gram_lower,
              symbolic::Polynomial* poly) {
    int gram_var_count = 0;
    for (auto& gram : this->grams) {
      const int gram_lower_size = gram.rows() * (gram.rows() + 1) / 2;
      SymmetricMatrixFromLowerTriangularPart<symbolic::Variable>(
          gram.rows(), gram_lower.segment(gram_var_count, gram_lower_size),
          &gram);
      gram_var_count += gram_lower_size;
    }
    *poly = symbolic::Polynomial();
    gram_var_count = 0;
    for (int i = 0; i < static_cast<int>(this->grams.size()); ++i) {
      AddPsdConstraint(prog, this->grams[i]);
      const int gram_lower_size =
          this->grams[i].rows() * (this->grams[i].rows() + 1) / 2;
      *poly += symbolic::CalcPolynomialWLowerTriangularPart(
          this->monomial_basis[i],
          gram_lower.segment(gram_var_count, gram_lower_size));
      gram_var_count += gram_lower_size;
    }
  }

  int gram_var_size;
  std::vector<MatrixX<symbolic::Variable>> grams;
  std::vector<VectorX<symbolic::Monomial>> monomial_basis;
};


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

CspaceFreePolytope::SeparatingPlaneLagrangians
CspaceFreePolytope::SeparatingPlaneLagrangians::GetSolution(
    const solvers::MathematicalProgramResult& result) const {
  CspaceFreePolytope::SeparatingPlaneLagrangians ret(this->polytope_.rows(),
                                                     this->s_lower_.rows());
  for (int i = 0; i < this->polytope().rows(); ++i) {
    ret.polytope_(i) = result.GetSolution(this->polytope()(i));
  }
  for (int i = 0; i < this->s_lower_.rows(); ++i) {
    ret.s_lower_(i) = result.GetSolution(this->s_lower_(i));
    ret.s_upper_(i) = result.GetSolution(this->s_upper_(i));
  }
  return ret;
}

CspaceFreePolytope::SeparationCertificateProgram
CspaceFreePolytope::ConstructPlaneSearchProgram(
    const PlaneSeparatesGeometries& plane_geometries,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::unordered_set<int>& C_redundant_indices,
    const std::unordered_set<int>& s_lower_redundant_indices,
    const std::unordered_set<int>& s_upper_redundant_indices) const {
  SeparationCertificateProgram ret;
  ret.plane_index = plane_geometries.plane_index;
  ret.prog->AddIndeterminates(rational_forward_kin_.s());
  const auto& plane = separating_planes_[plane_geometries.plane_index];
  ret.prog->AddDecisionVariables(plane.decision_variables);

  // First count the total size of the gram matrix variables.
  int gram_var_count = 0;
  auto count_gram = [this, &d_minus_Cs, &C_redundant_indices,
                     &s_lower_redundant_indices, &s_upper_redundant_indices](
                        const symbolic::RationalFunction& rational,
                        const std::array<VectorX<symbolic::Monomial>, 4>&
                            monomial_basis_array) -> int {
    // Each rational >= 0 requires the Lagrangian multiplier for d-C*s,
    // s-s_lower and s_upper-s.
    const int s_size = this->rational_forward_kin_.s().rows();
    const int num_sos =
        1 + d_minus_Cs.rows() + 2 * s_size - C_redundant_indices.size() -
        s_lower_redundant_indices.size() - s_upper_redundant_indices.size();
    const int y_size = GetNumYInRational(rational, this->y_slack_);
    const int num_gram_vars_per_sos =
        GetGramVarSize(monomial_basis_array, this->with_cross_y_, y_size);
    return num_gram_vars_per_sos * num_sos;
  };
  const SortedPair<multibody::BodyIndex> positive_body_pair(
      plane.expressed_body, plane.positive_side_geometry->body_index());
  const std::array<VectorX<symbolic::Monomial>, 4>&
      monomial_basis_array_positive_side =
          this->map_body_to_monomial_basis_array_.at(positive_body_pair);
  for (const auto& rational : plane_geometries.positive_side_rationals) {
    gram_var_count += count_gram(rational, monomial_basis_array_positive_side);
  }
  const SortedPair<multibody::BodyIndex> negative_body_pair(
      plane.expressed_body, plane.negative_side_geometry->body_index());
  const auto& monomial_basis_array_negative_side =
      this->map_body_to_monomial_basis_array_.at(negative_body_pair);
  for (const auto& rational : plane_geometries.negative_side_rationals) {
    gram_var_count += count_gram(rational, monomial_basis_array_negative_side);
  }
  const auto gram_vars =
      ret.prog->NewContinuousVariables(gram_var_count, "Gram");

  gram_var_count = 0;
  auto add_rational_nonnegative =
      [this, &d_minus_Cs, &C_redundant_indices, &s_lower_redundant_indices,
       &s_upper_redundant_indices, &gram_vars, &gram_var_count](
          solvers::MathematicalProgram* prog,
          const symbolic::RationalFunction& rational,
          const std::array<VectorX<symbolic::Monomial>, 4>&
              monomial_basis_array) -> SeparatingPlaneLagrangians {
    const int y_size = GetNumYInRational(rational, this->y_slack_);
    GramAndMonomialBasis gram_and_monomial_basis(monomial_basis_array,
                                                 this->with_cross_y_, y_size);
    const int num_gram_vars_per_sos = gram_and_monomial_basis.gram_var_size;
    const int s_size = this->rational_forward_kin_.s().rows();
    SeparatingPlaneLagrangians lagrangians(d_minus_Cs.rows(), s_size);

    // Set lagrangians.polytope, add sos constraints.
    for (int j = 0; j < d_minus_Cs.rows(); ++j) {
      if (C_redundant_indices.count(j) == 0) {
        gram_and_monomial_basis.AddSos(
            prog, gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
            &(lagrangians.get_mutable_polytope()(j)));
        gram_var_count += num_gram_vars_per_sos;
      } else {
        lagrangians.get_mutable_polytope()(j) = symbolic::Polynomial();
      }
    }
    // Set lagrangians.s_lower and lagrangians.s_upper, add sos
    // constraints.
    for (int j = 0; j < s_size; ++j) {
      if (s_lower_redundant_indices.count(j) == 0) {
        gram_and_monomial_basis.AddSos(
            prog, gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
            &(lagrangians.get_mutable_s_lower()(j)));
        gram_var_count += num_gram_vars_per_sos;
      } else {
        lagrangians.get_mutable_s_lower()(j) = symbolic::Polynomial();
      }
      if (s_upper_redundant_indices.count(j) == 0) {
        gram_and_monomial_basis.AddSos(
            prog, gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
            &(lagrangians.get_mutable_s_upper()(j)));
        gram_var_count += num_gram_vars_per_sos;
      } else {
        lagrangians.get_mutable_s_upper()(j) = symbolic::Polynomial();
      }
    }
    const symbolic::Polynomial poly =
        rational.numerator() - lagrangians.polytope().dot(d_minus_Cs) -
        lagrangians.s_lower().dot(this->s_minus_s_lower_) -
        lagrangians.s_upper().dot(this->s_upper_minus_s_);
    symbolic::Polynomial poly_sos;
    gram_and_monomial_basis.AddSos(
        prog, gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
        &poly_sos);
    gram_var_count += num_gram_vars_per_sos;
    prog->AddEqualityConstraintBetweenPolynomials(poly, poly_sos);
    return lagrangians;
  };

  if (plane.positive_side_geometry->type() != CIrisGeometryType::kPolytope ||
      plane.negative_side_geometry->type() != CIrisGeometryType::kPolytope) {
    ret.prog->AddIndeterminates(y_slack_);
  }

  ret.certificate.positive_side_rational_lagrangians.reserve(
      plane_geometries.positive_side_rationals.size());
  for (const auto& rational : plane_geometries.positive_side_rationals) {
    ret.certificate.positive_side_rational_lagrangians.push_back(
        add_rational_nonnegative(ret.prog.get(), rational,
                                 monomial_basis_array_positive_side));
  }

  ret.certificate.negative_side_rational_lagrangians.reserve(
      plane_geometries.negative_side_rationals.size());
  for (const auto& rational : plane_geometries.negative_side_rationals) {
    ret.certificate.negative_side_rational_lagrangians.push_back(
        add_rational_nonnegative(ret.prog.get(), rational,
                                 monomial_basis_array_negative_side));
  }
  DRAKE_DEMAND(gram_var_count == gram_vars.rows());

  return ret;
}

CspaceFreePolytope::SeparationCertificateResult
CspaceFreePolytope::SeparationCertificate::GetSolution(
    int plane_index, const Vector3<symbolic::Polynomial>& a,
    const symbolic::Polynomial& b,
    const VectorX<symbolic::Variable>& plane_decision_vars,
    const solvers::MathematicalProgramResult& result) const {
  CspaceFreePolytope::SeparationCertificateResult ret;
  ret.plane_index = plane_index;

  auto set_lagrangians =
      [&result](
          const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
              lagrangians_vec,
          std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>*
              lagrangians_result) {
        lagrangians_result->reserve(lagrangians_vec.size());
        for (const auto& lagrangians : lagrangians_vec) {
          lagrangians_result->push_back(lagrangians.GetSolution(result));
        }
      };
  set_lagrangians(this->positive_side_rational_lagrangians,
                  &ret.positive_side_rational_lagrangians);
  set_lagrangians(this->negative_side_rational_lagrangians,
                  &ret.negative_side_rational_lagrangians);
  for (int i = 0; i < 3; ++i) {
    ret.a(i) = result.GetSolution(a(i));
  }
  ret.b = result.GetSolution(b);

  ret.plane_decision_var_vals = result.GetSolution(plane_decision_vars);
  return ret;
}

std::vector<std::optional<CspaceFreePolytope::SeparationCertificateResult>>
CspaceFreePolytope::FindSeparationCertificateGivenPolytope(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const FindSeparationCertificateGivenPolytopeOptions& options) const {
  const VectorX<symbolic::Polynomial> d_minus_Cs = this->CalcDminusCs(C, d);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  this->FindRedundantInequalities(
      C, d, this->s_lower_, this->s_upper_, 0., &C_redundant_indices,
      &s_lower_redundant_indices, &s_upper_redundant_indices);
  if (!options.ignore_redundant_C) {
    C_redundant_indices.clear();
  }

  // Stores the indices in separating_planes_ that don't appear in
  // ignored_collision_pairs.
  std::vector<int> active_plane_indices;
  active_plane_indices.reserve(separating_planes_.size());
  for (int i = 0; i < static_cast<int>(separating_planes_.size()); ++i) {
    if (ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
            separating_planes_[i].positive_side_geometry->id(),
            separating_planes_[i].negative_side_geometry->id())) == 0) {
      active_plane_indices.push_back(i);
    }
  }

  std::vector<std::optional<bool>> is_success(active_plane_indices.size(),
                                              std::nullopt);
  std::vector<std::optional<SeparationCertificateResult>> ret(
      active_plane_indices.size(), std::nullopt);

  // This lambda function formulates and solves a small SOS program for each
  // pair of geometries.
  auto solve_small_sos = [this, &d_minus_Cs, &C_redundant_indices,
                          &s_lower_redundant_indices,
                          &s_upper_redundant_indices, &active_plane_indices,
                          &options, &is_success, &ret](int plane_count) {
    const int plane_index = active_plane_indices[plane_count];
    auto certificate_program = this->ConstructPlaneSearchProgram(
        this->plane_geometries_[plane_index], d_minus_Cs, C_redundant_indices,
        s_lower_redundant_indices, s_upper_redundant_indices);
    solvers::MathematicalProgramResult result;
    solvers::MakeSolver(options.solver_id)
        ->Solve(*certificate_program.prog, std::nullopt, options.solver_options,
                &result);
    if (result.is_success()) {
      ret[plane_count].emplace(certificate_program.certificate.GetSolution(
          plane_index, separating_planes_[plane_index].a,
          separating_planes_[plane_index].b,
          separating_planes_[plane_index].decision_variables, result));
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
  bool stop_dispatching = false;
  while ((active_operations.size() > 0 ||
          (sos_dispatched < static_cast<int>(active_plane_indices.size()) &&
           !stop_dispatching))) {
    // Check for completed operations.
    for (auto operation = active_operations.begin();
         operation != active_operations.end();) {
      if (IsFutureReady(*operation)) {
        // This call to future.get() is necessary to propagate any exception
        // thrown during SOS setup/solve.
        const int plane_count = operation->get();
        if (options.verbose) {
          drake::log()->info("SOS {}/{} completed, is_success {}", plane_count,
                             active_plane_indices.size(),
                             is_success[plane_count].value());
        }
        if (!(is_success[plane_count].value()) &&
            options.terminate_at_failure) {
          stop_dispatching = true;
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
           sos_dispatched < static_cast<int>(active_plane_indices.size()) &&
           !stop_dispatching) {
      active_operations.emplace_back(std::async(
          std::launch::async, std::move(solve_small_sos), sos_dispatched));
      if (options.verbose) {
        drake::log()->info("SOS {}/{} dispatched", sos_dispatched,
                           active_plane_indices.size());
      }
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
           plane_count < static_cast<int>(active_plane_indices.size());
           ++plane_count) {
        const int plane_index = active_plane_indices[plane_count];
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

bool CspaceFreePolytope::FindSeparationCertificateGivenPolytope(
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions&
        options,
    std::unordered_map<SortedPair<geometry::GeometryId>,
                       CspaceFreePolytope::SeparationCertificateResult>*
        certificates) const {
  const auto d_minus_Cs = this->CalcDminusCs<double>(C, d);

  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  this->FindRedundantInequalities(
      C, d, s_lower_, s_upper_, 0., &C_redundant_indices,
      &s_lower_redundant_indices, &s_upper_redundant_indices);

  const std::vector<std::optional<SeparationCertificateResult>>
      certificates_vec = this->FindSeparationCertificateGivenPolytope(
          ignored_collision_pairs, C, d, options);

  certificates->clear();
  bool is_success = true;
  for (const auto& certificate : certificates_vec) {
    if (certificate.has_value()) {
      const auto& plane = separating_planes_[certificate->plane_index];
      certificates->emplace(
          SortedPair<geometry::GeometryId>(plane.positive_side_geometry->id(),
                                           plane.negative_side_geometry->id()),
          std::move(certificate.value()));
    } else {
      is_success = false;
    }
  }
  return is_success;
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
