#include "drake/geometry/optimization/cspace_free_internal.h"

#include <limits>
#include <map>
#include <optional>
#include <unordered_map>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {
namespace {
const double kInf = std::numeric_limits<double>::infinity();

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

// Returns true if all the bodies on the kinematics chain from `start` to `end`
// are welded together (namely all the mobilizers in between are welded).
[[nodiscard]] bool ChainIsWelded(const multibody::MultibodyPlant<double>& plant,
                                 multibody::BodyIndex start,
                                 multibody::BodyIndex end) {
  const std::vector<multibody::internal::MobodIndex> mobilizers =
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

// TODO(hongkai.dai): move this function to a header file for general usage.
template <typename T>
void SymmetricMatrixFromLowerTriangularPart(
    int rows, const Eigen::Ref<const VectorX<T>>& lower_triangle,
    MatrixX<T>* mat) {
  mat->resize(rows, rows);
  DRAKE_THROW_UNLESS(lower_triangle.rows() == rows * (rows + 1) / 2);
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

// TODO(hongkai.dai): move this function to MathematicalProgram.
void AddPsdConstraint(solvers::MathematicalProgram* prog,
                      const MatrixX<symbolic::Variable>& X) {
  DRAKE_THROW_UNLESS(X.rows() == X.cols());
  if (X.rows() == 1) {
    prog->AddBoundingBoxConstraint(0, kInf, X(0, 0));
  } else if (X.rows() == 2) {
    prog->AddRotatedLorentzConeConstraint(
        Vector3<symbolic::Variable>(X(0, 0), X(1, 1), X(0, 1)));
  } else {
    prog->AddPositiveSemidefiniteConstraint(X);
  }
}

}  // namespace

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

int GenerateCollisionPairs(
    const multibody::MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const std::map<multibody::BodyIndex,
                   std::vector<std::unique_ptr<CIrisCollisionGeometry>>>&
        link_geometries,
    std::map<SortedPair<multibody::BodyIndex>,
             std::vector<std::pair<const CIrisCollisionGeometry*,
                                   const CIrisCollisionGeometry*>>>*
        collision_pairs) {
  int num_collision_pairs = 0;
  for (const auto& [link1, geometries1] : link_geometries) {
    for (const auto& [link2, geometries2] : link_geometries) {
      if (link1 < link2) {
        auto it = collision_pairs->emplace_hint(
            collision_pairs->end(),
            SortedPair<multibody::BodyIndex>(link1, link2),
            GetLinkCollisionPairs(plant, scene_graph, link1, link2, geometries1,
                                  geometries2));
        num_collision_pairs += static_cast<int>(it->second.size());
      }
    }
  }
  return num_collision_pairs;
}

void GenerateRationals(
    const std::map<int, const CSpaceSeparatingPlane<symbolic::Variable>*>&
        separating_planes,
    const Vector3<symbolic::Variable>& y_slack,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    std::vector<PlaneSeparatesGeometries>* plane_geometries) {
  // There can be multiple geometries on the same pair, hence the body pose will
  // be reused. We use this map to store the body pose to avoid redundant
  // computation.
  std::unordered_map<
      BodyPair,
      multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>,
      BodyPairHash>
      body_pair_to_X_AB_multilinear;
  for (const auto& [plane_index, plane_ptr] : separating_planes) {
    DRAKE_ASSERT(plane_ptr != nullptr);
    const auto& separating_plane = *plane_ptr;
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
            rational_forward_kin.CalcBodyPoseAsMultilinearPolynomial(
                q_star, link_geometry->body_index(),
                separating_plane.expressed_body));
      }
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear = it->second;
      auto& rationals = plane_side == PlaneSide::kPositive
                            ? positive_side_rationals
                            : negative_side_rationals;
      link_geometry->OnPlaneSide(separating_plane.a, separating_plane.b,
                                 X_AB_multilinear, rational_forward_kin,
                                 plane_side, y_slack, &rationals);
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
      plane_geometries->emplace_back(positive_side_rationals,
                                     negative_side_rationals, plane_index);
    } else if (separating_plane.positive_side_geometry->type() ==
                   CIrisGeometryType::kPolytope &&
               separating_plane.negative_side_geometry->type() !=
                   CIrisGeometryType::kPolytope) {
      // Do not add the negative side rationals that have no y_slack variable.
      std::vector<symbolic::RationalFunction> negative_side_rationals_with_y;
      for (int i = 0; i < static_cast<int>(negative_side_rationals.size());
           ++i) {
        if (GetNumYInRational(negative_side_rationals[i], y_slack) > 0) {
          negative_side_rationals_with_y.push_back(
              std::move(negative_side_rationals[i]));
        }
      }
      plane_geometries->emplace_back(
          positive_side_rationals, negative_side_rationals_with_y, plane_index);
    } else if (separating_plane.positive_side_geometry->type() !=
                   CIrisGeometryType::kPolytope &&
               separating_plane.negative_side_geometry->type() ==
                   CIrisGeometryType::kPolytope) {
      // Do not add the positive side rationals that have no y_slack variables.
      std::vector<symbolic::RationalFunction> positive_side_rationals_with_y;
      for (int i = 0; i < static_cast<int>(positive_side_rationals.size());
           ++i) {
        if (GetNumYInRational(positive_side_rationals[i], y_slack) > 0) {
          positive_side_rationals_with_y.push_back(
              std::move(positive_side_rationals[i]));
        }
      }
      plane_geometries->emplace_back(positive_side_rationals_with_y,
                                     negative_side_rationals, plane_index);
    } else {
      // Both sides are non-polytopic, we only need the rationals without y from
      // one side, we choose the positive side. For the negative side, we only
      // add the rationals that contain y as indeterminates.
      std::vector<symbolic::RationalFunction> negative_side_rationals_with_y;
      for (int i = 0; i < static_cast<int>(negative_side_rationals.size());
           ++i) {
        if (GetNumYInRational(negative_side_rationals[i], y_slack) > 0) {
          negative_side_rationals_with_y.push_back(
              std::move(negative_side_rationals[i]));
        }
      }
      plane_geometries->emplace_back(
          positive_side_rationals, negative_side_rationals_with_y, plane_index);
    }
    DRAKE_DEMAND(plane_geometries->back().plane_index == plane_index);
  }
}

void GenerateRationals(
    const std::vector<std::unique_ptr<
        CSpaceSeparatingPlane<symbolic::Variable>>>& separating_planes,
    const Vector3<symbolic::Variable>& y_slack,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    std::vector<PlaneSeparatesGeometries>* plane_geometries) {
  std::map<int, const CSpaceSeparatingPlane<symbolic::Variable>*>
      separating_planes_map;
  for (int i = 0; i < static_cast<int>(separating_planes.size()); ++i) {
    separating_planes_map.emplace(i, separating_planes.at(i).get());
  }
  GenerateRationals(separating_planes_map, y_slack, q_star,
                    rational_forward_kin, plane_geometries);
}

void SolveSeparationCertificateProgramBase(
    const SeparationCertificateProgramBase& certificate_program,
    const FindSeparationCertificateOptions& options,
    const CSpaceSeparatingPlane<symbolic::Variable>& separating_plane,
    SeparationCertificateResultBase* result) {
  result->plane_index = certificate_program.plane_index;
  solvers::MakeSolver(options.solver_id)
      ->Solve(*certificate_program.prog, std::nullopt, options.solver_options,
              &(result->result));
  if (result->result.is_success()) {
    result->plane_decision_var_vals =
        result->result.GetSolution(separating_plane.decision_variables);
    for (int i = 0; i < 3; ++i) {
      result->a(i) = result->result.GetSolution(separating_plane.a(i));
    }
    result->b = result->result.GetSolution(separating_plane.b);
  }
}

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

GramAndMonomialBasis::GramAndMonomialBasis(
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
    bool with_cross_y, int num_y) {
  this->gram_var_size =
      internal::GetGramVarSize(monomial_basis_array, with_cross_y, num_y);
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
        const int gram_rows =
            monomial_basis_array[0].rows() + monomial_basis_array[i + 1].rows();
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

void GramAndMonomialBasis::AddSos(
    solvers::MathematicalProgram* prog,
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

solvers::MathematicalProgramResult SolveWithBackoff(
    solvers::MathematicalProgram* prog, std::optional<double> backoff_scale,
    const std::optional<solvers::SolverOptions>& solver_options,
    const solvers::SolverId& solver_id) {
  DRAKE_THROW_UNLESS(prog->quadratic_costs().size() == 0);
  auto solver = solvers::MakeSolver(solver_id);
  solvers::MathematicalProgramResult result;
  solver->Solve(*prog, std::nullopt, solver_options, &result);
  if (!result.is_success()) {
    drake::log()->debug("Failed before backoff.");
  }
  if (backoff_scale.has_value() && !(prog->linear_costs().empty())) {
    DRAKE_THROW_UNLESS(prog->linear_costs().size() == 1);
    const double cost_val = result.get_optimal_cost();
    const double cost_upper_bound =
        cost_val > 0 ? (1 + backoff_scale.value()) * cost_val
                     : (1 - backoff_scale.value()) * cost_val;
    prog->AddLinearConstraint(
        prog->linear_costs()[0].evaluator()->a(), -kInf,
        cost_upper_bound - prog->linear_costs()[0].evaluator()->b(),
        prog->linear_costs()[0].variables());
    prog->RemoveCost(prog->linear_costs()[0]);
    solver->Solve(*prog, std::nullopt, solver_options, &result);
    if (!result.is_success()) {
      drake::log()->debug("Failed in backoff.");
    }
  }
  return result;
}

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
