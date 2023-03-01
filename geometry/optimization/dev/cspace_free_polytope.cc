#include "drake/geometry/optimization/cspace_free_polytope.h"

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
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
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

solvers::MathematicalProgramResult SolveWithBackoff(
    solvers::MathematicalProgram* prog, std::optional<double> backoff_scale,
    const std::optional<solvers::SolverOptions>& solver_options,
    const solvers::SolverId& solver_id) {
  DRAKE_DEMAND(prog->quadratic_costs().size() == 0);
  auto solver = solvers::MakeSolver(solver_id);
  solvers::MathematicalProgramResult result;
  solver->Solve(*prog, std::nullopt, solver_options, &result);
  if (!result.is_success()) {
    drake::log()->warn("Failed before backoff.");
  }
  if (backoff_scale.has_value() && !(prog->linear_costs().empty())) {
    DRAKE_DEMAND(prog->linear_costs().size() == 1);
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
      drake::log()->info("Failed in backoff.");
    }
  }
  return result;
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

// Returns the number of y_slack variables in `rational`.
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
}  // namespace

int CspaceFreePolytope::GetGramVarSizeForPolytopeSearchProgram(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    bool search_s_bounds_lagrangians) const {
  int ret = 0;
  auto count_gram_per_rational =
      [this, search_s_bounds_lagrangians, &ret](
          const symbolic::RationalFunction& rational,
          const std::array<VectorX<symbolic::Monomial>, 4>&
              monomial_basis_array) {
        // Each rational will add Lagrangian multipliers for s-s_lower and
        // s_upper-s (if search_s_bounds_lagrangian=true), together with one
        // sos that rational.numerator() - λ(s)ᵀ * (d - C*s) - λ_lower(s)ᵀ *
        // (s - s_lower) -λ_upper(s)ᵀ * (s_upper - s) is sos
        const int s_size = this->rational_forward_kin_.s().rows();
        const int num_sos =
            (1 + (search_s_bounds_lagrangians ? 2 * s_size : 0));
        const int num_y = GetNumYInRational(rational, this->y_slack_);
        ret += num_sos *
               GetGramVarSize(monomial_basis_array, this->with_cross_y_, num_y);
      };

  for (const auto& plane_geometries : plane_geometries_) {
    const auto& plane = separating_planes_[plane_geometries.plane_index];
    if (ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
            plane.positive_side_geometry->id(),
            plane.negative_side_geometry->id())) == 0) {
      const auto& monomial_basis_array_positive_side =
          this->map_body_to_monomial_basis_array_.at(
              SortedPair<multibody::BodyIndex>(
                  plane.expressed_body,
                  plane.positive_side_geometry->body_index()));
      for (const auto& rational : plane_geometries.positive_side_rationals) {
        count_gram_per_rational(rational, monomial_basis_array_positive_side);
      }
      const auto& monomial_basis_array_negative_side =
          this->map_body_to_monomial_basis_array_.at(
              SortedPair<multibody::BodyIndex>(
                  plane.expressed_body,
                  plane.negative_side_geometry->body_index()));
      for (const auto& rational : plane_geometries.negative_side_rationals) {
        count_gram_per_rational(rational, monomial_basis_array_negative_side);
      }
    }
  }
  return ret;
}

std::unique_ptr<solvers::MathematicalProgram>
CspaceFreePolytope::InitializePolytopeSearchProgram(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const MatrixX<symbolic::Variable>& C, const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::vector<std::optional<SeparationCertificateResult>>&
        certificates_vec,
    bool search_s_bounds_lagrangians, int gram_total_size,
    std::unordered_map<int, SeparationCertificate>* new_certificates_map)
    const {
  auto prog = std::make_unique<solvers::MathematicalProgram>();
  prog->AddIndeterminates(rational_forward_kin_.s());
  // Add the indeterminates y if we need to certify non-polytopic collision
  // geometry
  for (const auto& plane : separating_planes_) {
    if (ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
            plane.positive_side_geometry->id(),
            plane.negative_side_geometry->id())) == 0) {
      if (plane.positive_side_geometry->type() !=
              CIrisGeometryType::kPolytope ||
          plane.negative_side_geometry->type() !=
              CIrisGeometryType::kPolytope) {
        prog->AddIndeterminates(y_slack_);
        break;
      }
    }
  }

  prog->AddDecisionVariables(Eigen::Map<const VectorX<symbolic::Variable>>(
      C.data(), C.rows() * C.cols()));
  prog->AddDecisionVariables(d);
  const auto gram_vars = prog->NewContinuousVariables(gram_total_size, "Gram");

  // plane_to_certificate_map maps the plane index to the index of certificate
  // in certificates_vec. Namely
  // certificates_vec[plane_to_certificate_map[i]]->plane_index = i
  std::unordered_map<int, int> plane_to_certificate_map;
  for (int i = 0; i < static_cast<int>(certificates_vec.size()); ++i) {
    plane_to_certificate_map.emplace(certificates_vec[i]->plane_index, i);
  }
  const int s_size = rational_forward_kin_.s().rows();
  int gram_var_count = 0;
  for (int plane_index = 0;
       plane_index < static_cast<int>(separating_planes_.size());
       ++plane_index) {
    const auto& plane = separating_planes_[plane_index];
    const SortedPair<geometry::GeometryId> geometry_pair(
        plane.positive_side_geometry->id(), plane.negative_side_geometry->id());
    if (ignored_collision_pairs.count(geometry_pair) == 0) {
      prog->AddDecisionVariables(plane.decision_variables);
      const auto& certificate =
          certificates_vec[plane_to_certificate_map.at(plane_index)];
      DRAKE_DEMAND(certificate.has_value());
      DRAKE_DEMAND(certificate->plane_index == plane_index);
      SeparationCertificate* new_certificate = nullptr;
      if (new_certificates_map != nullptr) {
        auto insertion_pair =
            new_certificates_map->emplace(plane_index, SeparationCertificate());
        new_certificate = &(insertion_pair.first->second);
      }
      VectorX<symbolic::Polynomial> s_lower_lagrangians(s_size);
      VectorX<symbolic::Polynomial> s_upper_lagrangians(s_size);

      auto add_rationals_nonnegative_given_lagrangians =
          [this, &prog, &d_minus_Cs, &gram_vars, s_size,
           search_s_bounds_lagrangians, &gram_var_count, &s_lower_lagrangians,
           &s_upper_lagrangians](
              const std::vector<symbolic::RationalFunction>& rationals,
              const std::array<VectorX<symbolic::Monomial>, 4>&
                  monomial_basis_array,
              const std::vector<SeparatingPlaneLagrangians>& lagrangians_vec,
              std::vector<SeparatingPlaneLagrangians>* new_lagrangians_vec) {
            DRAKE_DEMAND(rationals.size() == lagrangians_vec.size());
            for (int i = 0; i < static_cast<int>(rationals.size()); ++i) {
              const int num_y = GetNumYInRational(rationals[i], this->y_slack_);
              const int num_gram_vars_per_sos = GetGramVarSize(
                  monomial_basis_array, this->with_cross_y_, num_y);
              GramAndMonomialBasis gram_and_monomial_basis(
                  monomial_basis_array, this->with_cross_y_, num_y);
              // Add Lagrangian multipliers for joint limits.
              if (search_s_bounds_lagrangians) {
                for (int j = 0; j < s_size; ++j) {
                  gram_and_monomial_basis.AddSos(
                      prog.get(),
                      gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
                      &(s_lower_lagrangians(j)));
                  gram_var_count += num_gram_vars_per_sos;
                  gram_and_monomial_basis.AddSos(
                      prog.get(),
                      gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
                      &(s_upper_lagrangians(j)));
                  gram_var_count += num_gram_vars_per_sos;
                }
              } else {
                s_lower_lagrangians = lagrangians_vec[i].s_lower;
                s_upper_lagrangians = lagrangians_vec[i].s_upper;
              }

              if (new_lagrangians_vec != nullptr) {
                new_lagrangians_vec->emplace_back(d_minus_Cs.rows(), s_size);
                new_lagrangians_vec->back().polytope =
                    lagrangians_vec[i].polytope;
                new_lagrangians_vec->back().s_lower = s_lower_lagrangians;
                new_lagrangians_vec->back().s_upper = s_upper_lagrangians;
              }

              const symbolic::Polynomial poly =
                  rationals[i].numerator() -
                  lagrangians_vec[i].polytope.dot(d_minus_Cs) -
                  s_lower_lagrangians.dot(this->s_minus_s_lower_) -
                  s_upper_lagrangians.dot(this->s_upper_minus_s_);
              symbolic::Polynomial poly_sos;
              gram_and_monomial_basis.AddSos(
                  prog.get(),
                  gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
                  &poly_sos);
              gram_var_count += num_gram_vars_per_sos;
              prog->AddEqualityConstraintBetweenPolynomials(poly, poly_sos);
            }
          };

      // Add the constraint that positive_side_rationals are nonnegative in
      // C-space polytope.
      const auto& monomial_basis_array_positive_side =
          this->map_body_to_monomial_basis_array_.at(
              SortedPair<multibody::BodyIndex>(
                  plane.expressed_body,
                  plane.positive_side_geometry->body_index()));
      add_rationals_nonnegative_given_lagrangians(
          plane_geometries_[plane_index].positive_side_rationals,
          monomial_basis_array_positive_side,
          certificate->positive_side_rational_lagrangians,
          new_certificate == nullptr
              ? nullptr
              : &(new_certificate->positive_side_rational_lagrangians));

      // Add the constraint that negative_side_rationals are nonnegative in
      // C-space polytope.
      const auto& monomial_basis_array_negative_side =
          this->map_body_to_monomial_basis_array_.at(
              SortedPair<multibody::BodyIndex>(
                  plane.expressed_body,
                  plane.negative_side_geometry->body_index()));
      add_rationals_nonnegative_given_lagrangians(
          plane_geometries_[plane_index].negative_side_rationals,
          monomial_basis_array_negative_side,
          certificate->negative_side_rational_lagrangians,
          new_certificate == nullptr
              ? nullptr
              : &(new_certificate->negative_side_rational_lagrangians));
    }
  }
  DRAKE_DEMAND(gram_var_count == gram_total_size);
  return prog;
}

void CspaceFreePolytope::AddEllipsoidContainmentConstraint(
    solvers::MathematicalProgram* prog, const Eigen::MatrixXd& Q,
    const Eigen::VectorXd& s0, const MatrixX<symbolic::Variable>& C,
    const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Variable>& ellipsoid_margins) const {
  DRAKE_DEMAND(Q.rows() == Q.cols());
  DRAKE_DEMAND((s0.array() <= s_upper_.array()).all());
  DRAKE_DEMAND((s0.array() >= s_lower_.array()).all());
  // Add the constraint |cᵢᵀQ|₂ ≤ dᵢ − cᵢᵀs0 − δᵢ as a Lorentz cone
  // constraint, namely [dᵢ − cᵢᵀs0 − δᵢ, cᵢᵀQ] is in the Lorentz cone. [dᵢ
  // − cᵢᵀs0 − δᵢ, cᵢᵀQ] = A_lorentz1 * [cᵢ, dᵢ, δᵢ] + b_lorentz1
  Eigen::MatrixXd A_lorentz1(Q.rows() + 1, 2 + C.cols());
  Eigen::VectorXd b_lorentz1(Q.rows() + 1);
  VectorX<symbolic::Variable> lorentz1_vars(2 + C.cols());
  for (int i = 0; i < C.rows(); ++i) {
    A_lorentz1.setZero();
    A_lorentz1.block(0, 0, 1, C.cols()) = -s0.transpose();
    A_lorentz1(0, C.cols()) = 1;
    A_lorentz1(0, C.cols() + 1) = -1;
    A_lorentz1.block(1, 0, Q.rows(), Q.cols()) = Q;
    b_lorentz1.setZero();
    lorentz1_vars << C.row(i).transpose(), d(i), ellipsoid_margins(i);
    prog->AddLorentzConeConstraint(A_lorentz1, b_lorentz1, lorentz1_vars);
  }
  // Add the constraint |cᵢ|₂ ≤ 1 as a Lorentz cone constraint that [1,
  // cᵢ] is in the Lorentz cone. [1, cᵢ] = A_lorentz2 * cᵢ + b_lorentz2
  Eigen::MatrixXd A_lorentz2 = Eigen::MatrixXd::Zero(1 + C.cols(), C.cols());
  A_lorentz2.bottomRows(C.cols()) =
      Eigen::MatrixXd::Identity(C.cols(), C.cols());
  Eigen::VectorXd b_lorentz2 = Eigen::VectorXd::Zero(1 + C.cols());
  b_lorentz2(0) = 1;
  for (int i = 0; i < C.rows(); ++i) {
    prog->AddLorentzConeConstraint(A_lorentz2, b_lorentz2,
                                   C.row(i).transpose());
  }
}

void CspaceFreePolytope::AddCspacePolytopeContainment(
    solvers::MathematicalProgram* prog, const MatrixX<symbolic::Variable>& C,
    const VectorX<symbolic::Variable>& d,
    const Eigen::MatrixXd& s_inner_pts) const {
  DRAKE_DEMAND(s_inner_pts.rows() == this->rational_forward_kin_.s().rows());
  // Check that s_inner_pts is within [s_lower_, s_upper_].
  for (int i = 0; i < s_inner_pts.rows(); ++i) {
    for (int j = 0; j < s_inner_pts.cols(); ++j) {
      if (s_inner_pts(i, j) > s_upper_(i)) {
        throw std::runtime_error(
            fmt::format("AddCspacePolytopeContainment(): s_inner_pts({}, "
                        "{})={}, larger than s_upper({})={}",
                        i, j, s_inner_pts(i, j), i, s_upper_(i)));
      }
      if (s_inner_pts(i, j) < s_lower_(i)) {
        throw std::runtime_error(
            fmt::format("AddCspacePolytopeContainment(): s_inner_pts({}, "
                        "{})={}, smaller than s_lower({})={}",
                        i, j, s_inner_pts(i, j), i, s_lower_(i)));
      }
    }
  }
  // We have the constraint C.row(i).dot(s_inner_pts.col(j)) <= d(i) for all
  // i, j. We can write this as s_inner_ptsᵀ * C.row(i)ᵀ <= [d(i);...;d(i)] We
  // repeat this constraint for each row and concantenate it into the matrix
  // form blockdiag(s_inner_ptsᵀ, ..., s_inner_ptsᵀ) * [C.row(0)ᵀ;
  // C.row(1)ᵀ;...;C.row(n-1)] - blockdiag(𝟏, 𝟏, ..., 𝟏) * d <= 0
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
      s_inner_pts.cols() * C.rows(), (s_inner_pts.rows() + 1) * C.rows());
  VectorX<symbolic::Variable> vars(A.cols());
  for (int i = 0; i < C.rows(); ++i) {
    A.block(i * s_inner_pts.cols(), i * (s_inner_pts.rows() + 1),
            s_inner_pts.cols(), s_inner_pts.rows()) = s_inner_pts.transpose();
    A.block(i * s_inner_pts.cols(),
            i * (s_inner_pts.rows() + 1) + s_inner_pts.rows(),
            s_inner_pts.cols(), 1) = -Eigen::VectorXd::Ones(s_inner_pts.cols());
    vars.segment((s_inner_pts.rows() + 1) * i, s_inner_pts.rows()) =
        C.row(i).transpose();
    vars((s_inner_pts.rows() + 1) * i + s_inner_pts.rows()) = d(i);
  }
  prog->AddLinearConstraint(A, Eigen::VectorXd::Constant(A.rows(), -kInf),
                            Eigen::VectorXd::Zero(A.rows()), vars);
}

std::optional<CspaceFreePolytope::FindPolytopeGivenLagrangianResult>
CspaceFreePolytope::FindPolytopeGivenLagrangian(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const MatrixX<symbolic::Variable>& C, const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::vector<std::optional<SeparationCertificateResult>>&
        certificates_vec,
    const Eigen::MatrixXd& Q, const Eigen::VectorXd& s0,
    const VectorX<symbolic::Variable>& ellipsoid_margins, int gram_total_size,
    const FindPolytopeGivenLagrangianOptions& options,
    std::unordered_map<int, SeparationCertificateResult>* certificates_result)
    const {
  std::unordered_map<int, SeparationCertificate> new_certificates_map;
  auto prog = this->InitializePolytopeSearchProgram(
      ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec,
      options.search_s_bounds_lagrangians, gram_total_size,
      certificates_result == nullptr ? nullptr : &new_certificates_map);
  prog->AddDecisionVariables(ellipsoid_margins);
  AddEllipsoidContainmentConstraint(prog.get(), Q, s0, C, d, ellipsoid_margins);
  // We know that the verified polytope has to be contained in the box
  // s_lower <= s <= s_upper. Hence there is no point to grow the polytope such
  // that any of its halfspace C.row(i) * s <= d(i) contains the entire box
  // s_lower <= s <= s_upper. Therefore an upper bound of the margin δ is the
  // maximal distance from any vertices of the box s_lower <= s <= s_upper to
  // the ellipsoid. Computing the distance from a point to the hyperellipsoid is
  // non-trivial (there is not closed-form solution). Here we use an upper
  // bound of this distance, which is the maximal distance between any two
  // points within the box.
  const double margin_upper_bound = (s_upper_ - s_lower_).norm();
  prog->AddBoundingBoxConstraint(0, margin_upper_bound, ellipsoid_margins);
  if (options.s_inner_pts.has_value()) {
    for (int i = 0; i < options.s_inner_pts->cols(); ++i) {
      DRAKE_DEMAND(
          (options.s_inner_pts->col(i).array() <= s_upper_.array()).all());
      DRAKE_DEMAND(
          (options.s_inner_pts->col(i).array() >= s_lower_.array()).all());
    }
    // Add the constraint C * s_inner_pts <= d
    AddCspacePolytopeContainment(prog.get(), C, d, options.s_inner_pts.value());
  }

  switch (options.ellipsoid_margin_cost) {
    case CspaceFreePolytope::EllipsoidMarginCost::kSum:
      // Maximize ∑ᵢ δᵢ
      prog->AddLinearCost(-Eigen::VectorXd::Ones(ellipsoid_margins.rows()), 0,
                          ellipsoid_margins);
      break;
    case CspaceFreePolytope::EllipsoidMarginCost::kGeometricMean:
      // Maximize ∏ᵢ (δᵢ + ε)
      prog->AddMaximizeGeometricMeanCost(
          Eigen::MatrixXd::Identity(ellipsoid_margins.rows(),
                                    ellipsoid_margins.rows()),
          Eigen::VectorXd::Constant(ellipsoid_margins.rows(),
                                    options.ellipsoid_margin_epsilon),
          ellipsoid_margins);
      break;
  }

  const solvers::MathematicalProgramResult result =
      SolveWithBackoff(prog.get(), options.backoff_scale,
                       options.solver_options, options.solver_id);
  if (result.is_success()) {
    CspaceFreePolytope::FindPolytopeGivenLagrangianResult ret;
    ret.C = result.GetSolution(C);
    ret.d = result.GetSolution(d);
    for (int plane_index = 0;
         plane_index < static_cast<int>(this->separating_planes_.size());
         ++plane_index) {
      const auto& plane = this->separating_planes_[plane_index];
      const SortedPair<geometry::GeometryId> geometry_pair(
          plane.positive_side_geometry->id(),
          plane.negative_side_geometry->id());
      if (ignored_collision_pairs.count(geometry_pair) == 0) {
        Vector3<symbolic::Polynomial> a;
        for (int i = 0; i < 3; ++i) {
          a(i) = result.GetSolution(plane.a(i));
        }
        ret.a.emplace(plane_index, a);
        ret.b.emplace(plane_index, result.GetSolution(plane.b));
        ret.ellipsoid_margins = result.GetSolution(ellipsoid_margins);
      }
    }

    if (certificates_result != nullptr) {
      certificates_result->clear();
      for (int plane_index = 0;
           plane_index < static_cast<int>(separating_planes_.size());
           ++plane_index) {
        const auto& plane = separating_planes_[plane_index];
        if (ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
                plane.positive_side_geometry->id(),
                plane.negative_side_geometry->id())) == 0) {
          certificates_result->emplace(
              plane_index,
              new_certificates_map.at(plane_index)
                  .GetSolution(
                      plane_index, separating_planes_[plane_index].a,
                      separating_planes_[plane_index].b,
                      separating_planes_[plane_index].decision_variables,
                      result));
        }
      }
    }
    return ret;
  } else {
    return std::nullopt;
  }
}

HPolyhedron CspaceFreePolytope::GetPolyhedronWithJointLimits(
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d) const {
  const int s_size = rational_forward_kin_.s().rows();
  Eigen::MatrixXd A(C.rows() + 2 * s_size, s_size);
  Eigen::VectorXd b(A.rows());
  A.topRows(C.rows()) = C;
  b.head(C.rows()) = d;
  A.middleRows(C.rows(), s_size) = Eigen::MatrixXd::Identity(s_size, s_size);
  b.segment(C.rows(), s_size) = s_upper_;
  A.bottomRows(s_size) = -Eigen::MatrixXd::Identity(s_size, s_size);
  b.tail(s_size) = -s_lower_;
  return HPolyhedron(A, b);
}

void CspaceFreePolytope::SearchResult::SetSeparatingPlanes(
    const std::vector<std::optional<SeparationCertificateResult>>&
        certificates_result) {
  a.clear();
  b.clear();
  for (const auto& certificate : certificates_result) {
    a.emplace(certificate->plane_index, certificate->a);
    b.emplace(certificate->plane_index, certificate->b);
  }
}

void CspaceFreePolytope::SearchResult::UpdateSeparatingPlanes(
    const std::vector<std::optional<SeparationCertificateResult>>&
        certificates_result) {
  for (const auto& certificate : certificates_result) {
    if (certificate.has_value()) {
      auto it_a = a.find(certificate->plane_index);
      if (it_a == a.end()) {
        a.emplace_hint(it_a, certificate->plane_index, certificate->a);
      } else {
        it_a->second = certificate->a;
      }
      auto it_b = b.find(certificate->plane_index);
      if (it_b == b.end()) {
        b.emplace_hint(it_b, certificate->plane_index, certificate->b);
      } else {
        it_b->second = certificate->b;
      }
    }
  }
}

std::vector<CspaceFreePolytope::SearchResult>
CspaceFreePolytope::SearchWithBilinearAlternation(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const Eigen::Ref<const Eigen::MatrixXd>& C_init,
    const Eigen::Ref<const Eigen::VectorXd>& d_init,
    const BilinearAlternationOptions& options) const {
  DRAKE_DEMAND(C_init.rows() == d_init.rows());
  DRAKE_DEMAND(C_init.cols() == this->rational_forward_kin_.s().rows());
  std::vector<CspaceFreePolytope::SearchResult> ret{};
  int iter = 0;
  // When we search for the C-space polytope {s |C*s<=d, s_lower<=s<=s_upper},
  // we will require that each row of C has length <= 1. Hence to start with a
  // feasible solution, we normalize each row of C and d.
  Eigen::MatrixXd C = C_init;
  Eigen::VectorXd d = d_init;
  for (int i = 0; i < C.rows(); ++i) {
    const double C_row_norm = C.row(i).norm();
    C.row(i) = C.row(i) / C_row_norm;
    d(i) = d(i) / C_row_norm;
  }
  // Create symbolic variable for C and d
  MatrixX<symbolic::Variable> C_var(C_init.rows(), C_init.cols());
  VectorX<symbolic::Variable> d_var(d_init.rows());
  // Create symbolic variable for ellipsoid margins.
  VectorX<symbolic::Variable> ellipsoid_margins(C_init.rows());
  for (int i = 0; i < C_init.rows(); ++i) {
    d_var(i) = symbolic::Variable("d" + std::to_string(i));
    ellipsoid_margins(i) =
        symbolic::Variable("ellipsoid_margin" + std::to_string(i));
    for (int j = 0; j < C_init.cols(); ++j) {
      C_var(i, j) = symbolic::Variable(fmt::format("C({},{})", i, j));
    }
  }
  const VectorX<symbolic::Polynomial> d_minus_Cs =
      this->CalcDminusCs<symbolic::Variable>(C_var, d_var);
  const int gram_total_size_in_polytope_program =
      this->GetGramVarSizeForPolytopeSearchProgram(
          ignored_collision_pairs,
          options.find_polytope_options.search_s_bounds_lagrangians);
  // Find the inscribed ellipsoid.
  HPolyhedron cspace_polytope = this->GetPolyhedronWithJointLimits(C, d);
  Hyperellipsoid ellipsoid = cspace_polytope.MaximumVolumeInscribedEllipsoid();
  DRAKE_DEMAND(options.ellipsoid_scaling > 0);
  DRAKE_DEMAND(options.ellipsoid_scaling <= 1);
  Eigen::MatrixXd ellipsoid_Q =
      options.ellipsoid_scaling * (ellipsoid.A().inverse());
  double prev_cost = ellipsoid_Q.determinant();
  drake::log()->info("det(Q) at the beginning is {}", prev_cost);
  while (iter < options.max_iter) {
    const std::vector<std::optional<SeparationCertificateResult>>
        certificates_result = this->FindSeparationCertificateGivenPolytope(
            ignored_collision_pairs, C, d, options.find_lagrangian_options);
    if (std::any_of(
            certificates_result.begin(), certificates_result.end(),
            [](const std::optional<SeparationCertificateResult>& certificate) {
              return !certificate.has_value();
            })) {
      drake::log()->error(
          "Cannot find the separation certificate at iteration {} given the "
          "polytope.",
          iter);
      break;
    } else {
      ret.emplace_back();
      ret.back().C = C;
      ret.back().d = d;
      ret.back().num_iter = iter;
      ret.back().certified_polytope = GetPolyhedronWithJointLimits(C, d);
      ret.back().SetSeparatingPlanes(certificates_result);
    }
    // Now fix the Lagrangian and search for C-space polytope and separating
    // planes.
    const auto polytope_result = this->FindPolytopeGivenLagrangian(
        ignored_collision_pairs, C_var, d_var, d_minus_Cs, certificates_result,
        ellipsoid_Q, ellipsoid.center(), ellipsoid_margins,
        gram_total_size_in_polytope_program, options.find_polytope_options,
        nullptr /* certificates_result */);
    if (polytope_result.has_value()) {
      C = polytope_result->C;
      d = polytope_result->d;
      ret.back().C = polytope_result->C;
      ret.back().d = polytope_result->d;
      ret.back().certified_polytope =
          GetPolyhedronWithJointLimits(polytope_result->C, polytope_result->d);
      ret.back().a = std::move(polytope_result->a);
      ret.back().b = std::move(polytope_result->b);
      ret.back().num_iter = iter;
      // Now find the inscribed ellipsoid.
      ellipsoid =
          ret.back().certified_polytope.MaximumVolumeInscribedEllipsoid();
      ellipsoid_Q = options.ellipsoid_scaling * (ellipsoid.A().inverse());
      const double cost = ellipsoid_Q.determinant();
      drake::log()->info("Iteration {}: det(Q)={}", iter, cost);
      if ((cost - prev_cost)/prev_cost < options.convergence_tol) {
        break;
      } else {
        prev_cost = cost;
      }
    } else {
      drake::log()->error(
          "Cannot find the separation certificate at iteration {} given the "
          "Lagrangians.",
          iter);
      break;
    }
    ++iter;
  }
  return ret;
}

std::optional<CspaceFreePolytope::SearchResult>
CspaceFreePolytope::BinarySearch(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d_init,
    const Eigen::Ref<const Eigen::VectorXd>& s_center,
    const BinarySearchOptions& options) const {
  DRAKE_DEMAND(options.scale_max >= options.scale_min &&
               options.scale_min >= 0);
  DRAKE_DEMAND(((C * s_center).array() <= d_init.array()).all());
  DRAKE_DEMAND((s_center.array() >= s_lower_.array()).all());
  DRAKE_DEMAND((s_center.array() <= s_upper_.array()).all());
  CspaceFreePolytope::SearchResult ret;

  const Eigen::ArrayXd C_row_norm = C.rowwise().norm().array();
  if ((C_row_norm == 0).any()) {
    throw std::runtime_error(
        "C contains rows with all 0 entries. Please remove these rows.");
  }

  // geometry_pair_scale_lower_bounds[i] stores the certified lower bound on
  // the scaling factor for the i'th pair of geometries (the geometries in
  // this->separating_planes_[i]).
  std::vector<double> geometry_pair_scale_lower_bounds(
      this->separating_planes_.size(), 0);

  // Determines if we can certify the scaled C-space polytope {s |C*s<=d,
  // s_lower<=s<=s_upper} is collision free or not. Also update `ret` if eps
  // is feasible.
  auto is_scale_feasible = [this, &ignored_collision_pairs, &C, &d_init,
                            &s_center, &options,
                            &geometry_pair_scale_lower_bounds,
                            &ret](double scale) {
    // (d - C*s_center) / |C| = scale * (d_init - C*s_center) / |C|, hence d =
    // scale * d_init + (1-scale) * C * s_center.
    const Eigen::VectorXd d = scale * d_init + (1 - scale) * C * s_center;

    // If `scale` is smaller than geometry_pair_scale_lower_bounds[plane_index],
    // then it means that in the previous iteration of the binary search, we
    // have already certified this pair of geometry is separated for a larger
    // scale (hence a larger C-space free region), and we don't need to certify
    // the separation for this `scale` (hence we add the pair to
    // `ignored_collision_pairs_for_scale`). Only attempt to certify the
    // separating plane for this pair of geometry, if `scale` is larger than
    // geometry_pair_scale_lower_bounds[plane_index].
    CspaceFreePolytope::IgnoredCollisionPairs
        ignored_collision_pairs_for_scale = ignored_collision_pairs;
    for (int i = 0; i < static_cast<int>(separating_planes_.size()); ++i) {
      const auto& plane = separating_planes_[i];
      const SortedPair<geometry::GeometryId> geometry_pair(
          plane.positive_side_geometry->id(),
          plane.negative_side_geometry->id());
      if (ignored_collision_pairs.count(geometry_pair) == 0 &&
          geometry_pair_scale_lower_bounds[i] >= scale) {
        ignored_collision_pairs_for_scale.insert(geometry_pair);
      }
    }
    const auto certificates_result =
        this->FindSeparationCertificateGivenPolytope(
            ignored_collision_pairs_for_scale, C, d,
            options.find_lagrangian_options);
    for (const auto& certificate_result : certificates_result) {
      if (certificate_result.has_value()) {
        // If `scale` is feasible for this pair of geometry, then update the
        // lower bound stored in geometry_pair_scale_lower_bounds.
        geometry_pair_scale_lower_bounds[certificate_result->plane_index] =
            scale;
      }
    }

    if (std::any_of(
            certificates_result.begin(), certificates_result.end(),
            [](const std::optional<SeparationCertificateResult>& certificate) {
              return !certificate.has_value();
            })) {
      // We might have found the certificates for some (but not all) geometry
      // pairs, so we still update the separation planes for these certified
      // pairs.
      ret.UpdateSeparatingPlanes(certificates_result);
      return false;
    } else {
      ret.C = C;
      ret.d = d;
      ret.certified_polytope = this->GetPolyhedronWithJointLimits(C, d);
      ret.UpdateSeparatingPlanes(certificates_result);
      return true;
    }
  };

  if (!is_scale_feasible(options.scale_min)) {
    drake::log()->error(
        "CspaceFreePolytope::BinarySearch(): scale_min={} is infeasible.",
        options.scale_min);
    return std::nullopt;
  }
  if (is_scale_feasible(options.scale_max)) {
    drake::log()->info(
        "CspaceFreePolytope::BinarySearch(): scale_max={} is feasible.",
        options.scale_max);
    ret.num_iter = 0;
    return ret;
  }
  double scale_min = options.scale_min;
  double scale_max = options.scale_max;
  int iter = 0;
  while (scale_max - scale_min > options.convergence_tol &&
         iter < options.max_iter) {
    const double scale = (scale_max + scale_min) / 2;
    if (is_scale_feasible(scale)) {
      drake::log()->info(
          "CspaceFreePolytope::BinarySearch(): scale={} is feasible", scale);
      scale_min = scale;
    } else {
      drake::log()->info(
          "CspaceFreePolytope::BinarySearch(): scale={} is infeasible", scale);
      scale_max = scale;
    }
    ++iter;
  }
  ret.num_iter = iter;
  return ret;
}

std::unique_ptr<solvers::MathematicalProgram>
CspaceFreePolytope::InitializePolytopeSearchProgram(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const std::unordered_map<SortedPair<geometry::GeometryId>,
                             SeparationCertificateResult>& certificates,
    bool search_s_bounds_lagrangians, MatrixX<symbolic::Variable>* C,
    VectorX<symbolic::Variable>* d,
    std::unordered_map<int, SeparationCertificate>* new_certificates) const {
  const int s_size = rational_forward_kin_.s().rows();
  const int C_rows = certificates.begin()
                         ->second.positive_side_rational_lagrangians[0]
                         .polytope.rows();
  C->resize(C_rows, s_size);
  d->resize(C_rows);
  for (int i = 0; i < C_rows; ++i) {
    for (int j = 0; j < s_size; ++j) {
      (*C)(i, j) = symbolic::Variable(fmt::format("C({},{})", i, j));
    }
    (*d)(i) = symbolic::Variable(fmt::format("d{}", i));
  }
  const auto d_minus_Cs = this->CalcDminusCs<symbolic::Variable>(*C, *d);
  // In order to get consistent result, I put element into certificates_vec in a
  // sorted order, based on the plane index.
  std::vector<std::optional<SeparationCertificateResult>> certificates_vec;
  for (const auto& plane : separating_planes_) {
    const SortedPair<geometry::GeometryId> geometry_pair(
        plane.positive_side_geometry->id(), plane.negative_side_geometry->id());
    if (ignored_collision_pairs.count(geometry_pair) == 0) {
      const auto it = certificates.find(geometry_pair);
      if (it == certificates.end()) {
        const auto& inspector = scene_graph_.model_inspector();
        throw std::runtime_error(
            fmt::format("InitializePolytopeSearchProgram: certificates doesn't "
                        "contain result for the geometry pair ({}, {})",
                        inspector.GetName(geometry_pair.first()),
                        inspector.GetName(geometry_pair.second())));
      }
      certificates_vec.emplace_back(it->second);
    }
  }
  const int gram_total_size = this->GetGramVarSizeForPolytopeSearchProgram(
      ignored_collision_pairs, search_s_bounds_lagrangians);
  return this->InitializePolytopeSearchProgram(
      ignored_collision_pairs, *C, *d, d_minus_Cs, certificates_vec,
      search_s_bounds_lagrangians, gram_total_size, new_certificates);
}

CspaceFreePolytope::SeparationCertificateProgram
CspaceFreePolytope::MakeIsGeometrySeparableProgram(
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d) const {
  const auto d_minus_Cs = this->CalcDminusCs<double>(C, d);
  auto geometry_pair_it =
      map_geometries_to_separating_planes_.find(geometry_pair);
  if (geometry_pair_it == map_geometries_to_separating_planes_.end()) {
    throw std::runtime_error(fmt::format(
        "GetIsGeometrySeparableProgram(): geometry pair ({}, {}) does not need "
        "a separation certificate",
        scene_graph_.model_inspector().GetName(geometry_pair.first()),
        scene_graph_.model_inspector().GetName(geometry_pair.second())));
  }
  const int plane_index = geometry_pair_it->second;

  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  this->FindRedundantInequalities(C, d, this->s_lower_, this->s_upper_,
                                  0 /* tighten */, &C_redundant_indices,
                                  &s_lower_redundant_indices,
                                  &s_upper_redundant_indices);
  return this->ConstructPlaneSearchProgram(
      this->plane_geometries_[plane_index], d_minus_Cs, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

std::optional<CspaceFreePolytope::SeparationCertificateResult>
CspaceFreePolytope::SolveSeparationCertificateProgram(
    const CspaceFreePolytope::SeparationCertificateProgram& certificate_program,
    const FindSeparationCertificateGivenPolytopeOptions& options) const {
  solvers::MathematicalProgramResult result;
  solvers::MakeSolver(options.solver_id)
      ->Solve(*certificate_program.prog, std::nullopt, options.solver_options,
              &result);
  std::optional<CspaceFreePolytope::SeparationCertificateResult> ret{
      std::nullopt};
  if (result.is_success()) {
    ret.emplace(certificate_program.certificate.GetSolution(
        certificate_program.plane_index,
        separating_planes_[certificate_program.plane_index].a,
        separating_planes_[certificate_program.plane_index].b,
        separating_planes_[certificate_program.plane_index].decision_variables,
        result));
  }
  return ret;
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
