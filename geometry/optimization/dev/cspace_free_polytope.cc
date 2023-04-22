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
}  // namespace

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
      if ((cost - prev_cost) / prev_cost < options.convergence_tol) {
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
                         .polytope()
                         .rows();
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
