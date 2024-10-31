#include "drake/geometry/optimization/cspace_free_box.h"

#include <array>
#include <limits>
#include <map>
#include <memory>
#include <unordered_set>
#include <utility>

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/optimization/cspace_free_internal.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {
const double kInf = std::numeric_limits<double>::infinity();
}  // namespace

CspaceFreeBox::SeparatingPlaneLagrangians::~SeparatingPlaneLagrangians() =
    default;

CspaceFreeBox::SeparationCertificateResult::~SeparationCertificateResult() =
    default;

CspaceFreeBox::SeparationCertificate::~SeparationCertificate() = default;

CspaceFreeBox::SeparationCertificateResult
CspaceFreeBox::SeparationCertificate::GetSolution(
    int plane_index, const Vector3<symbolic::Polynomial>& a,
    const symbolic::Polynomial& b,
    const VectorX<symbolic::Variable>& plane_decision_vars,
    const solvers::MathematicalProgramResult& result) const {
  CspaceFreeBox::SeparationCertificateResult ret{};
  ret.plane_index = plane_index;

  auto set_lagrangians =
      [&result](const std::vector<CspaceFreeBox::SeparatingPlaneLagrangians>&
                    lagrangians_vec,
                std::vector<CspaceFreeBox::SeparatingPlaneLagrangians>*
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

CspaceFreeBox::SeparationCertificateProgram::~SeparationCertificateProgram() =
    default;

CspaceFreeBox::CspaceFreeBox(const multibody::MultibodyPlant<double>* plant,
                             const geometry::SceneGraph<double>* scene_graph,
                             SeparatingPlaneOrder plane_order,
                             const Options& options)
    : CspaceFreePolytopeBase(plant, scene_graph, plane_order,
                             CspaceFreePolytopeBase::SForPlane::kOnChain,
                             options) {}

bool CspaceFreeBox::FindSeparationCertificateGivenBox(
    const Eigen::Ref<const Eigen::VectorXd>& q_box_lower,
    const Eigen::Ref<const Eigen::VectorXd>& q_box_upper,
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const FindSeparationCertificateOptions& options,
    std::unordered_map<SortedPair<geometry::GeometryId>,
                       SeparationCertificateResult>* certificates) const {
  std::vector<std::optional<SeparationCertificateResult>> certificates_vec =
      this->FindSeparationCertificateGivenBox(
          ignored_collision_pairs, q_box_lower, q_box_upper, options);

  certificates->clear();
  bool is_success = true;
  for (const auto& certificate : certificates_vec) {
    if (certificate.has_value()) {
      const auto& plane = separating_planes()[certificate->plane_index];
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

CspaceFreeBox::~CspaceFreeBox() {}

CspaceFreeBox::SeparatingPlaneLagrangians
CspaceFreeBox::SeparatingPlaneLagrangians::GetSolution(
    const solvers::MathematicalProgramResult& result) const {
  CspaceFreeBox::SeparatingPlaneLagrangians ret(this->s_box_lower_.rows());
  for (int i = 0; i < this->s_box_lower_.rows(); ++i) {
    ret.s_box_lower_(i) = result.GetSolution(this->s_box_lower_(i));
    ret.s_box_upper_(i) = result.GetSolution(this->s_box_upper_(i));
  }
  return ret;
}

void CspaceFreeBox::ComputeSBox(
    const Eigen::Ref<const Eigen::VectorXd>& q_box_lower,
    const Eigen::Ref<const Eigen::VectorXd>& q_box_upper,
    Eigen::VectorXd* s_box_lower, Eigen::VectorXd* s_box_upper,
    Eigen::VectorXd* q_star) const {
  if ((q_box_lower.array() > q_box_upper.array()).any()) {
    throw std::runtime_error(
        fmt::format("CspaceFreeBox: q_box_lower={} has some entries larger "
                    "than q_box_upper={}.",
                    fmt_eigen(q_box_lower.transpose()),
                    fmt_eigen(q_box_upper.transpose())));
  }
  const auto& plant = this->rational_forward_kin().plant();
  const Eigen::VectorXd q_position_lower = plant.GetPositionLowerLimits();
  const Eigen::VectorXd q_position_upper = plant.GetPositionUpperLimits();
  if ((q_box_lower.array() > q_position_upper.array()).any()) {
    throw std::runtime_error(fmt::format(
        "CspaceFreeBox: q_box_lower={} has some entries larger the the robot "
        "position upper limit={}.",
        fmt_eigen(q_box_lower.transpose()),
        fmt_eigen(q_position_upper.transpose())));
  }
  if ((q_box_upper.array() < q_position_lower.array()).any()) {
    throw std::runtime_error(fmt::format(
        "CspaceFreeBox: q_box_upper={} has some entries smaller than the robot "
        "position lower limit={}.",
        fmt_eigen(q_box_upper.transpose()),
        fmt_eigen(q_position_lower.transpose())));
  }
  const Eigen::VectorXd q_lower =
      q_box_lower.array().max(q_position_lower.array()).matrix();
  const Eigen::VectorXd q_upper =
      q_box_upper.array().min(q_position_upper.array()).matrix();
  *q_star = 0.5 * (q_lower + q_upper);
  *s_box_lower = this->rational_forward_kin().ComputeSValue(q_lower, *q_star);
  *s_box_upper = this->rational_forward_kin().ComputeSValue(q_upper, *q_star);
}

void CspaceFreeBox::GeneratePolynomialsToCertify(
    const Eigen::Ref<const Eigen::VectorXd>& s_box_lower,
    const Eigen::Ref<const Eigen::VectorXd>& s_box_upper,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const IgnoredCollisionPairs& ignored_collision_pairs,
    PolynomialsToCertify* certify_polynomials) const {
  this->CalcSBoundsPolynomial<double>(
      s_box_lower, s_box_upper, &(certify_polynomials->s_minus_s_box_lower),
      &(certify_polynomials->s_box_upper_minus_s));

  std::map<int, const CSpaceSeparatingPlane<symbolic::Variable>*>
      separating_planes_map;
  for (int i = 0; i < static_cast<int>(separating_planes().size()); ++i) {
    const auto& plane = separating_planes()[i];
    if (!ignored_collision_pairs.contains(SortedPair<geometry::GeometryId>(
            plane.positive_side_geometry->id(),
            plane.negative_side_geometry->id()))) {
      separating_planes_map.emplace(i, &plane);
    }
  }

  internal::GenerateRationals(separating_planes_map, y_slack(), q_star,
                              rational_forward_kin(),
                              &(certify_polynomials->plane_geometries));
}

CspaceFreeBox::SeparationCertificateProgram
CspaceFreeBox::ConstructPlaneSearchProgram(
    const PlaneSeparatesGeometries& plane_geometries,
    const VectorX<symbolic::Polynomial>& s_minus_s_lower,
    const VectorX<symbolic::Polynomial>& s_upper_minus_s) const {
  SeparationCertificateProgram ret;
  ret.plane_index = plane_geometries.plane_index;
  const auto& plane = separating_planes()[plane_geometries.plane_index];
  const SortedPair<multibody::BodyIndex> body_pair{
      plane.positive_side_geometry->body_index(),
      plane.negative_side_geometry->body_index()};
  const std::vector<int> s_for_plane_indices =
      this->map_body_pair_to_s_on_chain().at(body_pair);
  const std::unordered_set<int> s_for_plane_indices_set{
      s_for_plane_indices.begin(), s_for_plane_indices.end()};
  VectorX<symbolic::Variable> s_for_plane(s_for_plane_indices.size());
  for (int i = 0; i < ssize(s_for_plane_indices); ++i) {
    s_for_plane(i) = this->rational_forward_kin().s()(s_for_plane_indices[i]);
  }
  ret.prog->AddIndeterminates(s_for_plane);
  ret.prog->AddDecisionVariables(plane.decision_variables);

  // First count the total size of the gram matrix variables.
  int gram_var_count = 0;
  auto count_gram = [this, &s_for_plane_indices](
                        const symbolic ::RationalFunction& rational,
                        const std::array<VectorX<symbolic::Monomial>, 4>&
                            monomial_basis_array) {
    // Each rational >= 0 requires the Lagrangian multiplier for s-s_lower and
    // s_upper - s.
    const int s_for_plane_size = static_cast<int>(s_for_plane_indices.size());
    const int num_sos = 1 + 2 * s_for_plane_size;
    const int y_size = internal::GetNumYInRational(rational, this->y_slack());
    const int num_gram_vars_per_sos = internal::GetGramVarSize(
        monomial_basis_array, this->with_cross_y(), y_size);
    return num_gram_vars_per_sos * num_sos;
  };

  for (PlaneSide plane_side : {PlaneSide::kPositive, PlaneSide::kNegative}) {
    const SortedPair<multibody::BodyIndex> one_side_body_pair(
        plane.expressed_body, plane.geometry(plane_side)->body_index());
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array =
        this->map_body_to_monomial_basis_array().at(one_side_body_pair);
    for (const auto& rational : plane_geometries.rationals(plane_side)) {
      gram_var_count += count_gram(rational, monomial_basis_array);
    }
  }

  const auto gram_vars =
      ret.prog->NewContinuousVariables(gram_var_count, "Gram");

  gram_var_count = 0;
  auto add_rational_nonnegative =
      [this, &s_for_plane_indices_set, &s_minus_s_lower, &s_upper_minus_s,
       &gram_vars, &gram_var_count](
          solvers::MathematicalProgram* prog,
          const symbolic::RationalFunction& rational,
          const std::array<VectorX<symbolic::Monomial>, 4>&
              monomial_basis_array) -> SeparatingPlaneLagrangians {
    const int y_size = internal::GetNumYInRational(rational, this->y_slack());
    internal::GramAndMonomialBasis gram_and_monomial_basis(
        monomial_basis_array, this->with_cross_y(), y_size);
    const int num_gram_vars_per_sos = gram_and_monomial_basis.gram_var_size;
    const int s_size = this->rational_forward_kin().s().rows();
    SeparatingPlaneLagrangians lagrangians{s_size};

    symbolic::Polynomial poly = rational.numerator();

    for (int i = 0; i < s_size; ++i) {
      if (s_for_plane_indices_set.contains(i)) {
        gram_and_monomial_basis.AddSos(
            prog, gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
            &lagrangians.mutable_s_box_lower()(i));
        gram_var_count += num_gram_vars_per_sos;
        gram_and_monomial_basis.AddSos(
            prog, gram_vars.segment(gram_var_count, num_gram_vars_per_sos),
            &lagrangians.mutable_s_box_upper()(i));
        gram_var_count += num_gram_vars_per_sos;
        poly -= lagrangians.s_box_lower()(i) * s_minus_s_lower(i) +
                lagrangians.s_box_upper()(i) * s_upper_minus_s(i);
      } else {
        lagrangians.mutable_s_box_lower()(i) = symbolic::Polynomial();
        lagrangians.mutable_s_box_upper()(i) = symbolic::Polynomial();
      }
    }
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
    ret.prog->AddIndeterminates(y_slack());
  }

  for (PlaneSide plane_side : {PlaneSide::kPositive, PlaneSide::kNegative}) {
    ret.certificate.mutable_lagrangians(plane_side)
        .reserve(plane_geometries.rationals(plane_side).size());
    const SortedPair<multibody::BodyIndex> one_side_body_pair(
        plane.expressed_body, plane.geometry(plane_side)->body_index());
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array =
        this->map_body_to_monomial_basis_array().at(one_side_body_pair);
    for (const auto& rational : plane_geometries.rationals(plane_side)) {
      ret.certificate.mutable_lagrangians(plane_side)
          .push_back(add_rational_nonnegative(ret.prog.get_mutable(), rational,
                                              monomial_basis_array));
    }
  }

  DRAKE_DEMAND(gram_var_count == gram_vars.rows());
  return ret;
}

std::vector<std::optional<CspaceFreeBox::SeparationCertificateResult>>
CspaceFreeBox::FindSeparationCertificateGivenBox(
    const IgnoredCollisionPairs& ignored_collision_pairs,
    const Eigen::Ref<const Eigen::VectorXd>& q_box_lower,
    const Eigen::Ref<const Eigen::VectorXd>& q_box_upper,
    const FindSeparationCertificateOptions& options) const {
  Eigen::VectorXd s_box_lower;
  Eigen::VectorXd s_box_upper;
  Eigen::VectorXd q_star;
  this->ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                    &q_star);
  PolynomialsToCertify polynomials_to_certify;
  this->GeneratePolynomialsToCertify(s_box_lower, s_box_upper, q_star,
                                     ignored_collision_pairs,
                                     &polynomials_to_certify);
  std::vector<int> active_plane_indices;
  active_plane_indices.reserve(polynomials_to_certify.plane_geometries.size());
  for (const auto& plane_geometries : polynomials_to_certify.plane_geometries) {
    active_plane_indices.push_back(plane_geometries.plane_index);
  }
  std::vector<std::optional<SeparationCertificateResult>> ret(
      active_plane_indices.size(), std::nullopt);

  // This lambda function formulates and solves a small SOS program for each
  // pair of geometries.
  auto solve_small_sos = [this, &polynomials_to_certify, &active_plane_indices,
                          &options,
                          &ret](int plane_count) -> std::pair<bool, int> {
    const int plane_index = active_plane_indices[plane_count];
    auto certificate_program = this->ConstructPlaneSearchProgram(
        polynomials_to_certify.plane_geometries[plane_count],
        polynomials_to_certify.s_minus_s_box_lower,
        polynomials_to_certify.s_box_upper_minus_s);

    solvers::MathematicalProgramResult result;
    solvers::MakeSolver(options.solver_id)
        ->Solve(*certificate_program.prog, std::nullopt, options.solver_options,
                &result);
    if (result.is_success()) {
      ret[plane_count].emplace(certificate_program.certificate.GetSolution(
          plane_index, separating_planes()[plane_index].a,
          separating_planes()[plane_index].b,
          separating_planes()[plane_index].decision_variables, result));
      return std::make_pair(true, plane_count);
    } else {
      ret[plane_count].reset();
      return std::make_pair(false, plane_count);
    }
  };
  this->SolveCertificationForEachPlaneInParallel(
      active_plane_indices, solve_small_sos, options.parallelism,
      options.verbose, options.terminate_at_failure);
  return ret;
}

void CspaceFreeBox::AddCspaceBoxContainment(
    solvers::MathematicalProgram* prog,
    const VectorX<symbolic::Variable>& s_box_lower,
    const VectorX<symbolic::Variable>& s_box_upper,
    const Eigen::MatrixXd& s_inner_pts) const {
  const int s_size = this->rational_forward_kin().s().rows();
  DRAKE_THROW_UNLESS(s_inner_pts.rows() == s_size);
  DRAKE_THROW_UNLESS(s_box_lower.rows() == s_size);
  DRAKE_THROW_UNLESS(s_box_upper.rows() == s_size);
  // We have the constraint s_box_lower <= s <= s_box_upper.
  prog->AddBoundingBoxConstraint(Eigen::VectorXd::Constant(s_size, -kInf),
                                 s_inner_pts.rowwise().minCoeff(), s_box_lower);
  prog->AddBoundingBoxConstraint(s_inner_pts.rowwise().maxCoeff(),
                                 Eigen::VectorXd::Constant(s_size, kInf),
                                 s_box_upper);
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
