#include "drake/geometry/optimization/dev/test/c_iris_test_utilities.h"

namespace drake {
namespace geometry {
namespace optimization {
void CspaceFreePolytopeTester::FindRedundantInequalities(
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
    const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
    double tighten, std::unordered_set<int>* C_redundant_indices,
    std::unordered_set<int>* s_lower_redundant_indices,
    std::unordered_set<int>* s_upper_redundant_indices) const {
  cspace_free_polytope_->FindRedundantInequalities(
      C, d, s_lower, s_upper, tighten, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

CspaceFreePolytope::SeparationCertificateProgram
CspaceFreePolytopeTester::ConstructPlaneSearchProgram(
    const PlaneSeparatesGeometries& plane_geometries,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::unordered_set<int>& C_redundant_indices,
    const std::unordered_set<int>& s_lower_redundant_indices,
    const std::unordered_set<int>& s_upper_redundant_indices) const {
  return cspace_free_polytope_->ConstructPlaneSearchProgram(
      plane_geometries, d_minus_Cs, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

std::vector<std::optional<CspaceFreePolytope::SeparationCertificateResult>>
CspaceFreePolytopeTester::FindSeparationCertificateGivenPolytope(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions&
        options) const {
  return cspace_free_polytope_->FindSeparationCertificateGivenPolytope(
      ignored_collision_pairs, C, d, options);
}

int CspaceFreePolytopeTester::GetGramVarSizeForPolytopeSearchProgram(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    bool search_s_bounds_lagrangians) const {
  return cspace_free_polytope_->GetGramVarSizeForPolytopeSearchProgram(
      ignored_collision_pairs, search_s_bounds_lagrangians);
}

std::unique_ptr<solvers::MathematicalProgram>
CspaceFreePolytopeTester::InitializePolytopeSearchProgram(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    const MatrixX<symbolic::Variable>& C, const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Polynomial> d_minus_Cs,
    const std::vector<
        std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
        certificates_vec,
    bool search_s_bounds_lagrangians, int gram_total_size,
    std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>*
        new_certificates) const {
  return cspace_free_polytope_->InitializePolytopeSearchProgram(
      ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec,
      search_s_bounds_lagrangians, gram_total_size, new_certificates);
}

void CspaceFreePolytopeTester::AddEllipsoidContainmentConstraint(
    solvers::MathematicalProgram* prog, const Eigen::MatrixXd& Q,
    const Eigen::VectorXd& s0, const MatrixX<symbolic::Variable>& C,
    const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Variable>& ellipsoid_margins) const {
  cspace_free_polytope_->AddEllipsoidContainmentConstraint(prog, Q, s0, C, d,
                                                           ellipsoid_margins);
}

void CspaceFreePolytopeTester::AddCspacePolytopeContainment(
    solvers::MathematicalProgram* prog, const MatrixX<symbolic::Variable>& C,
    const VectorX<symbolic::Variable>& d,
    const Eigen::MatrixXd& s_inner_pts) const {
  cspace_free_polytope_->AddCspacePolytopeContainment(prog, C, d, s_inner_pts);
}

std::optional<CspaceFreePolytope::FindPolytopeGivenLagrangianResult>
CspaceFreePolytopeTester::FindPolytopeGivenLagrangian(
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
    const MatrixX<symbolic::Variable>& C, const VectorX<symbolic::Variable>& d,
    const VectorX<symbolic::Polynomial>& d_minus_Cs,
    const std::vector<
        std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
        certificates_vec,
    const Eigen::MatrixXd& Q, const Eigen::VectorXd& s0,
    const VectorX<symbolic::Variable>& ellipsoid_margins, int gram_total_size,
    const CspaceFreePolytope::FindPolytopeGivenLagrangianOptions& options,
    std::unordered_map<int, CspaceFreePolytope::SeparationCertificateResult>*
        certificates_result) const {
  return cspace_free_polytope_->FindPolytopeGivenLagrangian(
      ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec, Q, s0,
      ellipsoid_margins, gram_total_size, options, certificates_result);
}

HPolyhedron CspaceFreePolytopeTester::GetPolyhedronWithJointLimits(
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d) const {
  return cspace_free_polytope_->GetPolyhedronWithJointLimits(C, d);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
