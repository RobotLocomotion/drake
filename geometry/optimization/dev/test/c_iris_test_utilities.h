#pragma once

#include <array>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "drake/geometry/optimization/dev/cspace_free_polytope.h"

namespace drake {
namespace geometry {
namespace optimization {
// This is a friend class of CspaceFreePolytope, we use it to expose the private
// functions in CspaceFreePolytope for unit testing.
class CspaceFreePolytopeTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePolytopeTester)

  CspaceFreePolytopeTester(const multibody::MultibodyPlant<double>* plant,
                           const geometry::SceneGraph<double>* scene_graph,
                           SeparatingPlaneOrder plane_order,
                           const Eigen::Ref<const Eigen::VectorXd>& q_star,
                           const CspaceFreePolytope::Options& options =
                               CspaceFreePolytope::Options())
      : cspace_free_polytope_{new CspaceFreePolytope(
            plant, scene_graph, plane_order, q_star, options)} {}

  const CspaceFreePolytope& cspace_free_polytope() const {
    return *cspace_free_polytope_;
  }

  void FindRedundantInequalities(
      const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
      const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
      double tighten, std::unordered_set<int>* C_redundant_indices,
      std::unordered_set<int>* s_lower_redundant_indices,
      std::unordered_set<int>* s_upper_redundant_indices) const;

  const Eigen::VectorXd& s_lower() const {
    return cspace_free_polytope_->s_lower_;
  }

  const Eigen::VectorXd& s_upper() const {
    return cspace_free_polytope_->s_upper_;
  }

  const VectorX<symbolic::Polynomial>& s_minus_s_lower() const {
    return cspace_free_polytope_->s_minus_s_lower_;
  }

  const VectorX<symbolic::Polynomial>& s_upper_minus_s() const {
    return cspace_free_polytope_->s_upper_minus_s_;
  }

  const std::vector<PlaneSeparatesGeometries>& plane_geometries() const {
    return cspace_free_polytope_->plane_geometries_;
  }

  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const {
    return cspace_free_polytope_->CalcDminusCs<T>(C, d);
  }

  [[nodiscard]] const std::unordered_map<
      SortedPair<multibody::BodyIndex>,
      std::array<VectorX<symbolic::Monomial>, 4>>&
  map_body_to_monomial_basis_array() const {
    return cspace_free_polytope_->map_body_to_monomial_basis_array_;
  }

  [[nodiscard]] CspaceFreePolytope::SeparationCertificateProgram
  ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices) const;

  [[nodiscard]] std::vector<
      std::optional<CspaceFreePolytope::SeparationCertificateResult>>
  FindSeparationCertificateGivenPolytope(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d,
      const CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions&
          options) const;

  [[nodiscard]] int GetGramVarSizeForPolytopeSearchProgram(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      bool search_s_bounds_lagrangians) const;

  [[nodiscard]] std::unique_ptr<solvers::MathematicalProgram>
  InitializePolytopeSearchProgram(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Polynomial> d_minus_Cs,
      const std::vector<
          std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
          certificates_vec,
      bool search_s_bounds_lagrangians, int gram_total_size,
      std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>*
          new_certificates) const;

  void AddEllipsoidContainmentConstraint(
      solvers::MathematicalProgram* prog, const Eigen::MatrixXd& Q,
      const Eigen::VectorXd& s0, const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Variable>& ellipsoid_margins) const;

  void AddCspacePolytopeContainment(solvers::MathematicalProgram* prog,
                                    const MatrixX<symbolic::Variable>& C,
                                    const VectorX<symbolic::Variable>& d,
                                    const Eigen::MatrixXd& s_inner_pts) const;

  [[nodiscard]] std::optional<
      CspaceFreePolytope::FindPolytopeGivenLagrangianResult>
  FindPolytopeGivenLagrangian(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::vector<
          std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
          certificates_vec,
      const Eigen::MatrixXd& Q, const Eigen::VectorXd& s0,
      const VectorX<symbolic::Variable>& ellipsoid_margins, int gram_total_size,
      const CspaceFreePolytope::FindPolytopeGivenLagrangianOptions& options,
      std::unordered_map<int, CspaceFreePolytope::SeparationCertificateResult>*
          certificates_result) const;

  HPolyhedron GetPolyhedronWithJointLimits(const Eigen::MatrixXd& C,
                                           const Eigen::VectorXd& d) const;

 private:
  std::unique_ptr<CspaceFreePolytope> cspace_free_polytope_;
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
