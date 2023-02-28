#pragma once

#include <array>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/optimization/cspace_free_polytope.h"
#include "drake/geometry/optimization/dev/cspace_free_path.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace geometry {
namespace optimization {

// Create a toy robot with different collision geometries.
// world - weld - body0 - revolute - body1 - prismatic - body2
//                 |
//              revolute - body3
// We will use this model for C-IRIS on kinematics planning, so the dynamics
// properties (inertia, com, etc) don't matter.
// We want to make sure that the robot model includes every pair of geometry
// types for collision avoidance.
class CIrisToyRobotTest : public ::testing::Test {
 public:
  CIrisToyRobotTest();

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::vector<multibody::BodyIndex> body_indices_;
  geometry::GeometryId world_box_;
  geometry::GeometryId world_cylinder_;
  geometry::GeometryId body0_box_;
  geometry::GeometryId body0_sphere_;
  geometry::GeometryId body1_convex_;
  geometry::GeometryId body1_capsule_;
  geometry::GeometryId body2_capsule_;
  geometry::GeometryId body2_sphere_;
  geometry::GeometryId body3_box_;
  geometry::GeometryId body3_cylinder_;
};

// Create a robot with only polytopic collision geometry.
// world - revolute - body0 - revolute - body1 - revolute - body2 - revolute -
// body3
class CIrisRobotPolytopicGeometryTest : public ::testing::Test {
 public:
  CIrisRobotPolytopicGeometryTest();

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::vector<multibody::BodyIndex> body_indices_;
  std::vector<geometry::GeometryId> world_boxes_;
  geometry::GeometryId world_convex_;
  std::vector<geometry::GeometryId> body_boxes_;
};

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

  std::vector<PlaneSeparatesGeometries>& plane_geometries() const {
    return cspace_free_polytope_->get_mutable_plane_geometries();
  }

  const symbolic::Variables s_set() const {
    return cspace_free_polytope_->get_s_set();
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

// This is a friend class of CspaceFreePath, we use it to expose the private
// functions in CspaceFreePath for unit testing.
class CspaceFreePathTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePathTester)

  CspaceFreePathTester(const multibody::MultibodyPlant<double>* plant,
                       const geometry::SceneGraph<double>* scene_graph,
                       SeparatingPlaneOrder plane_order,
                       const Eigen::Ref<const Eigen::VectorXd>& q_star,
                       unsigned int maximum_path_degree,
                       const CspaceFreePolytope::Options& options =
                           CspaceFreePolytope::Options())
      : cspace_free_path_{new CspaceFreePath(plant, scene_graph, plane_order,
                                             q_star, maximum_path_degree,
                                             options)} {}

  const CspaceFreePath& cspace_free_path() const { return *cspace_free_path_; }

  const symbolic::Variable& get_mu() const { return cspace_free_path_->mu_; }

  const std::unordered_map<symbolic::Variable, symbolic::Polynomial>& get_path()
      const {
    return cspace_free_path_->path_;
  }

  const symbolic::Variables s_set() const {
    return cspace_free_path_->get_s_set();
  }

  std::vector<PlaneSeparatesGeometries>& plane_geometries() const {
    return cspace_free_path_->get_mutable_plane_geometries();
  }

 private:
  std::unique_ptr<CspaceFreePath> cspace_free_path_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
