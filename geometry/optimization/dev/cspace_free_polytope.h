#pragma once

#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/geometry/optimization/dev/collision_geometry.h"
#include "drake/geometry/optimization/dev/separating_plane.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 Contains the information to enforce a pair of geometries are separated by a
 plane.
 The conditions are that certain rational functions should be always positive,
 and a certain vector has length <= 1.
 */
struct PlaneSeparatesGeometries {
  PlaneSeparatesGeometries(
      std::vector<symbolic::RationalFunction> m_positive_side_rationals,
      std::vector<symbolic::RationalFunction> m_negative_side_rationals,
      std::vector<VectorX<symbolic::Polynomial>> m_unit_length_vectors,
      int m_plane_index, std::optional<symbolic::Variable> m_separating_margin)
      : positive_side_rationals{std::move(m_positive_side_rationals)},
        negative_side_rationals{std::move(m_negative_side_rationals)},
        unit_length_vectors{std::move(m_unit_length_vectors)},
        plane_index{m_plane_index},
        separating_margin{std::move(m_separating_margin)} {}
  const std::vector<symbolic::RationalFunction> positive_side_rationals;
  const std::vector<symbolic::RationalFunction> negative_side_rationals;
  const std::vector<VectorX<symbolic::Polynomial>> unit_length_vectors;
  int plane_index;
  std::optional<symbolic::Variable> separating_margin;
};

/**
 This class tries to find large convex region in the configuration space, such
 that this whole convex set is collision free.
 For more details, refer to the paper
 "Finding and Optimizing Certified, Colision-Free Regions in Configuration Space
 for Robot Manipulators" by Alexandre Amice, Hongkai Dai, Peter Werner, Annan
 Zhang and Russ Tedrake.
 */
class CspaceFreePolytope {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePolytope)

  using FilteredCollsionPairs =
      std::unordered_set<SortedPair<geometry::GeometryId>>;

  ~CspaceFreePolytope() {}

  /**
   @param plant The plant for which we compute the C-space free polytopes. It
   must outlive this CspaceFreePolytope object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreePolytope object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.
   */
  CspaceFreePolytope(const multibody::MultibodyPlant<double>* plant,
                     const geometry::SceneGraph<double>* scene_graph,
                     SeparatingPlaneOrder plane_order);

  [[nodiscard]] const multibody::RationalForwardKinematics&
  rational_forward_kin() const {
    return rational_forward_kin_;
  }

  /**
   separating_planes()[map_geometries_to_separating_planes.at(geometry1_id,
   geometry2_id)] is the separating plane that separates geometry1 and
   geometry 2.
   */
  [[nodiscard]] const std::unordered_map<SortedPair<geometry::GeometryId>, int>&
  map_geometries_to_separating_planes() const {
    return map_geometries_to_separating_planes_;
  }

  [[nodiscard]] const std::vector<SeparatingPlane<symbolic::Variable>>&
  separating_planes() const {
    return separating_planes_;
  }

  /**
   Generate all the conditions (certain rationals being non-negative, and
   certain vectors with length <= 1) such that the robot configuration is
   collision free.
   @param q_star Refer to RationalForwardKinematics for its meaning.
   @param filtered_collision_pairs
   @param separating_margin The distance between the separating plane to each
   geometry is at least this margin. If set to std::nullopt, then we only
   require separation, but don't require a separating margin.
   */
  [[nodiscard]] std::vector<PlaneSeparatesGeometries> GenerateRationals(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const CspaceFreePolytope::FilteredCollsionPairs& filtered_collision_pairs,
      const std::optional<symbolic::Variable>& separating_margin) const;

  /**
   When searching for the separating plane, we want to certify that the
   numerator of a rational is non-negative in the C-space region C*s<=d,
   s_lower<= s <=s_upper. Hence for each of the rational we will introduce
   Lagrangian multipliers for the polytopic constraint d-C*s >= 0, s - s_lower
   >= 0, s_upper - s >= 0.
   */
  struct SeparatingPlaneLagrangians {
    SeparatingPlaneLagrangians(int C_rows, int s_size)
        : polytope(C_rows), s_lower(s_size), s_upper(s_size) {}
    // The Lagrangians for d - C*s >= 0.
    VectorX<symbolic::Polynomial> polytope;
    // The Lagrangians for s - s_lower >= 0.
    VectorX<symbolic::Polynomial> s_lower;
    // The Lagrangians for s_upper - s >= 0.
    VectorX<symbolic::Polynomial> s_upper;

    [[nodiscard]] SeparatingPlaneLagrangians GetSolution(
        const solvers::MathematicalProgramResult& result) const;
  };

  /**
   This struct stores the necessary information to search for the separating
   plane for the polytopic C-space region C*s <= d, s_lower <= s <= s_upper.
   We need to impose that N rationals are non-negative in this C-space polytope.
   The denominator of each rational is always positive hence we need to impose
   the N numerators are non-negative in this C-space region.
   We impose the condition
   numerator_i(s) - λ(s)ᵀ * (d - C*s) - λ_lower(s)ᵀ * (s - s_lower)
         -λ_upper(s)ᵀ * (s_upper - s) is sos
   λ(s) are sos, λ_lower(s) are sos, λ_upper(s) are sos.
   */
  struct SeparationCertificate {
    SeparationCertificate() : prog{new solvers::MathematicalProgram()} {}

    /// The program that stores all the constraints to search for the separating
    /// plane and Lagrangian multipliers as certificate.
    std::unique_ptr<solvers::MathematicalProgram> prog;
    /// positive_side_lagrangians[i] is the Lagrangian multipliers for
    /// PlaneSeparatesGeometries::positive_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> positive_side_lagrangians;
    /// negative_side_lagrangians[i] is the Lagrangian multipliers for
    /// PlaneSeparatesGeometries::negative_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> negative_side_lagrangians;
  };

 private:
  // Forward declaration the tester class. This tester class will expose the
  // private members of CspaceFreePolytope for unit test.
  friend class CspaceFreePolytopeTester;
  // Find the redundant inequalities in C*s <= d, s_lower <= s <= s_upper
  void FindRedundantInequalities(
      const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
      const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
      double tighten, std::unordered_set<int>* C_redundant_indices,
      std::unordered_set<int>* s_lower_redundant_indices,
      std::unordered_set<int>* s_upper_redundant_indices) const;

  // Computes s-s_lower and s_upper - s as polynomials of s.
  void CalcSBoundsPolynomial(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      VectorX<symbolic::Polynomial>* s_minus_s_lower,
      VectorX<symbolic::Polynomial>* s_upper_minus_s) const;

  // Computes d - C*s as a vector of polynomials on indeterminate s.
  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const;

  /**
   Constructs the program which searches for the plane separating a pair of
   geometries, for all configuration in the set {s | C * s <= d, s_lower <= s
   <= s_upper}.
   @param[in] plane_geometries Contain the conditions that need to be
   non-negative on the region C * s <= d and s_lower <= s <= s_upper.
   @param[in] d_minus_Cs d - C*s.
   @param[in] s_minus_s_lower s - s_lower.
   @param[in] s_upper_minus_s s_upper - s.
   @param[in] C_redundant_indices In the polyhedron C*s <= d, s_lower <= s <=
   s_upper, some rows of C*s<=d might be redundant. We store the indices of the
   redundant rows in C_redundant_indices.
   @param[in] s_lower_redundant_indices. Store the indices of the redundant rows
   in s >= s_lower.
   @param[in] s_upper_redundant_indices. Store the indices of the redundant rows
   in s <= s_upper.
   @param[in/out] map_body_to_monomial_basis The sos polynomial requires a
   monomial basis. This data maps the pair of bodies (expressed body,
   collision_body) to the monomial basis. Since this function will be called for
   other pair of geometries which might share the same body pair, to save the
   computation we store the computed monomial basis in this data and reuse it
   later.
   */
  [[nodiscard]] SeparationCertificate ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const VectorX<symbolic::Polynomial>& s_minus_s_lower,
      const VectorX<symbolic::Polynomial>& s_upper_minus_s,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices,
      std::unordered_map<SortedPair<multibody::BodyIndex>,
                         VectorX<symbolic::Monomial>>*
          map_body_to_monomial_basis) const;

  multibody::RationalForwardKinematics rational_forward_kin_;
  const geometry::SceneGraph<double>& scene_graph_;
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CollisionGeometry>>>
      link_geometries_;

  SeparatingPlaneOrder plane_order_;
  std::vector<SeparatingPlane<symbolic::Variable>> separating_planes_;
  std::unordered_map<SortedPair<geometry::GeometryId>, int>
      map_geometries_to_separating_planes_;
};

/**
 * Given a diagram (which contains the plant and the scene_graph), returns all
 * the collision geometries.
 */
[[nodiscard]] std::map<multibody::BodyIndex,
                       std::vector<std::unique_ptr<CollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
