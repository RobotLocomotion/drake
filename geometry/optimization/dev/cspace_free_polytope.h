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
#include "drake/solvers/mosek_solver.h"

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
   @param q_star Refer to RationalForwardKinematics for its meaning.
   */
  CspaceFreePolytope(const multibody::MultibodyPlant<double>* plant,
                     const geometry::SceneGraph<double>* scene_graph,
                     SeparatingPlaneOrder plane_order,
                     const Eigen::Ref<const Eigen::VectorXd>& q_star);

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

  [[nodiscard]] const VectorX<symbolic::Variable>& y_slack() const {
    return y_slack_;
  }

  [[nodiscard]] const std::vector<PlaneSeparatesGeometries>&
  rationals_with_margin() const {
    return rationals_with_margin_;
  }

  [[nodiscard]] const std::vector<PlaneSeparatesGeometries>&
  rationals_without_margin() const {
    return rationals_with_margin_;
  }

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
   When searching for the separating plane, sometimes we want to certify that a
   vector of polynomials has length <= 1 in the C-space region {s | C*s<=d,
   s_lower<=s<=s_upper}. To this end, we will impose a matrix-sos constraint,
   that certain polynomial p(y, s) >=0 in the polytope, namely
   p(y, s) - λ₁(y, s)ᵀ(d−Cs) − λ₂(y, s)ᵀ(s−s_lower) − λ₃(y, s)ᵀ(s_upper−s) -
   ν₄(y, s)(1−yᵀy) is sos.
   λ₁(y, s), λ₂(y, s), λ₃(y, s) are all sos.
   ν₄(y, s) is a free polynomial.
   Notice that we add the extra restriction yᵀy=1 through the term λ₄(y,
   s)(1−yᵀy). This restriction makes the semialgebraic set of y being compact,
   enabling us using stronger Positivstellensatz theorem. For more explanation
   on this term, refer to the documentation in the private function
   AddUnitLengthConstraint.

   This struct records the Lagrangian multipliers λ₁(y,
   s), λ₂(y, s), λ₃(y, s) and ν₄(y, s).
   */
  struct UnitLengthLagrangians {
    UnitLengthLagrangians(int C_rows, int s_size)
        : polytope(C_rows), s_lower(s_size), s_upper(s_size) {}
    // λ₁(y, s) in the documentation.
    VectorX<symbolic::Polynomial> polytope;
    // λ₂(y, s) in the documentation.
    VectorX<symbolic::Polynomial> s_lower;
    // λ₃(y, s) in the documentation.
    VectorX<symbolic::Polynomial> s_upper;
    // ν₄(y, s) in the documentation.
    symbolic::Polynomial y_square;

    [[nodiscard]] UnitLengthLagrangians GetSolution(
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

    // unit_length_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::unit_length_vectors[i].
    std::vector<UnitLengthLagrangians> unit_length_lagrangians;
  };

  /**
   We certify that a pair of geometries is collision free in the C-space region
   {s | Cs<=d, s_lower<=s<=s_upper}, by finding the separating plane and the
   Lagrangian multipliers. This struct contains the certificate, that the
   separating plane {x | aᵀx+b=0 } separates the two geometries in
   separating_planes()[plane_index] in the C-space polytope.
   */
  struct SeparationCertificateResult {
    int plane_index;
    Eigen::MatrixXd C;
    Eigen::VectorXd d;
    std::vector<SeparatingPlaneLagrangians> positive_side_lagrangians;
    std::vector<SeparatingPlaneLagrangians> negative_side_lagrangians;
    std::vector<UnitLengthLagrangians> unit_length_lagrangians;
    std::optional<double> separating_margin;
    // The separating plane is { x | aᵀx+b=0 }
    Vector3<symbolic::Polynomial> a;
    symbolic::Polynomial b;
  };

  struct FindSeparationCertificateGivenPolytopeOptions {
    // We can find the certificate for each pair of geometries in parallel.
    // num_threads specifies how many threads we run in parallel. If num_threads
    // <=0, then we use all available threads on the computer.
    int num_threads{1};

    // If verbose set to true, then we will print some information to the
    // terminal.
    bool verbose{false};

    // The solver invoked for the sos program.
    solvers::SolverId solver_id{solvers::MosekSolver::id()};

    // If the SOS in one thread fails, then don't launch any more threads.
    bool terminate_at_failure{true};
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
  void CalcSBoundsPolynomial();

  // Computes d - C*s as a vector of polynomials on indeterminate s.
  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const;

  /**
   Generate all the conditions (certain rationals being non-negative, and
   certain vectors with length <= 1) such that the robot configuration is
   collision free.
   @param filtered_collision_pairs
   @param search_separating_margin If set to true, then when we search for the
   separating planes, we will attempt to maximize the margin of the separating
   planes.
   */
  [[nodiscard]] std::vector<PlaneSeparatesGeometries> GenerateRationals(
      bool search_separating_margin) const;

  /**
   Computes the monomial basis for each pair of bodies.

   There can be multiple collision geometries on the same body, and their SOS
   problem will all share the same monomial basis. Hence we can first compute
   the monomial basis for each body, and reuse the result for all the collision
   geometries on the same body pair.
   */
  [[nodiscard]] std::unordered_map<SortedPair<multibody::BodyIndex>,
                                   VectorX<symbolic::Monomial>>
  CalcMonomialBasis(
      const std::vector<PlaneSeparatesGeometries>& plane_geometries) const;

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
   @param[in] map_body_to_monomial_basis The sos polynomial requires a
   monomial basis. This data maps the pair of bodies (expressed body,
   collision_body) to the monomial basis.
   */
  [[nodiscard]] SeparationCertificate ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices,
      const std::unordered_map<SortedPair<multibody::BodyIndex>,
                               VectorX<symbolic::Monomial>>&
          map_body_to_monomial_basis) const;

  // Impose the condition that |a(s)| <= 1 in the polytope
  // {s | C*s <= d, s_lower <= s <= s_upper}. where a(s) is a vector of
  // polynomials. This can be imposed using matrix-sos condition
  // ⌈ 1   a(s)ᵀ⌉ is psd in the polytope.
  // ⌊a(s)    I ⌋
  // Namely the polynomial of (y, s)
  // yᵀ * ⌈ 1   a(s)ᵀ⌉ * y >= 0 in the polytope.
  //      ⌊a(s)    I ⌋
  // If we denote p(y, s) = yᵀ * ⌈ 1   a(s)ᵀ⌉ * y
  //                             ⌊a(s)    I ⌋
  // We want p(y, s) >= 0 on the polytope.
  //
  // Note that this polytope is NOT a compact set on y (it is a compact set on
  // s). We wish to make this set compact so that we can use a stronger
  // Positivstellensatz. To do so, we notice that p(y, s) is homogeneous in y,
  // hence p(y, s) >= 0 on the polytope if and only if we further restrain y to
  // be on the unit circle. Namely we consider the condition p(y, s) >= 0 if (y,
  // s) ∈ {s | C*s<=d, s_lower<=s<=s_upper} ∩ {y | yᵀy=1}
  //
  // Then we impose the condition
  // p(y, s) - λ₁(y, s)ᵀ(d−Cs) − λ₂(y, s)ᵀ(s−s_lower) − λ₃(y, s)ᵀ(s_upper−s) -
  // ν₄(y, s)(1−yᵀy) is sos.
  // λ₁(y, s), λ₂(y, s), λ₃(y, s) are all sos.
  // ν₄(y, s) is a free polynomial.
  [[nodiscard]] UnitLengthLagrangians AddUnitLengthConstraint(
      solvers::MathematicalProgram* prog,
      const VectorX<symbolic::Polynomial>& unit_length_vec,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices) const;

  /**
   For each pair of geometries, find the certificate that the pair is collision
   free in the C-space region {s | C*s<=d, s_lower<=s<=s_upper}.

   @retval certificates certificates[i] is the separation certificate for the
   i'th pair in plane_geometries[i]. If we cannot certify or haven't certified
   the separation for this pair, then certificates[i] contains std::nullopt.
   Note that when we run this function in parallel and
   options.terminate_at_failure=true, we will terminate all the remaining
   certification programs that have been launched, so certificates[i] =
   std::nullopt could be either because that we have attempted to find the
   certificate for this pair of geometry but failed, or it could be that we fail
   to find the certificate for another pair and haven't attempted to find the
   certificate for this pair.
   */
  [[nodiscard]] std::vector<std::optional<SeparationCertificateResult>>
  FindSeparationCertificateGivenPolytope(
      const std::vector<PlaneSeparatesGeometries>& plane_geometries,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d,
      const std::unordered_map<SortedPair<multibody::BodyIndex>,
                               VectorX<symbolic::Monomial>>&
          map_body_to_monomial_basis,
      const FindSeparationCertificateGivenPolytopeOptions& options) const;

  multibody::RationalForwardKinematics rational_forward_kin_;
  const geometry::SceneGraph<double>& scene_graph_;
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CollisionGeometry>>>
      link_geometries_;

  SeparatingPlaneOrder plane_order_;
  std::vector<SeparatingPlane<symbolic::Variable>> separating_planes_;
  std::unordered_map<SortedPair<geometry::GeometryId>, int>
      map_geometries_to_separating_planes_;

  // Sometimes we need to impose that a certain matrix of polynomials are always
  // psd (for example with sphere or capsule collision geometries). We will use
  // this slack variable to help us impose the matrix-sos constraint.
  VectorX<symbolic::Variable> y_slack_;

  symbolic::Variables s_set_;

  Eigen::VectorXd q_star_;
  Eigen::VectorXd s_lower_;
  Eigen::VectorXd s_upper_;
  VectorX<symbolic::Polynomial> s_minus_s_lower_;
  VectorX<symbolic::Polynomial> s_upper_minus_s_;
  std::vector<PlaneSeparatesGeometries> rationals_with_margin_;
  std::vector<PlaneSeparatesGeometries> rationals_without_margin_;
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
