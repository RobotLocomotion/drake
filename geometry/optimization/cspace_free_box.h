#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "drake/geometry/optimization/cspace_free_polytope_base.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 This class tries to find large axis-aligned bounding boxes in the configuration
 space, such that all configurations in the boxes are collision free.
 Note that we don't guarantee to find the largest box.

 @ingroup planning_iris
 */
// CspaceFreeBox "is a" CspaceFreePolytopeBase because it can do anything inside
// CspaceFreePolytopeBase. We factor out the common code in CspaceFreeBox and
// CspaceFreePolytope to CspaceFreePolytopeBase, and also rely on the access
// control (public/protected/private) in CspaceFreePolytopeBase.
class CspaceFreeBox : public CspaceFreePolytopeBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreeBox);

  using CspaceFreePolytopeBase::IgnoredCollisionPairs;

  ~CspaceFreeBox() override;

  using CspaceFreePolytopeBase::Options;

  /**
   When searching for the separating plane, we want to certify that the
   numerator of a rational is non-negative in the C-space box q_box_lower <= q
   <= q_box_upper (or equivalently s_box_lower <= s <= s_box_upper). Hence for
   each of the rational we will introduce Lagrangian multipliers for the
   polytopic constraint s - s_box_lower >= 0, s_box_upper - s >= 0.
   */
  class SeparatingPlaneLagrangians {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparatingPlaneLagrangians);

    explicit SeparatingPlaneLagrangians(int s_size)
        : s_box_lower_(s_size), s_box_upper_(s_size) {}

    ~SeparatingPlaneLagrangians();

    /** Substitutes the decision variables in each Lagrangians with its value in
     * result, returns the substitution result.
     */
    [[nodiscard]] SeparatingPlaneLagrangians GetSolution(
        const solvers::MathematicalProgramResult& result) const;

    /// The Lagrangians for s - s_box_lower >= 0.
    const VectorX<symbolic::Polynomial>& s_box_lower() const {
      return s_box_lower_;
    }

    /// The Lagrangians for s - s_box_lower >= 0.
    VectorX<symbolic::Polynomial>& mutable_s_box_lower() {
      return s_box_lower_;
    }

    /// The Lagrangians for s_box_upper - s >= 0.
    const VectorX<symbolic::Polynomial>& s_box_upper() const {
      return s_box_upper_;
    }

    /// The Lagrangians for s_box_upper - s >= 0.
    VectorX<symbolic::Polynomial>& mutable_s_box_upper() {
      return s_box_upper_;
    }

   private:
    // The Lagrangians for s - s_box_lower >= 0.
    VectorX<symbolic::Polynomial> s_box_lower_;
    // The Lagrangians for s_box_upper - s >= 0.
    VectorX<symbolic::Polynomial> s_box_upper_;
  };

  /**
   We certify that a pair of geometries is collision free in the C-space box
   {q | q_box_lower<=q<=q_box_upper} by finding the separating plane and the
   Lagrangian multipliers. This struct contains the certificate, that the
   separating plane {x | aᵀx+b=0 } separates the two geometries in
   separating_planes()[plane_index] in the C-space box.
   */
  struct SeparationCertificateResult final : SeparationCertificateResultBase {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateResult);
    SeparationCertificateResult() = default;
    ~SeparationCertificateResult() final;

    const std::vector<SeparatingPlaneLagrangians>& lagrangians(
        PlaneSide plane_side) const {
      return plane_side == PlaneSide::kPositive
                 ? positive_side_rational_lagrangians
                 : negative_side_rational_lagrangians;
    }

    std::vector<SeparatingPlaneLagrangians> positive_side_rational_lagrangians;
    std::vector<SeparatingPlaneLagrangians> negative_side_rational_lagrangians;
  };

  /**
   This struct stores the necessary information to search for the separating
   plane for the polytopic C-space box q_box_lower <= q <= q_box_upper.
   We need to impose that N rationals are non-negative in this C-space box.
   The denominator of each rational is always positive hence we need to impose
   the N numerators are non-negative in this C-space box.
   We impose the condition
   numerator_i(s) - λ_lower(s)ᵀ * (s - s_lower)
         -λ_upper(s)ᵀ * (s_upper - s) is sos
   λ_lower(s) are sos, λ_upper(s) are sos.
   */
  struct SeparationCertificate {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificate);
    SeparationCertificate() = default;
    ~SeparationCertificate();

    [[nodiscard]] SeparationCertificateResult GetSolution(
        int plane_index, const Vector3<symbolic::Polynomial>& a,
        const symbolic::Polynomial& b,
        const VectorX<symbolic::Variable>& plane_decision_vars,
        const solvers::MathematicalProgramResult& result) const;

    std::vector<SeparatingPlaneLagrangians>& mutable_lagrangians(
        PlaneSide plane_side) {
      return plane_side == PlaneSide::kPositive
                 ? positive_side_rational_lagrangians
                 : negative_side_rational_lagrangians;
    }
    // positive_side_rational_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::positive_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> positive_side_rational_lagrangians;
    // negative_side_rational_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::negative_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> negative_side_rational_lagrangians;
  };

  struct SeparationCertificateProgram final : SeparationCertificateProgramBase {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateProgram);
    SeparationCertificateProgram() = default;
    ~SeparationCertificateProgram() final;

    SeparationCertificate certificate;
  };

  /**
   @param plant The plant for which we compute the C-space free boxes. It
   must outlive this CspaceFreeBox object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreeBox object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.

   @note CspaceFreeBox knows nothing about contexts. The plant and
   scene_graph must be fully configured before instantiating this class.
   */
  CspaceFreeBox(const multibody::MultibodyPlant<double>* plant,
                const geometry::SceneGraph<double>* scene_graph,
                SeparatingPlaneOrder plane_order,
                const Options& options = Options{});

  /** Finds the certificates that the C-space box {q | q_box_lower <= q <=
   * q_box_upper} is collision free.
   *
   * @param q_box_lower The lower bound of the C-space box.
   * @param q_box_upper The upper bound of the C-space box.
   * @param ignored_collision_pairs We ignore the pair of geometries in
   * `ignored_collision_pairs`.
   * @param[out] certificates Contains the certificate we successfully found for
   * each pair of geometries. Notice that depending on `options`, the program
   * could search for the certificate for each geometry pair in parallel, and
   * will terminate the search once it fails to find the certificate for any
   * pair. At termination, the pair of geometries whose optimization hasn't been
   * finished will not show up in @p certificates.
   * @retval success If true, then we have certified that the C-space box
   * {q | q_box_lower<=q<=q_box_upper} is collision free. Otherwise
   * success=false.
   */
  bool FindSeparationCertificateGivenBox(
      const Eigen::Ref<const Eigen::VectorXd>& q_box_lower,
      const Eigen::Ref<const Eigen::VectorXd>& q_box_upper,
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const FindSeparationCertificateOptions& options,
      std::unordered_map<SortedPair<geometry::GeometryId>,
                         SeparationCertificateResult>* certificates) const;

 private:
  // Forward declare the tester class that will test the private members.
  friend class CspaceFreeBoxTester;

  /*
   Computes the range of s from the box q_box_lower <= q <= q_box_upper. We also
   set q_star = 0.5(q_box_lower + q_box_upper).
   If q_box_lower is smaller than the robot position lower limit (or q_box_upper
   is larger than the robot position upper limit), then we clamp q_box_lower (or
   q_box_upper) within the robot position limits.
   @throws error if any q_box_lower is larger than q_box_upper or the robot
   position upper limit; similarly throws an error if any q_box_upper is smaller
   than the robot position lower limit.
   */
  void ComputeSBox(const Eigen::Ref<const Eigen::VectorXd>& q_box_lower,
                   const Eigen::Ref<const Eigen::VectorXd>& q_box_upper,
                   Eigen::VectorXd* s_box_lower, Eigen::VectorXd* s_box_upper,
                   Eigen::VectorXd* q_star) const;

  /*
   This class contains the polynomials that we wish to certify are non-negative.
   */
  struct PolynomialsToCertify {
    // We have the invariant plane_geometries_[i].plane_index == i.
    std::vector<PlaneSeparatesGeometries> plane_geometries;
    VectorX<symbolic::Polynomial> s_minus_s_box_lower;
    VectorX<symbolic::Polynomial> s_box_upper_minus_s;
  };

  // Generates the polynomials used for certifying the box s_box_lower <= s <=
  // s_box_upper is collision free.
  // @note The box [s_box_lower, s_box_upper] is already inside the
  // tangent-configuration space box computed from the robot position
  // lower/upper limits.
  // TODO(hongkai.dai): after we finish implementing this class, consider to
  // change the input argument to q_box_lower and q_box_upper, if ComputeSBox is
  // only used with this function.
  void GeneratePolynomialsToCertify(
      const Eigen::Ref<const Eigen::VectorXd>& s_box_lower,
      const Eigen::Ref<const Eigen::VectorXd>& s_box_upper,
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const IgnoredCollisionPairs& ignored_collision_pairs,
      PolynomialsToCertify* certify_polynomials) const;

  /*
   Constructs the program which searches for the plane separating a pair of
   geometries, for all configuration in the box {q | q_box_lower <= q <=
   q_box_upper}.
   @param[in] plane_geometries Contain the conditions that need to be
   non-negative in the box q_box_lower <= q <= q_box_upper.
   @param[in] s_minus_s_lower s - s_lower.
   @param[in] s_upper_minus_s s_upper - s.
   */
  [[nodiscard]] SeparationCertificateProgram ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& s_minus_s_lower,
      const VectorX<symbolic::Polynomial>& s_upper_minus_s) const;

  /*
   Finds the certificates that the C-space box {q | q_box_lower <= q <=
   q_box_upper} is collision free.
   @retval certificates certificates[i] is the separation certificate for a pair
   of geometries. If we cannot certify or haven't certified the separation for
   this pair, then certificates[i] contains std::nullopt. Note that when we run
   this function in parallel and options.terminate_at_failure=true, we will
   terminate all the remaining certification programs that have been launched,
   so certificates[i] = std::nullopt could be either because that we have
   attempted to find the certificate for this pair of geometry but failed, or it
   could be that we fail to find the certificate for another pair and haven't
   attempted to find the certificate for this pair.
   The geometry pair which certificates[i] certifies is given by
   separating_planes()[certificates[i].plane_index].geometry_pair().
   */
  [[nodiscard]] std::vector<std::optional<SeparationCertificateResult>>
  FindSeparationCertificateGivenBox(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const Eigen::Ref<const Eigen::VectorXd>& q_box_lower,
      const Eigen::Ref<const Eigen::VectorXd>& q_box_upper,
      const FindSeparationCertificateOptions& options) const;

  /*
   Adds the constraint that each column of s_inner_pts is in the box s_box_lower
   <= s <= s_box_upper.
   */
  void AddCspaceBoxContainment(solvers::MathematicalProgram* prog,
                               const VectorX<symbolic::Variable>& s_box_lower,
                               const VectorX<symbolic::Variable>& s_box_upper,
                               const Eigen::MatrixXd& s_inner_pts) const;
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
