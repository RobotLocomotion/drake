#pragma once

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
 */
// CspaceFreeBox "is a" CspaceFreePolytopeBase because it can do anything inside
// CspaceFreePolytopeBase. We factor out the common code in CspaceFreeBox and
// CspaceFreePolytope to CspaceFreePolytopeBase, and also rely on the access
// control (public/protected/private) in CspaceFreePolytopeBase.
class CspaceFreeBox : public CspaceFreePolytopeBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreeBox)

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
    explicit SeparatingPlaneLagrangians(int s_size)
        : s_box_lower_(s_size), s_box_upper_(s_size) {}

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
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
