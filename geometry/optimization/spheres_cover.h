#pragma once
#include <optional>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 * Given many polytopes Pᵢ, i=1, ..., N. We want to find spheres such that
 * each polytope Pᵢ is covered by at least one of the spheres, and we want the
 * union of the spheres to be a tight approximation of the union of Pᵢ. We
 * measure the tightness by putting many sampled points outside of the
 * polytopes Pᵢ, and our goal is to minimize the number of sample points being
 * covered by the union of spheres. If possible, also minimize the size of the
 * spheres.
 *
 * @note We assume that the center of each sphere is in the convex hull of the
 * polytopes.
 *
 * Mathematically we solve this problem as
 *
 * Step 1:
 * min ∑ⱼ1(oⱼ ∈ (S₁∪ S₂ ∪...∪ Sₘ))
 * s.t Pᵢ ⊂ Sₖ for at least one k
 *
 * where oⱼ is a sampled point outside of the union of geometries Pᵢ. 1(oⱼ ∈
 * (S₁∪ S₂ ∪...∪ Sₘ)) is the indicator variable that qⱼ is inside any
 * spheres. Sₖ is the k'th sphere. This optimization will be a mixed-integer
 * program.
 *
 * Step 1 finds the spheres that cover all polytopes, and minimize the outliers
 * being covered by any sphere. Note that it is possible to use less number of
 * spheres, while still cover the polytopes and obtain the same optimal cost as
 * in step 1. Ideally we would prefer using less number of spheres. Hence we
 * solve another optimization problem
 *
 * Step 2:
 * min m
 * s.t Pᵢ ⊂ Sₖ for at least one k
 *     ∑ⱼ1(oⱼ ∈ (S₁∪ S₂ ∪...∪ Sₘ)) = optimal_cost_in_step_1.
 *
 * where m is the number of active spheres. It equals to the sum of some
 * binary variables.
 *
 * Note that we can still scale the size of the
 * spheres such that they obtain the same optimal cost (one example is that
 * given a box and many outliers far away from the box, we can cover this box
 * while avoiding the outliers using one sphere of different radius). When this
 * happens, ideally we want to minimize the size of the spheres. We do this in
 * step 3.
 *
 * Step 3:
 * min radius²(S₁) + radius²(S₂) + ... + radius²(Sₘ)
 * s.t Pᵢ ⊂ Sₖ for at least one k
 *     ∑ⱼ1(oⱼ ∈ (S₁∪ S₂ ∪...∪ Sₘ)) = optimal_cost_in_step_2.
 *
 */
class SpheresOuterApproximator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpheresOuterApproximator)

  /**
   * We parameterize a sphere as
   * xᵀx + aᵀx + b ≤ 0
   * where a and b are decision variables.
   */
  struct Sphere {
    solvers::VectorXDecisionVariable a;
    symbolic::Variable b;
  };

  /** Constructor
   * @param num_spheres. Number of spheres we use to cover the geometries.
   * @param polytopes. The polytopes we want to cover with spheres. polytopes[i]
   * contains the vertices of the i'th polytope, each column of polytopes[i] is
   * one vertex.
   * @param outliers The points outside of each polytopes that we want to cover
   * as few as possible. The i'th column is the location of the i'th outlier
   * point.
   * @param max_sphere_radius The maximal value of the sphere radius. If this
   * value is set, then we add the second-order cone constraint aᵢᵀaᵢ  ≤ 4(rₘₐₓ²
   * + bᵢ) which constrains the sphere radius to be less than rₘₐₓ
   */
  SpheresOuterApproximator(
      int num_spheres, std::vector<Eigen::MatrixXd> polytopes,
      const Eigen::Ref<const Eigen::MatrixXd>& outliers,
      std::optional<double> max_sphere_radius = std::nullopt);

  const solvers::MathematicalProgram& prog() const { return prog_; }

  solvers::MathematicalProgram& mutable_prog() { return prog_; }

  const std::vector<Sphere>& spheres() const { return spheres_; }

  /** Getter for the binary variable phi.
   * ϕ has size num_polytopes * num_spheres.
   * ϕ(i, j) = 1 => the i'th polytope is covered by the j'th sphere.
   * ϕ(i, j) = 0 => nothing. (Note that ϕ(i, j) = 0 DOESN'T imply the i'th
   * polytope isn't covered by the j'th sphere).
   * We require that ∑ⱼ ϕ(i, j) = 1. (Note that actually each polytope can be
   * covered by multiple spheres, but we only assign one of these spheres as the
   * covering sphere for that polytope).
   *
   * Note that if we swap two spheres, then we still get a valid covering. To
   * break this symmetry, we impose an ordering on the spheres. If we denote the
   * first polytope covered by the j'th sphere as sⱼ, i.e., ϕ(sⱼ, j) = 1 and
   * ϕ(i, j) = 0 if i < sⱼ, then we require sⱼ < sₖ if j < k. If we look at the
   * matrix ϕ, it means the index of the leading non-zero entry along each row
   * increases row by row. The following assignment of ϕ is valid
   * \verbatim
   * 1 0 0 0 0
   * 0 1 1 0 1
   * 0 0 0 1 0
   * \endverbatim
   * The following assignment of ϕ is not valid
   * \verbatim
   * 1 0 0 1 0
   * 0 0 1 0 1
   * 0 1 0 0 0
   * \endverbatim
   * since the leading non-zero entry in the third row comes before the leading
   * non-zero entry in the second row.
   */
  // TODO(hongkai.dai): impose this total ordering on phi.
  const solvers::MatrixXDecisionVariable& phi() const { return phi_; }

  /** Getter for the binary variable zeta.
   * zeta_ has size size num_outliers * num_spheres.
   * zeta_(i, j) = 1 => the i'th outlier is outside the j'th sphere.
   */
  const solvers::MatrixXDecisionVariable& zeta() const { return zeta_; }

 private:
  /**
   * Adds the constraint that each polytope has to be in one of the spheres.
   */
  void AddPolytopeInSphereConstraint();

  /**
   * Adds the constraint that
   * if zeta_(outlier_idx, sphere_idx) = 0 => outlier_[outlier_idx] is outside
   * spheres_[sphere_idx]
   * if zeta_(outlier_idx, sphere_idx) = 1 => outlier_[outlier_idx] is inside
   * spheres_[sphere_idx].
   * Mathematically the constraint is
   * oⱼᵀoⱼ + aᵢᵀoⱼ + bᵢ ≥ -M₁ζ(j, i)
   * oⱼᵀoⱼ + aᵢᵀoⱼ + bᵢ ≤ M₂(1-ζ(j, i))
   * where oⱼ = outlier_[outlier_idx], i = sphere_idx. M₁, M₂ are positive big
   * constants.
   * @param max_sphere_radius_squared. The maximal value of r² with r being the
   * radius of the sphere. This is used to compute the big-M constant M₁.
   */
  void AddOutlierNotInSphereConstraint(int outlier_idx, int sphere_idx,
                                       double max_sphere_radius_squared);

  /**
   * Adds the constraint that a point is inside a sphere when the binary
   * variable is active. Namely z = 1 => a point p is inside the sphere {x | xᵀx
   * + aᵀx+b ≤ 0}. We impose the constraint pᵀp + aᵀx + b ≤ M(1−z) where M is a
   * big constant.
   */
  void AddPointInSphereConstraint(const Eigen::Ref<const Eigen::VectorXd>& pt,
                                  int sphere_idx,
                                  const symbolic::Variable& binary_var);

  /**
   * Computes the maximal of the squared distance between polytope vertices.
   */
  double ComputePolytopePairwiseDistanceSquaredMaximal() const;

  /**
   * Add the linear constraint that center of each sphere is within the convex
   * hull of the polytopes.
   */
  void AddSphereCenterInPolytopeConvexHull();

  /**
   * Add constraints on the upper bound of sphere radius.
   * For a sphere xᵀx + aᵢᵀx + bᵢ ≤ 0, the radius square is
   * aᵢᵀaᵢ/4 − bᵢ
   * So we impose the second-order cone constraint 4(bᵢ + rₘₐₓ²) ≥ aᵢᵀaᵢ
   * @return rₘₐₓ² The vector of variables whose i'th entry represent the upper
   * bound of the radius square for the i'th sphere.
   */
  solvers::VectorXDecisionVariable AddSphereRadiusUpperBound();

  solvers::MathematicalProgram prog_;
  std::vector<Sphere> spheres_;
  int dim_;

  std::vector<Eigen::MatrixXd> polytopes_;
  Eigen::MatrixXd outliers_;

  // phi_ has size num_polytopes * num_spheres.
  // phi_(i, j) = 1 => the i'th polytope is covered by the j'th sphere.
  solvers::MatrixXDecisionVariable phi_;

  // zeta_ has size size num_outliers * num_spheres.
  // zeta_(i, j) = 0 => the i'th outlier is outside the j'th sphere.
  // zeta_(i, j) = 1 => the i'th outlier is inside the j'th sphere.
  solvers::MatrixXDecisionVariable zeta_;

  // outlier_covered_[i] = 1 if outliers_.col(i) is covered by any of
  // the spheres.
  solvers::VectorXDecisionVariable outlier_covered_;

  // polytope_vertex_square_max is the maximal of vᵀv among all polytope
  // vertices v. This quantity will be used for computing big-M in
  // AddPolytopeInSphereConstraint.
  double polytope_vertex_square_max_{NAN};
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
