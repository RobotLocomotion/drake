#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

namespace drake {
namespace solvers {

/**
 * We relax the non-convex SO(3) constraint on rotation matrix R to
 * mixed-integer linear constraints. The formulation of these constraints are
 * described in
 * Global Inverse Kinematics via Mixed-integer Convex Optimization
 * by Hongkai Dai, Gregory Izatt and Russ Tedrake, ISRR, 2017
 *
 * To relax SO(3) constraint on rotation matrix R, we divide the range [-1, 1]
 * (the range of each entry in R) into smaller intervals, and then relax the
 * SO(3) constraint within each interval. We provide 3 approaches for relaxation
 * 1. By replacing the bilinear product with a new variable, in the McCormick
 * envelope of the bilinear product w = x * y.
 * 2. By consider the intersection region between axis-aligned boxes, and the
 * surface of a unit sphere in 3D.
 * 3. By combining the two approaches above.
 * These three approaches give different relaxation of SO(3) constraint (the
 * feasible sets to each relaxation are different), and different computation
 * speed. The user can switch between the approaches to find the best fit for
 * the problem.
 *
 * @note If you have several rotation matrices, all need to be relaxed through
 * mixed-integer constraint, then you can create a single
 * MixedIntegerRotationConstraintGenerator object, and add the mixed-integer
 * constraint to each rotation matrix, by calling
 * MixedIntegerRotationConstraintGenerator::AddToProgram function repeatedly.
 */
class MixedIntegerRotationConstraintGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      MixedIntegerRotationConstraintGenerator)

  enum class Approach {
    kBoxSphereIntersection,  ///< Relax SO(3) constraint by considering the
                             ///< intersection between boxes and the unit sphere
                             ///< surface.
    kBilinearMcCormick,      ///< Relax SO(3) constraint by considering the
                             ///< McCormick envelope on the bilinear product.
    kBoth,                   ///< Relax SO(3) constraint by considering both the
                             ///< intersection between boxes and the unit sphere
                             ///< surface, and the McCormick envelope on the
                             ///< bilinear product.
  };

  struct ReturnType {
    /**
     * B_[i][j] represents in which interval R(i, j) lies. If we use linear
     * binning, then B_[i][j] is of length 2 * num_intervals_per_half_axis_.
     * B_[i][j](k) = 1 => φ(k) ≤ R(i, j) ≤ φ(k + 1)
     * B_[i][j](k) = 0 => R(i, j) ≥ φ(k + 1) or R(i, j) ≤ φ(k)
     * If we use logarithmic binning, then B_[i][j] is of length
     * 1 + log₂(num_intervals_per_half_axis_). If B_[i][j] represents integer
     * k in reflected Gray code, then R(i, j) is in the interval [φ(k), φ(k+1)].
     */
    std::array<std::array<VectorXDecisionVariable, 3>, 3> B_;
    /**
     * λ_[i][j] is of length 2 * num_intervals_per_half_axis_ + 1, such that
     * R(i, j) = φᵀ * λ_[i][j]. Notice that λ_[i][j] satisfies the special
     * ordered set of type 2 (SOS2) constraint. Namely at most two entries in
     * λ_[i][j] can be non-negative, and these two entries have to
     * be consecutive. Mathematically
     * ```
     * ∑ₖ λ_[i][j](k) = 1
     * λ_[i][j](k) ≥ 0 ∀ k
     * ∃ m s.t λ_[i][j](n) = 0 if n ≠ m and n ≠ m+1
     * ```
     */
    std::array<std::array<VectorXDecisionVariable, 3>, 3> lambda_;
  };

  /**
   * Constructor
   * @param approach Refer to MixedIntegerRotationConstraintGenerator::Approach
   * for the details.
   * @param num_intervals_per_half_axis We will cut the range [-1, 1] evenly
   * to 2 * `num_intervals_per_half_axis` small intervals. The number of binary
   * variables will depend on the number of intervals.
   * @param interval_binning The binning scheme we use to add SOS2 constraint
   * with binary variables. If interval_binning = kLinear, then we will add
   * 9 * 2 * `num_intervals_per_half_axis binary` variables;
   * if interval_binning = kLogarithmic, then we will add
   * 9 * (1 + log₂(num_intervals_per_half_axis)) binary variables. Refer to
   * AddLogarithmicSos2Constraint and AddSos2Constraint for more details.
   */
  MixedIntegerRotationConstraintGenerator(Approach approach,
                                          int num_intervals_per_half_axis,
                                          IntervalBinning interval_binning);

  /**
   * Add the mixed-integer linear constraints to the optimization program, as
   * an relaxation of SO(3) constraint on the rotation matrix `R`.
   * @param prog The optimization program to which the mixed-integer constraints
   * (and additional variables) are added.
   * @param R The rotation matrix on which the SO(3) constraint is imposed.
   */
  ReturnType AddToProgram(
      MathematicalProgram* prog,
      const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) const;

  /** Getter for φ. */
  const Eigen::VectorXd& phi() const { return phi_; }

  /** Getter for φ₊, the non-negative part of φ. */
  const Eigen::VectorXd phi_nonnegative() const { return phi_nonnegative_; }

  /** Getter for approach. */
  Approach approach() const { return approach_; }

  /** Getter for num_intervals_per_half_axis. */
  int num_intervals_per_half_axis() const {
    return num_intervals_per_half_axis_;
  }

  IntervalBinning interval_binning() const { return interval_binning_; }

 private:
  Approach approach_;
  int num_intervals_per_half_axis_;
  IntervalBinning interval_binning_;
  // φ(i) = -1 + 1 / num_intervals_per_half_axis_ * i
  Eigen::VectorXd phi_;
  // φ₊(i) = 1 / num_intervals_per_half_axis_ * i
  Eigen::VectorXd phi_nonnegative_;

  // When considering the intersection between the box and the sphere surface,
  // we will compute the vertices of the intersection region, and find one tight
  // halfspace nᵀx ≥ d, such that all points on the intersection surface satisfy
  // this halfspace constraint. For intersection region between the box
  // [φ₊(xi), φ₊(xi+1)] x [φ₊(yi), φ₊(yi+1)] x [φ₊(zi), φ₊(zi+1)] and the sphere
  // surface, the vertices of the intersection region is in
  // box_sphere_intersection_vertices_[xi][yi][zi], and the halfspace is
  // (box_sphere_intersection_halfspace_[xi][yi][zi].first)ᵀ x ≥
  // box_sphere_intersection_halfspace[xi][yi][zi].second
  std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>>
      box_sphere_intersection_vertices_;
  std::vector<std::vector<std::vector<std::pair<Eigen::Vector3d, double>>>>
      box_sphere_intersection_halfspace_;
};

std::string to_string(
    MixedIntegerRotationConstraintGenerator::Approach type);

std::ostream& operator<<(
    std::ostream& os,
    const MixedIntegerRotationConstraintGenerator::Approach& type);

using AddRotationMatrixBoxSphereIntersectionReturnType =
    std::tuple<std::vector<Matrix3<symbolic::Expression>>,
               std::vector<Matrix3<symbolic::Expression>>,
               std::vector<MatrixDecisionVariable<3, 3>>,
               std::vector<MatrixDecisionVariable<3, 3>>>;

/**
 * Adds binary variables that constrain the value of the column *and* row
 * vectors of R, in order to add the following (in some cases non-convex)
 * constraints as an MILP.  Specifically, for column vectors Ri, we constrain:
 * - forall i, |Ri| = 1 ± envelope,
 * - forall i,j. i ≠ j, Ri.dot(Rj) = 0 ± envelope,
 * - R2 = R0.cross(R1) ± envelope,
 *      and again for R0=R1.cross(R2), and R1=R2.cross(R0).
 * Then all of the same constraints are also added to R^T.  The size of the
 * envelope decreases quickly as num_binary_variables_per_half_axis is
 * is increased.
 *
 * Note: Creates `9*2*num_binary_variables_per_half_axis binary` variables named
 * "BRpos*(*,*)" and "BRneg*(*,*)", and the same number of continuous variables
 * named "CRpos*(*,*)" and "CRneg*(*,*)".
 *
 * Note: The particular representation/algorithm here was developed in an
 * attempt:
 *  - to enable efficient reuse of the variables between the constraints
 *    between multiple rows/columns (e.g. the constraints on Rᵀ use the same
 *    variables as the constraints on R), and
 *  - to facilitate branch-and-bound solution techniques -- binary regions are
 *    layered so that constraining one region establishes constraints
 *    on large portions of SO(3), and confers hopefully "useful" constraints
 *    the on other binary variables.
 * @param prog The mathematical program to which the constraints are added.
 * @param R The rotation matrix
 * @param num_intervals_per_half_axis number of intervals for a half axis.
 * @param limits The angle joints for space fixed z-y-x representation of the
 * rotation. @default is no constraint. @see RollPitchYawLimitOptions
 * @retval NewVars  Included the newly added variables
 * <CRpos, CRneg, BRpos, BRneg>. All new variables can only take values either
 * 0 or 1. `CRpos` and `CRneg` are declared as continuous variables, while
 * `BRpos` and `BRneg` are declared as binary variables.
 * The definition for these variables are
 * <pre>
 *   CRpos[k](i, j) = 1 => k / N <= R(i, j) <= (k+1) / N
 *   CRneg[k](i, j) = 1 => -(k+1) / N <= R(i, j) <= -k / N
 *   BRpos[k](i, j) = 1 => R(i, j) >= k / N
 *   BRneg[k](i, j) = 1 => R(i, j) <= -k / N
 * </pre>
 * where `N` is `num_intervals_per_half_axis`.
 * @note This method uses the same approach as
 * MixedIntegerRotationConstraintGenerator with kBoxSphereIntersection, namely
 * the feasible sets to the relaxation are the same. But they use different sets
 * of binary variables, and thus the computation speed can be different inside
 * optimization solvers.
 */

AddRotationMatrixBoxSphereIntersectionReturnType
AddRotationMatrixBoxSphereIntersectionMilpConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    int num_intervals_per_half_axis = 2);

}  // namespace solvers
}  // namespace drake
