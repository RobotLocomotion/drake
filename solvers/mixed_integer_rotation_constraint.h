#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

namespace drake {
namespace solvers {

/**
 * We will relax the non-convex SO(3) constraint on rotation matrix R to
 * mixed-integer linear constraints. The formulation of these constraints are
 * described in
 * Global Inverse Kinematics via Mixed-integer Convex Optimization
 * by Hongkai Dai, Gregory Izatt and Russ Tedrake, ISRR, 2017
 *
 * This class is templated, based on the approach to relax SO(3) constraint.
 * The return type of adding the mixed-integer constraint to the program is
 * different, depending on the relaxation approach.
 */
class MixedIntegerRotationConstraintGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      MixedIntegerRotationConstraintGenerator)

  enum class ConstraintType {
    kBoxSphereIntersection,  ///< Relax SO(3) constraint by considering the
                             // intersection between boxes and the unit sphere
                             // surface.
    kBilinearMcCormick,      ///< Relax SO(3) constraint by considering the
                             // McCormick envelope on the bilinear product.
    kBoth,                   ///< Relax SO(3) constraint by considering both the
                             // intersection between boxes and the unit sphere
                             // surface, and the McCormick envelope on the
                             // bilinear product.
  };

  struct ReturnType {
    std::array<std::array<VectorXDecisionVariable, 3>, 3> B_;
    std::array<std::array<VectorXDecisionVariable, 3>, 3> lambda_;
  };

  MixedIntegerRotationConstraintGenerator(ConstraintType constraint_type,
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

  const Eigen::VectorXd& phi() const { return phi_; };

  const Eigen::VectorXd phi_nonnegative() const { return phi_nonnegative_; }

  ConstraintType constraint_type() const {return constraint_type_; }

  int num_intervals_per_half_axis() const {
    return num_intervals_per_half_axis_;
  }

  IntervalBinning interval_binning() const { return interval_binning_; }

 private:
  ConstraintType constraint_type_;
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
    MixedIntegerRotationConstraintGenerator::ConstraintType type);

std::ostream& operator<<(
    std::ostream& os,
    const MixedIntegerRotationConstraintGenerator::ConstraintType& type);

}  // namespace solvers
}  // namespace drake
