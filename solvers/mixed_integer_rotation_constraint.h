#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

/**
 * @file
 * We will relax the non-convex SO(3) constraint on rotation matrix R to
 * mixed-integer linear constraints. The formulation of these constraints are
 * described in
 * Global Inverse Kinematics via Mixed-integer Convex Optimization
 * by Hongkai Dai, Gregory Izatt and Russ Tedrake, ISRR, 2017
 */

namespace drake {
namespace solvers {
/**
 * Relax the non-convex SO(3) constraint on rotation matrix, by considering the
 * intersection between boxes and unit length sphere surface. The boxes are
 * obtained by cutting the x, y and z axes into small intervals, and each cell
 * in 3D space is a box.
 * The formulation of these constraints are described in
 * Global Inverse Kinematics via Mixed-integer Convex Optimization
 * by Hongkai Dai, Gregory Izatt and Russ Tedrake, ISRR, 2017
 */
class MixedIntegerRotationConstraintByBoxSphereIntersectionGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      MixedIntegerRotationConstraintByBoxSphereIntersectionGenerator)

  MixedIntegerRotationConstraintByBoxSphereIntersectionGenerator(
      int num_intervals_per_half_axis, IntervalBinning interval_binning);

  struct ReturnType {
    std::vector<Matrix3<symbolic::Expression>> CRpos;
    std::vector<Matrix3<symbolic::Expression>> CRneg;
    std::vector<Matrix3<symbolic::Variable>> BRpos;
    std::vector<Matrix3<symbolic::Variable>> BRneg;
  };

  ReturnType AddToProgram(MathematicalProgram* prog,
               const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R);

 private:
  const int num_intervals_per_half_axis_;
  const RelaxationType relaxation_type_;
  // φ(i) = -1 + 1 / num_intervals_per_half_axis_ * i
  Eigen::VectorXd phi_;
  // φ₊(i) = 1 / num_intervals_per_half_axis_ * i
  Eigen::VectorXd phi_nonnegative_;

  // When considering the intersection between the box and the sphere surface,
  // we will find one halfspace nᵀx ≥ d, such that all points on the
  // intersection surface satisfy this halfspace constraint. For intersection
  // region between the box [φ₊(xi), φ₊(xi+1)] x [φ₊(yi), φ₊(yi+1)] x
  // [φ₊(zi), φ₊(zi+1)] and the sphere surface, this halfspace is
  // (intersection_halfspace_[xi][yi][zi].first)ᵀ x ≥
  // intersection_halfspace[xi][yi][zi].second
  std::vector<std::vector<std::vector<std::pair<Eigen::Vector3d, double>>>>
      intersection_halfspace_
};
}  // namespace solvers
}  // namespace drake
