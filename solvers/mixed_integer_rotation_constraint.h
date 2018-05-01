#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

namespace drake {
namespace solvers {
enum class MixedIntegerRotationConstraintType {
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

std::string to_string(MixedIntegerRotationConstraintType type); 

std::ostream& operator<<(std::ostream& os,
                         const MixedIntegerRotationConstraintType& type);

/**
 * The return type of adding mixed integer relaxation for SO(3) constraint, if
 * the approach is to consider the intersection between the box and the unit
 * sphere surface.
 */
struct MixedIntegerRotationConstraintBoxSphereIntersectionReturn {
  // CRpos[k](i, j) = 1 if φ₊(k) ≤ R(i, j) ≤ φ₊(k + 1)
  // CRneg[k](i, j) = 1 if -φ₊(k+1) ≤ R(i, j) ≤ -φ₊(k)
  std::vector<Matrix3<symbolic::Expression>> CRpos;
  std::vector<Matrix3<symbolic::Expression>> CRneg;
  // If we use logarithmic binning, and 
  // [B[0](i, j), B[1](i, j), ..., B[N](i, j)] represents integer k in reflected
  // Gray code, then φ(k) ≤ R(i, j) ≤ φ(k + 1).
  // If we use linear binning, and B.size() = 2 * N, then
  // B[N + k](i, j) = 1 if R(i, j) ≥ φ₊(k),
  // B[k](i, j) = 1 if R(i, j) ≤ -φ₊(k)
  // Namely the first half of B is `BRneg` in the code, while the second half
  // of B is `BRpos` in the mixed_integer_rotation_constraint.cc
  std::vector<Matrix3<symbolic::Variable>> B;
};

/**
 * The return type of adding mixed integer relaxation for SO(3) constraint, if
 * the approach is to consider both the intersection between the box and the
 * unit sphere surface, and the McCormick envelope of bilinear products.
 */
struct MixedIntegerRotationConstraintBothReturn {
  std::array<std::array<VectorXDecisionVariable, 3>, 3> B;
  std::vector<Matrix3<symbolic::Expression>> CRpos;
  std::vector<Matrix3<symbolic::Expression>> CRneg;
  std::vector<Matrix3<symbolic::Variable>> BRpos;
  std::vector<Matrix3<symbolic::Variable>> BRneg;
};

/**
 * The return type for adding mixed integer relaxation for SO(3) constraint.
 * We will specialize the return type based on different constraint types.
 */
template <MixedIntegerRotationConstraintType ConstraintType>
struct AddMixedIntegerRotationConstraintReturn {
  typedef void Type;
};

template <>
struct AddMixedIntegerRotationConstraintReturn<
    MixedIntegerRotationConstraintType::kBoxSphereIntersection> {
  typedef MixedIntegerRotationConstraintBoxSphereIntersectionReturn Type;
};

template <>
struct AddMixedIntegerRotationConstraintReturn<
    MixedIntegerRotationConstraintType::kBilinearMcCormick> {
  typedef std::array<std::array<VectorXDecisionVariable, 3>, 3> Type;
};

template <>
struct AddMixedIntegerRotationConstraintReturn<
    MixedIntegerRotationConstraintType::kBoth> {
  typedef MixedIntegerRotationConstraintBothReturn Type;
};

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
template <MixedIntegerRotationConstraintType ConstraintType>
class MixedIntegerRotationConstraintGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      MixedIntegerRotationConstraintGenerator)

  MixedIntegerRotationConstraintGenerator(int num_intervals_per_half_axis,
                                          IntervalBinning interval_binning);

  /**
   * Add the mixed-integer linear constraints to the optimization program, as
   * an relaxation of SO(3) constraint on the rotation matrix `R`.
   * @param prog The optimization program to which the mixed-integer constraints
   * (and additional variables) are added.
   * @param R The rotation matrix on which the SO(3) constraint is imposed.
   */
  typename AddMixedIntegerRotationConstraintReturn<ConstraintType>::Type
  AddToProgram(MathematicalProgram* prog,
               const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) const;

  const Eigen::VectorXd& phi() const { return phi_; };

  const Eigen::VectorXd phi_nonnegative() const { return phi_nonnegative_; }

  int num_intervals_per_half_axis() const {
    return num_intervals_per_half_axis_;
  }

  IntervalBinning interval_binning() const { return interval_binning_; }

 private:
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
}  // namespace solvers
}  // namespace drake
