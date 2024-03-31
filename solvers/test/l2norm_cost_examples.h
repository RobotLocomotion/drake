#pragma once

#include <array>
#include <optional>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
// Test a program without constraints
// min |x-p1|₂ + |x-p2|₂ + |x-p3|₂
class ShortestDistanceToThreePoints {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShortestDistanceToThreePoints)

  ShortestDistanceToThreePoints();

  const MathematicalProgram& prog() const { return prog_; }

  void CheckSolution(
      const SolverInterface& solver,
      const std::optional<SolverOptions>& solver_options = std::nullopt,
      double tol = 1E-5) const;

 private:
  MathematicalProgram prog_;
  Vector3<symbolic::Variable> x_;
  std::array<Eigen::Vector3d, 3> pts_;
};

// Compute the shortest distance from a cylinder to a point in 3D.
// min |x - pt|₂
// -1 <= x[2] <= 1
// x[0]² + x[1]² <= 4
class ShortestDistanceFromCylinderToPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShortestDistanceFromCylinderToPoint)

  ShortestDistanceFromCylinderToPoint();

  void CheckSolution(
      const SolverInterface& solver,
      const std::optional<SolverOptions>& solver_options = std::nullopt,
      double tol = 1E-5) const;

 private:
  MathematicalProgram prog_;
  Vector3<symbolic::Variable> x_;
  Eigen::Vector3d pt_;
};
}  // namespace test
}  // namespace solvers
}  // namespace drake
