#pragma once

#include <memory>

#include "drake/common/trajectories/piecewise_polynomial.h"
// #include "drake/common/sorted_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

using common::trajectories::PiecewisePolynomial;

namespace drake {
namespace multibody {
/**
 * Solves a Time Optimal Path Parameterization based on Reachability Analysis
 * (TOPPRA) to find the fastest traversal of a given path, satisfying the given
 * constraints.
 * The approach is described in "A new approach to Time-Optimal Path
 * Parameterization based on Reachability Analysis" by Hung Pham and Quang Cuong
 * Pham, IEEE Transactions on Robotics, 2018.
 *
 * @ingroup planning
 */
class Toppra {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Toppra)

  ~Toppra() {}

  /**
   * Constructs an inverse kinematics problem for a MultibodyPlant.
   * This constructor will create and own a context for @param plant.
   * @param path The trajectory on which the toppra problem will be solved.
   * @param plant The robot that will follow the solved trajectory.  Used for
   * enforcing torque and frame specific constraints.
   */
  explicit Toppra(const PiecewisePolynomial<double>& path,
                  const MultibodyPlant<double>& plant);

  /** Getter for the optimization program constructed by Toppra. */
  const solvers::MathematicalProgram& prog() const { return *prog_; }

  /** Getter for the optimization program constructed by Toppra. */
  solvers::MathematicalProgram* get_mutable_prog() const { return prog_.get(); }

 private:
  std::unique_ptr<solvers::MathematicalProgram> backward_prog_;
  solvers::VectorXDecisionVariable backward_vars_;
  std::unique_ptr<solvers::MathematicalProgram> forward_prog_;
  solvers::VectorXDecisionVariable forward_vars_;
  const MultibodyPlant<double>& plant_;
};
}  // namespace multibody
}  // namespace drake
