#pragma once
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/rational_function.h"
#include "drake/geometry/optimization/c_iris_collision_geometry.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 Contains the information to enforce a pair of geometries are separated by a
 plane. The conditions are that certain rational functions should be always
 positive.
 */
struct PlaneSeparatesGeometries {
  PlaneSeparatesGeometries(
      std::vector<symbolic::RationalFunction> m_positive_side_rationals,
      std::vector<symbolic::RationalFunction> m_negative_side_rationals,
      int m_plane_index)
      : positive_side_rationals{std::move(m_positive_side_rationals)},
        negative_side_rationals{std::move(m_negative_side_rationals)},
        plane_index{m_plane_index} {}

  const std::vector<symbolic::RationalFunction>& rationals(
      PlaneSide plane_side) const {
    return plane_side == PlaneSide::kPositive ? positive_side_rationals
                                              : negative_side_rationals;
  }
  const std::vector<symbolic::RationalFunction> positive_side_rationals;
  const std::vector<symbolic::RationalFunction> negative_side_rationals;
  int plane_index{-1};
};

struct FindSeparationCertificateOptions {
  FindSeparationCertificateOptions() = default;

  virtual ~FindSeparationCertificateOptions() = default;
  // We can find the certificate for each pair of geometries in parallel.
  // num_threads specifies how many threads we run in parallel. If num_threads
  // <=0, then we use all available threads on the computer.
  int num_threads{-1};

  // If verbose set to true, then we will print some information to the
  // terminal.
  bool verbose{false};

  // The solver invoked for the sos program.
  solvers::SolverId solver_id{solvers::MosekSolver::id()};

  // If the SOS in one thread fails, then don't launch any more threads.
  bool terminate_at_failure{true};

  // The solver options used for the SOS program.
  std::optional<solvers::SolverOptions> solver_options{std::nullopt};

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FindSeparationCertificateOptions)
};

/**
 We certify that a pair of geometries is collision free by finding the
 separating plane over a range of configuration. The Lagrangian multipliers
 used for certifying this condition will differ in derived classes. This struct
 contains the the separating plane {x | aᵀx+b=0 } and derived classes may store
 the Lagrangians certifying that the plane separates the two geometries in
 separating_planes()[plane_index] in the C-space region.
 */
struct SeparationCertificateResultBase {
  SeparationCertificateResultBase() {}
  virtual ~SeparationCertificateResultBase() = default;

  int plane_index{-1};
  /** The separating plane is { x | aᵀx+b=0 } */
  Vector3<symbolic::Polynomial> a;
  symbolic::Polynomial b;
  // The value of the plane.decision_variables at solution. This field is used
  // for debugging.
  Eigen::VectorXd plane_decision_var_vals;

  // The result of solving a SeparationCertificateProgramBase
  solvers::MathematicalProgramResult result;

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateResultBase)
};

struct SeparationCertificateProgramBase {
  SeparationCertificateProgramBase()
      : prog{new solvers::MathematicalProgram()} {}

  virtual ~SeparationCertificateProgramBase() = default;
  /// The program that stores all the constraints to search for the separating
  /// plane and Lagrangian multipliers as certificate.
  copyable_unique_ptr<solvers::MathematicalProgram> prog;
  int plane_index{-1};

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateProgramBase)
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
