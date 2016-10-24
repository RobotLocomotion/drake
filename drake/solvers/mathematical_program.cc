#include "mathematical_program.h"

#include "equality_constrained_qp_solver.h"
#include "ipopt_solver.h"
#include "linear_system_solver.h"
#include "moby_lcp_solver.h"
#include "nlopt_solver.h"
#include "snopt_solver.h"

namespace drake {
namespace solvers {

namespace {


// Solver for simple linear systems of equalities
AttributesSet kLinearSystemSolverCapabilities = kLinearEqualityConstraint;

// Solver for equality-constrained QPs
AttributesSet kEqualityConstrainedQPCapabilities = (
  kQuadraticCost | kLinearCost | kLinearEqualityConstraint);

// Solver for Linear Complementarity Problems (LCPs)
AttributesSet kMobyLcpCapabilities = kLinearComplementarityConstraint;

// Solver for Quadratic Programs (QPs); commented out until Gurobi is ready
// to land.
// AttributesSet kGurobiCapabilities = (
//     kLinearEqualityConstraint | kLinearInequalityConstraint |
//     kLinearCost | kQuadraticCost);

// Solvers for generic systems of constraints and costs.
AttributesSet kGenericSolverCapabilities = (
    kGenericCost | kGenericConstraint |
    kQuadraticCost | kQuadraticConstraint |
    kLinearCost | kLinearConstraint | kLinearEqualityConstraint);

// Returns true iff no capabilities are in required and not in available.
bool is_satisfied(AttributesSet required, AttributesSet available) {
  return ((required & ~available) == kNoCapabilities);
}

}  // anon namespace


enum {
  INITIAL_VARIABLE_ALLOCATION_NUM = 100
};  // not const static int because the VectorXd constructor takes a reference
// to int so it is odr-used (see
// https://gcc.gnu.org/wiki/VerboseDiagnostics#missing_static_const_definition)


MathematicalProgram::MathematicalProgram()
    : num_vars_(0),
      x_initial_guess_(
                  static_cast<Eigen::Index>(INITIAL_VARIABLE_ALLOCATION_NUM)),
      solver_result_(0),
      required_capabilities_(kNoCapabilities),
      ipopt_solver_(new IpoptSolver()),
      nlopt_solver_(new NloptSolver()),
      snopt_solver_(new SnoptSolver()),
      moby_lcp_solver_(new MobyLCPSolver()),
      linear_system_solver_(new LinearSystemSolver()),
      equality_constrained_qp_solver_(new EqualityConstrainedQPSolver()) {}


SolutionResult MathematicalProgram::Solve()  {
  // This implementation is simply copypasta for now; in the future we will
  // want to tweak the order of preference of solvers based on the types of
  // constraints present.

  if (is_satisfied(required_capabilities_, kLinearSystemSolverCapabilities) &&
      linear_system_solver_->available()) {
    // TODO(ggould-tri) Also allow quadratic objectives whose matrix is
    // Identity: This is the objective function the solver uses anyway when
    // underconstrainted, and is fairly common in real-world problems.
    return linear_system_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_,
                          kEqualityConstrainedQPCapabilities) &&
            equality_constrained_qp_solver_->available()) {
    return equality_constrained_qp_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kMobyLcpCapabilities) &&
             moby_lcp_solver_->available()) {
    return moby_lcp_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             snopt_solver_->available()) {
    return snopt_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             ipopt_solver_->available()) {
    return ipopt_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             nlopt_solver_->available()) {
    return nlopt_solver_->Solve(*this);
  } else {
    throw std::runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
}

}  // namespace solvers
}  // namespace drake
