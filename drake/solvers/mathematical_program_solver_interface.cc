#include "drake/solvers/mathematical_program_solver_interface.h"

#include <stdexcept>
#include <sstream>

namespace drake {
namespace solvers {
std::ostream& operator<<(
    std::ostream& os,
    const MathematicalProgramSolverInterface::Solver& solver_type) {
  switch (solver_type) {
    case MathematicalProgramSolverInterface::Solver::kDReal : {
      os << "dReal";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kEqualityConstrainedQP : {
      os << "Equality constrained QP";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kGurobi : {
      os << "Gurobi";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kIpopt : {
      os << "IPOPT";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kLinearSystem : {
      os << "Linear system";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kMobyLCP : {
      os << "Moby LCP";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kMosek : {
      os << "Mosek";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kNlopt : {
      os << "NLopt";
      break;
    }
    case MathematicalProgramSolverInterface::Solver::kSnopt : {
      os << "SNOPT";
      break;
    }
    default : throw std::runtime_error("Unsupported solver");
  }
  return os;
}

std::string Name(MathematicalProgramSolverInterface::Solver solver_type) {
  std::stringbuf str;
  std::ostream stream(nullptr);
  stream.rdbuf(&str);
  stream << solver_type;
  return str.str();
}

std::string MathematicalProgramSolverInterface::SolverName() const {
  return Name(solver_type_);
}
}  // namespace solvers
}  // namespace drake
