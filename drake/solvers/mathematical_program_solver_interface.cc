#include "drake/solvers/mathematical_program_solver_interface.h"

#include <sstream>
#include <stdexcept>

namespace drake {
namespace solvers {
std::ostream& operator<<(
    std::ostream& os,
    const SolverType& solver_type) {
  switch (solver_type) {
    case SolverType::kDReal : {
      os << "dReal";
      break;
    }
    case SolverType::kEqualityConstrainedQP : {
      os << "Equality constrained QP";
      break;
    }
    case SolverType::kGurobi : {
      os << "Gurobi";
      break;
    }
    case SolverType::kIpopt : {
      os << "IPOPT";
      break;
    }
    case SolverType::kLinearSystem : {
      os << "Linear system";
      break;
    }
    case SolverType::kMobyLCP : {
      os << "Moby LCP";
      break;
    }
    case SolverType::kMosek : {
      os << "Mosek";
      break;
    }
    case SolverType::kNlopt : {
      os << "NLopt";
      break;
    }
    case SolverType::kSnopt : {
      os << "SNOPT";
      break;
    }
    default : throw std::runtime_error("Unsupported solver");
  }
  return os;
}

std::string Name(SolverType solver_type) {
  std::ostringstream oss;
  oss << solver_type;
  return oss.str();
}

std::string MathematicalProgramSolverInterface::SolverName() const {
  return Name(solver_type_);
}
}  // namespace solvers
}  // namespace drake
