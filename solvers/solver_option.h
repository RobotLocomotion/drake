#pragma once

#include <string>
#include <unordered_map>
#include <map>

#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
class SolverOption {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverOption)

  SolverOption();

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, double option_value);

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, int option_value);

  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option,
                       const std::string& option_value);

  const std::unordered_map<std::string, double>& GetSolverOptionsDouble(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, int>& GetSolverOptionsInt(
      const SolverId& solver_id) const;

  const std::unordered_map<std::string, std::string>& GetSolverOptionsStr(
      const SolverId& solver_id) const;

 private:
  std::map<SolverId, std::unordered_map<std::string, double>>
      solver_options_double_;
  std::map<SolverId, std::unordered_map<std::string, int>>
      solver_options_int_;
  std::map<SolverId, std::unordered_map<std::string, std::string>>
      solver_options_str_;

  const std::unordered_map<std::string, double> solver_options_double_empty_;
  const std::unordered_map<std::string, int> solver_options_int_empty_;
  const std::unordered_map<std::string, std::string> solver_options_str_empty_;
};
}  // namespace solvers
}  // namespace drake
