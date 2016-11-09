#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcmt_call_matlab.hpp"

struct LcmMatlabRemoteVariable {
 public:
  LcmMatlabRemoteVariable();
  //  ~LcmMatlabRemoteVariable(); // TODO(russt): send a destroy message on
  //  deletion

  const int64_t uid;
};

extern void to_lcm_matlab_array(const LcmMatlabRemoteVariable& var,
                                drake::lcmt_matlab_array& matlab_array);

extern void to_lcm_matlab_array(double scalar,
                                drake::lcmt_matlab_array& matlab_array);

extern void to_lcm_matlab_array(const Eigen::Ref<Eigen::MatrixXd>& mat,
                                drake::lcmt_matlab_array& matlab_array);

extern void to_lcm_matlab_array(const std::string& str,
                                drake::lcmt_matlab_array& matlab_array);

namespace internal {
void assemble_lcm_call_matlab_msg(drake::lcmt_call_matlab& msg,
                                  unsigned int& index) {
  // intentionally left blank.  base case for template recursion.
}

template <typename T, typename... Types>
void assemble_lcm_call_matlab_msg(drake::lcmt_call_matlab& msg,
                                  unsigned int& index, T first, Types... args) {
  to_lcm_matlab_array(first, msg.rhs[index++]);
  assemble_lcm_call_matlab_msg(msg, index, args...);
};

void publish_lcm_call_matlab(const std::string& channel,
                             const drake::lcmt_call_matlab& msg);
}

template <typename... Types>
std::vector<LcmMatlabRemoteVariable> lcm_call_matlab(
    const std::string& channel, const std::string& function_name,
    const unsigned int num_outputs, Types... args) {
  const int num_inputs = sizeof...(args);
  std::vector<LcmMatlabRemoteVariable> remote_vars(num_outputs);

  drake::lcmt_call_matlab msg;
  msg.nlhs = num_outputs;
  msg.lhs.resize(num_outputs);
  for (int i = 0; i < num_outputs; i++) {
    msg.lhs[i] = remote_vars[i].uid;
  }

  unsigned int index = 0;
  msg.nrhs = num_inputs;
  msg.rhs.resize(num_inputs);
  internal::assemble_lcm_call_matlab_msg(msg, index, args...);

  msg.function_name = function_name;
  internal::publish_lcm_call_matlab(channel, msg);
  return remote_vars;
}

template <typename... Types>
std::vector<LcmMatlabRemoteVariable> lcm_call_matlab(
    const std::string& function_name, const unsigned int num_outputs,
    Types... args) {
  return lcm_call_matlab("LCM_CALL_MATLAB", function_name, num_outputs,
                         args...);
}
