#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/lcmt_call_matlab.hpp"

/// A simple interface for (one-directional) RPC to a simple matlab remote
/// client.  Methods are provided to serialize our favorite data types into lcm
/// and then publish.  The interface is modeled after mexCallMATLAB
///   https://www.mathworks.com/help/matlab/apiref/mexcallmatlab.html
/// but we use C++11 to provide a much nicer interface.
///
/// To start the client, simply run lcm_call_matlab_client from your matlab
/// terminal.
///
/// The primary use case that this was designed for was to make MATLAB plotting
/// available in C++ without requiring the C++ code to link against MATLAB in
/// any way... (if MATLAB is not present, the lcm messages simply fall on deaf
/// ears).
///
/// Support for multi-function commands is provided by allowing return values to
/// be stored on the remote client, and reused by a simple "remote variable
/// reference" that is kept by the publisher.
///
/// See lcm_call_matlab_test.cc for some simple examples.

namespace drake {
namespace lcm {

/// Holds a reference to a return variable stored on the matlab client, which
/// can be passed back into a future lcm_call_matlab call.
struct LcmMatlabRemoteVariable {
 public:
  LcmMatlabRemoteVariable();
  //  ~LcmMatlabRemoteVariable(); // TODO(russt): send a destroy message on
  //  deletion

  const int64_t uid_{};
};

/// Serialize our favorite data types into the lcm_matlab_array structure.
/// To support a calling lcm_call_matlab for a new data type, simply implement
//  another one of these methods.

void ToLcmMatlabArray(const LcmMatlabRemoteVariable& var,
                             drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(double scalar,
                             drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                             drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(const std::string& str,
                             drake::lcmt_matlab_array* matlab_array);

// Helper methods for variadic template call in CallMatlab.
namespace internal {
inline void AssembleLcmCallMatlabMsg(drake::lcmt_call_matlab* msg,
                                     int* index) {
  // Intentionally left blank.  Base case for template recursion.
}

template <typename T, typename... Types>
void AssembleLcmCallMatlabMsg(drake::lcmt_call_matlab* msg, int* index,
                              T first, Types... args) {
  ToLcmMatlabArray(first, &(msg->rhs[*index]));
  *index += 1;
  AssembleLcmCallMatlabMsg(msg, index, args...);
}

// Simple wrapper to prevent the outside world from needing to worry about
// creating an lcm::LCM object.
// TODO(russt): support setting the channel name (via a
// LcmCallMatlabChannelPush/Pop).
void PublishLcmCallMatlab(const drake::lcmt_call_matlab& msg);

}  // namespace internal

/// Invokes a mexCallMATLAB call on the remote client.
///
/// @param num_outputs Number of return variables (left-side arguments) to
///     request from matlab.  As in matlab, you need not request all of the
///     outputs for any given method.
/// @param function_name Name of the matlab function to call.  Any argument
///     that could have been passed to mexCallMATLAB is allowed.
///     https://www.mathworks.com/help/matlab/apiref/mexcallmatlab.html
/// @param argument1 Any data type which has a ToLcmMatlabArray method
/// implemented.
/// @param argument2 Same as above.
/// ...
///
/// See lcm_call_matlab_test.cc for some simple examples.
template <typename... Types>
std::vector<LcmMatlabRemoteVariable> LcmCallMatlab(
    int num_outputs, const std::string& function_name, Types... args) {
  const int num_inputs = sizeof...(args);
  if (num_outputs < 0) num_outputs = 0;
  std::vector<LcmMatlabRemoteVariable> remote_vars(num_outputs);

  drake::lcmt_call_matlab msg;
  msg.nlhs = num_outputs;
  msg.lhs.resize(num_outputs);
  for (int i = 0; i < num_outputs; i++) {
    msg.lhs[i] = remote_vars[i].uid_;
  }

  int index = 0;
  msg.nrhs = num_inputs;
  msg.rhs.resize(num_inputs);
  internal::AssembleLcmCallMatlabMsg(&msg, &index, args...);

  msg.function_name = function_name;
  internal::PublishLcmCallMatlab(msg);
  return remote_vars;
}

/// Special case the call with zero outputs, since it's so common.
template <typename... Types>
void LcmCallMatlab(const std::string& function_name, Types... args) {
  LcmCallMatlab(0, function_name, args...);
}

}  // namespace lcm
}  // namespace drake
