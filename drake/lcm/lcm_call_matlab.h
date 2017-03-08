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

/// Serialize our favorite data types into the lcm_matlab_array structure.
/// To support a calling lcm_call_matlab for a new data type, simply implement
//  another one of these methods.

class LcmMatlabRemoteVariable;
void ToLcmMatlabArray(const LcmMatlabRemoteVariable& var,
                      drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(double scalar, drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>>&
        mat,
    drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                      drake::lcmt_matlab_array* matlab_array);

void ToLcmMatlabArray(const std::string& str,
                      drake::lcmt_matlab_array* matlab_array);

// Helper methods for variadic template call in CallMatlab.
namespace internal {
inline void AssembleLcmCallMatlabMsg(drake::lcmt_call_matlab* msg, int* index) {
  // Intentionally left blank.  Base case for template recursion.
}

template <typename T, typename... Types>
void AssembleLcmCallMatlabMsg(drake::lcmt_call_matlab* msg, int* index, T first,
                              Types... args) {
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

// forward declaration:
template <typename... Types>
LcmMatlabRemoteVariable LcmCallMatlabSingleOutput(
    const std::string& function_name, Types... args);

/// Holds a reference to a variable stored on the matlab client, which can be
/// passed back into a future lcm_call_matlab call.
class LcmMatlabRemoteVariable {
 public:
  LcmMatlabRemoteVariable();
  //  ~LcmMatlabRemoteVariable(); // TODO(russt): send a destroy message on
  //  deletion

  int64_t unique_id() const { return unique_id_; }

  /// Creates a new remote variable that contains the data at the prescribed
  /// index.  Supported calls are, for instance:
  /// <pre>
  ///   var(1)      // Access the first element.
  ///   var(1,2)    // Access row 1, column 2.
  ///   var(3,":")  // Access the entire third row.
  ///   var(Eigen::Vector2d(1,2),":")   // Access the first and second rows.
  /// </pre>
  ///
  /// Note that a tempting use case which is NOT supported (directly) is
  /// <pre>
  ///   var("1:10")  // ERROR!
  /// </pre>
  /// Matlab doesn't work that way.  Instead use, e.g.
  /// <pre>
  ///   var(Eigen::VectorXd::LinSpaced(10,1,10))  // Access elements 1:10.
  /// <pre>
  /// Note: yes, vector indices in Matlab are doubles.
  template <typename... Types>
  LcmMatlabRemoteVariable operator()(Types... args) const {
    LcmMatlabRemoteVariable s = AssembleSubstruct(args...);

    return LcmCallMatlabSingleOutput("subsref", *this, s);
  }

  /// Creates a new remote variable that contains the data at the prescribed
  /// index.  Supported calls are, for instance:
  /// <pre>
  ///   var.subsasgn(val,1)               // Set the first element to val.
  ///   var.subsasgn(val,1,2)             // Set row 1, column 2.
  ///   var.subsasgn(val,3,":")           // Set the entire third row.
  ///   var(val,Eigen::Vector2d(1,2),":") // Set the first and second rows.
  /// </pre>
  ///
  /// Note that a tempting use case which is NOT supported (directly) is
  /// <pre>
  ///   var(val,"1:10")  // ERROR!
  /// </pre>
  /// Matlab doesn't work that way.  Instead use, e.g.
  /// <pre>
  ///   var(val, Eigen::VectorXd::LinSpaced(10,1,10)) // Set elements 1:10.
  /// <pre>
  /// Note: yes, vector indices in Matlab are doubles.
  template <typename T, typename... Types>
  LcmMatlabRemoteVariable subsasgn(T val, Types... args) const {
    LcmMatlabRemoteVariable s = AssembleSubstruct(args...);

    return LcmCallMatlabSingleOutput("subsasgn", *this, s, val);
  }

 private:
  // Helper methods for variadic template call in CallMatlab.
  inline void AssembleSubsPrepMsg(drake::lcmt_call_matlab* msg,
                                  int* index) const {
    // Intentionally left blank.  Base case for template recursion.
  }

  template <typename T, typename... Types>
  void AssembleSubsPrepMsg(drake::lcmt_call_matlab* msg, int* index, T first,
                           Types... args) const {
    const std::string dummy_field_name = "f" + std::to_string(*index);
    ToLcmMatlabArray(dummy_field_name, &(msg->rhs[(*index) * 2]));
    ToLcmMatlabArray(first, &(msg->rhs[(*index) * 2 + 1]));
    *index += 1;
    AssembleSubsPrepMsg(msg, index, args...);
  }

  template <typename... Types>
  LcmMatlabRemoteVariable AssembleSubstruct(Types... args) const {
    // construct a cell matrix (with one entry for each argument) using
    // e.g., struct2cell('f1',1:2,'f2','test'))
    LcmMatlabRemoteVariable temp_struct;
    {
      const int num_inputs = sizeof...(args);

      drake::lcmt_call_matlab msg;
      msg.nlhs = 1;
      msg.lhs.resize(1);
      msg.lhs[0] = temp_struct.unique_id_;

      int index = 0;
      msg.nrhs = 2 * num_inputs;
      msg.rhs.resize(2 * num_inputs);
      AssembleSubsPrepMsg(&msg, &index, args...);

      msg.function_name = "struct";
      internal::PublishLcmCallMatlab(msg);
    }
    LcmMatlabRemoteVariable temp_cell =
        LcmCallMatlabSingleOutput("struct2cell", temp_struct);

    // create the substruct
    return LcmCallMatlabSingleOutput("substruct", "()", temp_cell);
  }

 private:
  const int64_t unique_id_{};
};

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
    msg.lhs[i] = remote_vars[i].unique_id();
  }

  int index = 0;
  msg.nrhs = num_inputs;
  msg.rhs.resize(num_inputs);
  internal::AssembleLcmCallMatlabMsg(&msg, &index, args...);

  msg.function_name = function_name;
  internal::PublishLcmCallMatlab(msg);
  return remote_vars;
}

/// Special cases the call with zero outputs, since it's so common.
template <typename... Types>
void LcmCallMatlab(const std::string& function_name, Types... args) {
  LcmCallMatlab(0, function_name, args...);
}

/// Special cases the call with one output.
template <typename... Types>
LcmMatlabRemoteVariable LcmCallMatlabSingleOutput(
    const std::string& function_name, Types... args) {
  std::vector<LcmMatlabRemoteVariable> vars =
      LcmCallMatlab(1, function_name, args...);
  return vars[0];
}

/// Creates a new remote variable with the corresponding value set.
template <typename T>
LcmMatlabRemoteVariable LcmNewRemoteVariable(T value) {
  // It took some time to figure out how to make a simple "assign" call via
  // mexCallMatlab.  The `deal` method is the trick.
  return LcmCallMatlabSingleOutput("deal", value);
}

}  // namespace lcm
}  // namespace drake
