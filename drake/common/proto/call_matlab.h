#pragma once

#include <memory>
#include <string>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

#include "drake/common/eigen_types.h"
#include "drake/common/proto/matlab_rpc.pb.h"

/// @file
/// @brief Utilities for calling Matlab from C++
///
/// Provides a simple interface for (one-directional) RPC to a simple matlab
/// remote client.  Methods are provided to serialize our favorite data types
/// into protobuf and then published to a file.  The interface is modeled
/// after mexCallMATLAB
///   https://www.mathworks.com/help/matlab/apiref/mexcallmatlab.html
/// but we use C++11 to provide a much nicer interface.
///
/// To play the remote calls in matlab, simply run call_matlab_client from your
/// matlab terminal.  For synchronous playback, use a named pipe by running
///    `mkfifo /tmp/matlab_rpc`
/// in a bash terminal.
///
/// The primary use case that this was designed for was to make MATLAB plotting
/// available in C++ without requiring the C++ code to link against MATLAB in
/// any way... (if MATLAB is not present, the messages simply fall on deaf
/// ears).
///
/// Support for multi-function commands is provided by allowing return values to
/// be stored on the remote client, and reused by a simple "remote variable
/// reference" that is kept by the publisher.
///
/// See call_matlab_test.cc for some simple examples.

namespace drake {
namespace common {

/// Serialize our favorite data types into the matlab_array structure.
/// To support a calling call_matlab for a new data type, simply implement
///  another one of these methods.

class MatlabRemoteVariable;

namespace internal {

void ToMatlabArrayMatrix(
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>>&
        mat,
    MatlabArray* matlab_array, bool is_vector);

void ToMatlabArrayMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                         MatlabArray* matlab_array, bool is_vector);

void ToMatlabArrayMatrix(const Eigen::Ref<const Eigen::MatrixXi>& mat,
                         MatlabArray* matlab_array, bool is_vector);

}  // namespace internal

void ToMatlabArray(const MatlabRemoteVariable& var, MatlabArray* matlab_array);

void ToMatlabArray(double scalar, MatlabArray* matlab_array);

void ToMatlabArray(int scalar, MatlabArray* matlab_array);

void ToMatlabArray(const std::string& str, MatlabArray* matlab_array);

template <typename Derived>
void ToMatlabArray(const Eigen::MatrixBase<Derived>& mat,
                   MatlabArray* matlab_array) {
  const bool is_vector = (Derived::ColsAtCompileTime == 1);
  return internal::ToMatlabArrayMatrix(mat, matlab_array, is_vector);
}

// Helper methods for variadic template call in CallMatlab.
namespace internal {
inline void AssembleCallMatlabMsg(MatlabRPC*) {
  // Intentionally left blank.  Base case for template recursion.
}

template <typename T, typename... Types>
void AssembleCallMatlabMsg(MatlabRPC* msg, T first, Types... args) {
  ToMatlabArray(first, msg->add_rhs());
  AssembleCallMatlabMsg(msg, args...);
}

std::unique_ptr<google::protobuf::io::FileOutputStream>
CreateOutputStream(const std::string& filename);

void PublishCall(
    google::protobuf::io::FileOutputStream* praw_output,
    const MatlabRPC& message);

// Simple wrapper to prevent the outside world from needing to worry about
// creating a the i/o stream object.
// TODO(russt): support setting the pipe name (via a
// CallMatlabChannelPush/Pop).
void PublishCallMatlab(const MatlabRPC& msg);

}  // namespace internal

// forward declaration:
template <typename... Types>
MatlabRemoteVariable CallMatlabSingleOutput(const std::string& function_name,
                                            Types... args);

/// Holds a reference to a variable stored on the matlab client, which can be
/// passed back into a future lcm_call_matlab call.
class MatlabRemoteVariable {
 public:
  MatlabRemoteVariable();
  //  ~MatlabRemoteVariable(); // TODO(russt): send a destroy message on
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
  MatlabRemoteVariable operator()(Types... args) const {
    MatlabRemoteVariable s = AssembleSubstruct(args...);

    return CallMatlabSingleOutput("subsref", *this, s);
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
  MatlabRemoteVariable subsasgn(T val, Types... args) const {
    MatlabRemoteVariable s = AssembleSubstruct(args...);

    return CallMatlabSingleOutput("subsasgn", *this, s, val);
  }

 private:
  // Helper methods for variadic template call in CallMatlab.
  inline void AssembleSubsPrepMsg(MatlabRPC*) const {
    // Intentionally left blank.  Base case for template recursion.
  }

  template <typename T, typename... Types>
  void AssembleSubsPrepMsg(MatlabRPC* msg, T first, Types... args) const {
    const std::string dummy_field_name =
        "f" + std::to_string(msg->rhs_size() / 2 + 1);
    ToMatlabArray(dummy_field_name, msg->add_rhs());
    ToMatlabArray(first, msg->add_rhs());
    AssembleSubsPrepMsg(msg, args...);
  }

  template <typename... Types>
  MatlabRemoteVariable AssembleSubstruct(Types... args) const {
    // construct a cell matrix (with one entry for each argument) using
    // e.g., struct2cell(struct('f1',1:2,'f2','test'))
    MatlabRemoteVariable temp_struct;
    {
      MatlabRPC msg;
      msg.add_lhs(temp_struct.unique_id());

      AssembleSubsPrepMsg(&msg, args...);

      msg.set_function_name("struct");
      internal::PublishCallMatlab(msg);
    }
    MatlabRemoteVariable temp_cell =
        CallMatlabSingleOutput("struct2cell", temp_struct);

    // create the substruct
    return CallMatlabSingleOutput("substruct", "()", temp_cell);
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
/// @param argument1 Any data type which has a ToMatlabArray method
/// implemented.
/// @param argument2 Same as above.
/// ...
///
/// See call_matlab_test.cc for some simple examples.
template <typename... Types>
std::vector<MatlabRemoteVariable> CallMatlab(int num_outputs,
                                             const std::string& function_name,
                                             Types... args) {
  if (num_outputs < 0) num_outputs = 0;
  std::vector<MatlabRemoteVariable> remote_vars(num_outputs);

  MatlabRPC msg;
  for (int i = 0; i < num_outputs; i++) {
    msg.add_lhs(remote_vars[i].unique_id());
  }

  internal::AssembleCallMatlabMsg(&msg, args...);

  msg.set_function_name(function_name);
  internal::PublishCallMatlab(msg);
  return remote_vars;
}

/// Special cases the call with zero outputs, since it's so common.
template <typename... Types>
void CallMatlab(const std::string& function_name, Types... args) {
  CallMatlab(0, function_name, args...);
}

/// Special cases the call with one output.
template <typename... Types>
MatlabRemoteVariable CallMatlabSingleOutput(const std::string& function_name,
                                            Types... args) {
  std::vector<MatlabRemoteVariable> vars =
      CallMatlab(1, function_name, args...);
  return vars[0];
}

/// Creates a new remote variable with the corresponding value set.
template <typename T>
MatlabRemoteVariable NewRemoteVariable(T value) {
  // It took some time to figure out how to make a simple "assign" call via
  // mexCallMatlab.  The `deal` method is the trick.
  return CallMatlabSingleOutput("deal", value);
}

}  // namespace common
}  // namespace drake
