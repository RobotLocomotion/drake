#include "drake/common/proto/call_python.h"

#include "drake/common/proto/matlab_rpc.pb.h"

namespace drake {
namespace common {

static int py_globally_unique_id = 0;

PythonRemoteVariable::PythonRemoteVariable()
    : unique_id_(py_globally_unique_id++)
// TODO(eric.cousineau): Make this consistent with what `call_matlab` has when
// it changes.
{}

void ToMatlabArray(const PythonRemoteVariable& var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::REMOTE_VARIABLE_REFERENCE);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int64_t);
  int64_t uid = var.unique_id();
  matlab_array->set_data(&uid, num_bytes);
}

void internal::PublishCallPython(const MatlabRPC& message) {
  // TODO(eric.cousineau): Provide option for setting the filename.
  static auto raw_output = CreateOutputStream("/tmp/python_rpc");
  PublishCall(raw_output.get(), message);
}

}  // namespace common
}  // namespace drake
