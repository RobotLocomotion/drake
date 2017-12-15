#include "drake/common/proto/call_python.h"

#include <memory>

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

namespace {

const char kFilenameDefault[] = "/tmp/python_rpc";

auto GetPythonOutputStream(const std::string* filename = nullptr) {
  static std::unique_ptr<google::protobuf::io::FileOutputStream> raw_output;
  if (!raw_output) {
    // If we do not yet have a file, create it.
    raw_output =
        internal::CreateOutputStream(filename ? *filename : kFilenameDefault);
  } else {
    // If we already have a file, ensure that this does not come from
    // `CallPythonInit`.
    if (filename) {
      throw std::runtime_error(
          "`CallPython` or `CallPythonInit` has already been called");
    }
  }
  return raw_output.get();
}

}  // namespace

void CallPythonInit(const std::string& filename) {
  GetPythonOutputStream(&filename);
}

void internal::PublishCallPython(const MatlabRPC& message) {
  PublishCall(GetPythonOutputStream(), message);
}

}  // namespace common
}  // namespace drake
