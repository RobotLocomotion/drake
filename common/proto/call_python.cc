#include "drake/common/proto/call_python.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/proto/rpc_pipe_temp_directory.h"

namespace drake {
namespace common {

static int py_globally_unique_id = 0;

PythonRemoteVariable::PythonRemoteVariable()
    : unique_id_(py_globally_unique_id++)
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

void ToMatlabArray(double var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::DOUBLE);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(double);
  matlab_array->set_data(&var, num_bytes);
}

void ToMatlabArray(int var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::INT);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int);
  matlab_array->set_data(&var, num_bytes);
}

void ToMatlabArray(const std::string& str, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::CHAR);
  matlab_array->set_shape_type(MatlabArray::VECTOR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(str.length());
  int num_bytes = sizeof(char) * str.length();
  matlab_array->set_data(str.data(), num_bytes);
}

void internal::ToMatlabArrayMatrix(
    const Eigen::Ref<const MatrixX<bool>>& mat,
    MatlabArray* matlab_array, bool is_vector) {
  matlab_array->set_type(MatlabArray::LOGICAL);
  matlab_array->set_shape_type(
      is_vector ? MatlabArray::VECTOR : MatlabArray::MATRIX);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void internal::ToMatlabArrayMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& mat,
    MatlabArray* matlab_array, bool is_vector) {
  matlab_array->set_type(MatlabArray::DOUBLE);
  matlab_array->set_shape_type(
      is_vector ? MatlabArray::VECTOR : MatlabArray::MATRIX);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void internal::ToMatlabArrayMatrix(
    const Eigen::Ref<const Eigen::MatrixXi>& mat,
    MatlabArray* matlab_array, bool is_vector) {
  matlab_array->set_type(MatlabArray::INT);
  matlab_array->set_shape_type(
      is_vector ? MatlabArray::VECTOR : MatlabArray::MATRIX);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(int) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

namespace {

auto GetPythonOutputStream(const std::string* filename = nullptr) {
  static std::unique_ptr<google::protobuf::io::FileOutputStream> raw_output;
  if (!raw_output) {
    // If we do not yet have a file, create it.
    const std::string filename_default
        = GetRpcPipeTempDirectory() + "/python_rpc";
    const std::string filename_actual = filename ? *filename : filename_default;
    // NOTE(russt): This code violates the style-guide by expecting the file to
    // be closed properly at program termination (these streams will be stored
    // as static globals).  Experimentally we found that the output file could
    // be corrupt unless we included *both* the Flush() call and the
    // SetCloseOnDelete() calls below.  Sadly, I cannot explain why.
    raw_output = std::make_unique<google::protobuf::io::FileOutputStream>(
        open(filename_actual.c_str(), O_WRONLY | O_CREAT, S_IRWXU));
    raw_output->SetCloseOnDelete(true);
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

void PublishCall(
    google::protobuf::io::FileOutputStream* praw_output,
    const MatlabRPC& message) {
  DRAKE_DEMAND(praw_output);
  auto& raw_output = *praw_output;

  {  // Defines the lifetime of the CodedOutputStream.
    google::protobuf::io::CodedOutputStream output(&raw_output);

    // Write the size.
    const int size = message.ByteSize();
    output.WriteVarint32(size);

    uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
    if (buffer != NULL) {
      // Optimization:  The message fits in one buffer, so use the faster
      // direct-to-array serialization path.
      message.SerializeWithCachedSizesToArray(buffer);
    } else {
      // Slightly-slower path when the message is multiple buffers.
      message.SerializeWithCachedSizes(&output);
      DRAKE_DEMAND(!output.HadError());
    }
  }

  raw_output.Flush();
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
