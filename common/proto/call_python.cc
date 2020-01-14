#include "drake/common/proto/call_python.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/proto/rpc_pipe_temp_directory.h"

namespace drake {
namespace common {

static int py_globally_unique_id = 0;

PythonRemoteVariable::PythonRemoteVariable()
    : unique_id_(py_globally_unique_id++)
{}

namespace internal {

// Copies `size` bytes at `data` into `message->data`.
void CopyBytes(const void* bytes, const int size,
               lcmt_call_python_data* message) {
  message->num_bytes = size;
  message->data.resize(size);
  std::memcpy(message->data.data(), bytes, size);
}

void ToPythonRemoteData(const PythonRemoteVariable& variable,
                        lcmt_call_python_data* message) {
  message->data_type = lcmt_call_python_data::REMOTE_VARIABLE_REFERENCE;
  message->shape_type = lcmt_call_python_data::SCALAR;
  message->rows = 1;
  message->cols = 1;
  int64_t uid = variable.unique_id();
  CopyBytes(&uid, sizeof(uid), message);
}

void ToPythonRemoteData(double scalar, lcmt_call_python_data* message) {
  message->data_type = lcmt_call_python_data::DOUBLE;
  message->shape_type = lcmt_call_python_data::SCALAR;
  message->rows = 1;
  message->cols = 1;
  CopyBytes(&scalar, sizeof(scalar), message);
}

void ToPythonRemoteData(int scalar, lcmt_call_python_data* message) {
  message->data_type = lcmt_call_python_data::INT;
  message->shape_type = lcmt_call_python_data::SCALAR;
  message->rows = 1;
  message->cols = 1;
  CopyBytes(&scalar, sizeof(scalar), message);
}

void ToPythonRemoteData(const std::string& str,
                        lcmt_call_python_data* message) {
  message->data_type = lcmt_call_python_data::CHAR;
  message->shape_type = lcmt_call_python_data::VECTOR;
  message->rows = 1;
  message->cols = str.length();
  CopyBytes(str.data(), str.size(), message);
}

void ToPythonRemoteDataMatrix(const Eigen::Ref<const MatrixX<bool>>& mat,
                              lcmt_call_python_data* message, bool is_vector) {
  message->data_type = lcmt_call_python_data::LOGICAL;
  message->shape_type =
      is_vector ? lcmt_call_python_data::VECTOR : lcmt_call_python_data::MATRIX;
  message->rows = mat.rows();
  message->cols = mat.cols();
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  CopyBytes(mat.data(), num_bytes, message);
}

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                              lcmt_call_python_data* message, bool is_vector) {
  message->data_type = lcmt_call_python_data::DOUBLE;
  message->shape_type =
      is_vector ? lcmt_call_python_data::VECTOR : lcmt_call_python_data::MATRIX;
  message->rows = mat.rows();
  message->cols = mat.cols();
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  CopyBytes(mat.data(), num_bytes, message);
}

void ToPythonRemoteDataMatrix(
    const Eigen::Ref<const Eigen::MatrixXi>& mat,
    lcmt_call_python_data* message, bool is_vector) {
  message->data_type = lcmt_call_python_data::INT;
  message->shape_type =
      is_vector ? lcmt_call_python_data::VECTOR : lcmt_call_python_data::MATRIX;
  message->rows = mat.rows();
  message->cols = mat.cols();
  int num_bytes = sizeof(int) * mat.rows() * mat.cols();
  CopyBytes(mat.data(), num_bytes, message);
}

}  // namespace internal

namespace {

// Latch-initialize the ofstream output that writes to the python client.
// The return value is a long-lived pointer to a singleton.
std::ofstream* InitOutput(const std::optional<std::string>& filename) {
  static never_destroyed<std::unique_ptr<std::ofstream>> raw_output;
  if (!raw_output.access()) {
    // If we do not yet have a file, create it.
    const std::string filename_default
        = GetRpcPipeTempDirectory() + "/python_rpc";
    const std::string filename_actual = filename ? *filename : filename_default;
    raw_output.access() = std::make_unique<std::ofstream>(filename_actual);
  } else {
    // If we already have a file, ensure that this does not come from
    // `CallPythonInit`.
    if (filename) {
      throw std::runtime_error(
          "`CallPython` or `CallPythonInit` has already been called");
    }
  }
  return raw_output.access().get();
}

void PublishCall(std::ofstream* stream_arg, const lcmt_call_python& message) {
  DRAKE_DEMAND(stream_arg != nullptr);
  std::ofstream& stream = *stream_arg;

  const int num_bytes = message.getEncodedSize();
  DRAKE_DEMAND(num_bytes >= 0);
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> encoded(size_bytes);
  message.encode(encoded.data(), 0, num_bytes);

  stream << size_bytes;
  stream << '\0';
  const void* const data = encoded.data();
  stream.write(static_cast<const char*>(data), encoded.size());
  stream << '\0';

  stream.flush();
}

}  // namespace

void CallPythonInit(const std::string& filename) {
  InitOutput(filename);
}

void internal::PublishCallPython(const lcmt_call_python& message) {
  static const never_destroyed<std::ofstream*> output{InitOutput(std::nullopt)};
  PublishCall(output.access(), message);
}

}  // namespace common
}  // namespace drake
