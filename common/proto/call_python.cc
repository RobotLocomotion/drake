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

void copy_data(const void* data, const int size,
               lcmt_call_python_data* message) {
  message->num_bytes = size;
  message->data.resize(size);
  std::memcpy(message->data.data(), data, size);
}

void ToPythonRemoteData(const PythonRemoteVariable& variable,
                        lcmt_call_python_data* data) {
  data->data_type = lcmt_call_python_data::REMOTE_VARIABLE_REFERENCE;
  data->shape_type = lcmt_call_python_data::SCALAR;
  data->rows = 1;
  data->cols = 1;
  int num_bytes = sizeof(int64_t);
  int64_t uid = variable.unique_id();
  copy_data(&uid, num_bytes, data);
}

void ToPythonRemoteData(double scalar, lcmt_call_python_data* data) {
  data->data_type = lcmt_call_python_data::DOUBLE;
  data->shape_type = lcmt_call_python_data::SCALAR;
  data->rows = 1;
  data->cols = 1;
  int num_bytes = sizeof(double);
  copy_data(&scalar, num_bytes, data);
}

void ToPythonRemoteData(int scalar, lcmt_call_python_data* data) {
  data->data_type = lcmt_call_python_data::INT;
  data->shape_type = lcmt_call_python_data::SCALAR;
  data->rows = 1;
  data->cols = 1;
  int num_bytes = sizeof(int);
  copy_data(&scalar, num_bytes, data);
}

void ToPythonRemoteData(const std::string& str, lcmt_call_python_data* data) {
  data->data_type = lcmt_call_python_data::CHAR;
  data->shape_type = lcmt_call_python_data::VECTOR;
  data->rows = 1;
  data->cols = str.length();
  int num_bytes = sizeof(char) * str.length();
  copy_data(str.data(), num_bytes, data);
}

void ToPythonRemoteDataMatrix(const Eigen::Ref<const MatrixX<bool>>& mat,
                              lcmt_call_python_data* data, bool is_vector) {
  data->data_type = lcmt_call_python_data::LOGICAL;
  data->shape_type =
      is_vector ? lcmt_call_python_data::VECTOR : lcmt_call_python_data::MATRIX;
  data->rows = mat.rows();
  data->cols = mat.cols();
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  copy_data(mat.data(), num_bytes, data);
}

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                              lcmt_call_python_data* data, bool is_vector) {
  data->data_type = lcmt_call_python_data::DOUBLE;
  data->shape_type =
      is_vector ? lcmt_call_python_data::VECTOR : lcmt_call_python_data::MATRIX;
  data->rows = mat.rows();
  data->cols = mat.cols();
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  copy_data(mat.data(), num_bytes, data);
}

void ToPythonRemoteDataMatrix(
    const Eigen::Ref<const Eigen::MatrixXi>& mat,
    lcmt_call_python_data* data, bool is_vector) {
  data->data_type = lcmt_call_python_data::INT;
  data->shape_type =
      is_vector ? lcmt_call_python_data::VECTOR : lcmt_call_python_data::MATRIX;
  data->rows = mat.rows();
  data->cols = mat.cols();
  int num_bytes = sizeof(int) * mat.rows() * mat.cols();
  copy_data(mat.data(), num_bytes, data);
}

}  // namespace internal

namespace {

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

void PublishCall(
    std::ofstream* output_arg,
    const lcmt_call_python& message) {
  DRAKE_DEMAND(output_arg != nullptr);
  std::ofstream& output = *output_arg;

  const int num_bytes = message.getEncodedSize();
  DRAKE_DEMAND(num_bytes >= 0);
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> encoded(size_bytes);
  message.encode(encoded.data(), 0, num_bytes);

  output << size_bytes;
  output << '\0';
  const void* const data = encoded.data();
  output.write(static_cast<const char*>(data), encoded.size());
  output << '\0';

  output.flush();
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
