#include "drake/common/proto/call_python.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>

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

void ToPythonRemoteData(const PythonRemoteVariable& variable,
                        PythonRemoteData* data) {
  data->type = PythonRemoteData::REMOTE_VARIABLE_REFERENCE;
  data->shape_type = PythonRemoteData::SCALAR;
  data->rows = 1;
  data->cols = 1;
  int num_bytes = sizeof(int64_t);
  int64_t uid = variable.unique_id();
  data->set_data(&uid, num_bytes);
}

void ToPythonRemoteData(double scalar, PythonRemoteData* data) {
  data->type = PythonRemoteData::DOUBLE;
  data->shape_type = PythonRemoteData::SCALAR;
  data->rows = 1;
  data->cols = 1;
  int num_bytes = sizeof(double);
  data->set_data(&scalar, num_bytes);
}

void ToPythonRemoteData(int scalar, PythonRemoteData* data) {
  data->type = PythonRemoteData::INT;
  data->shape_type = PythonRemoteData::SCALAR;
  data->rows = 1;
  data->cols = 1;
  int num_bytes = sizeof(int);
  data->set_data(&scalar, num_bytes);
}

void ToPythonRemoteData(const std::string& str, PythonRemoteData* data) {
  data->type = PythonRemoteData::CHAR;
  data->shape_type = PythonRemoteData::VECTOR;
  data->rows = 1;
  data->cols = str.length();
  int num_bytes = sizeof(char) * str.length();
  data->set_data(str.data(), num_bytes);
}

}  // namespace internal

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(const PythonRemoteVariable& variable,
                   internal::PythonRemoteData* data) {
  ToPythonRemoteData(variable, data);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(double scalar, internal::PythonRemoteData* data) {
  ToPythonRemoteData(scalar, data);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(int scalar, internal::PythonRemoteData* data) {
  ToPythonRemoteData(scalar, data);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(const std::string& str, internal::PythonRemoteData* data) {
  ToPythonRemoteData(str, data);
}

namespace internal {

void ToPythonRemoteDataMatrix(const Eigen::Ref<const MatrixX<bool>>& mat,
                              PythonRemoteData* data, bool is_vector) {
  data->type = PythonRemoteData::LOGICAL;
  data->shape_type =
      is_vector ? PythonRemoteData::VECTOR : PythonRemoteData::MATRIX;
  data->rows = mat.rows();
  data->cols = mat.cols();
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  data->set_data(mat.data(), num_bytes);
}

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                              PythonRemoteData* data, bool is_vector) {
  data->type = PythonRemoteData::DOUBLE;
  data->shape_type =
      is_vector ? PythonRemoteData::VECTOR : PythonRemoteData::MATRIX;
  data->rows = mat.rows();
  data->cols = mat.cols();
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  data->set_data(mat.data(), num_bytes);
}

void ToPythonRemoteDataMatrix(
    const Eigen::Ref<const Eigen::MatrixXi>& mat,
    PythonRemoteData* data, bool is_vector) {
  data->type = PythonRemoteData::INT;
  data->shape_type =
      is_vector ? PythonRemoteData::VECTOR : PythonRemoteData::MATRIX;
  data->rows = mat.rows();
  data->cols = mat.cols();
  int num_bytes = sizeof(int) * mat.rows() * mat.cols();
  data->set_data(mat.data(), num_bytes);
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
    const internal::PythonRemoteMessage& message) {
  DRAKE_DEMAND(output_arg != nullptr);
  std::ofstream& output = *output_arg;

  std::vector<uint8_t> encoded = message.ToMsgpack();
  output << encoded.size();
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

void internal::PublishCallPython(const PythonRemoteMessage& message) {
  static const never_destroyed<std::ofstream*> output{InitOutput(std::nullopt)};
  PublishCall(output.access(), message);
}

}  // namespace common
}  // namespace drake
