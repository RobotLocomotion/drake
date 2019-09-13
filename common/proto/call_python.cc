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

void ToPythonRemoteData(const PythonRemoteVariable& variable,
                        PythonRemoteData* data) {
  data->set_type(PythonRemoteData::REMOTE_VARIABLE_REFERENCE);
  data->set_shape_type(PythonRemoteData::SCALAR);
  data->set_rows(1);
  data->set_cols(1);
  int num_bytes = sizeof(int64_t);
  int64_t uid = variable.unique_id();
  data->set_data(&uid, num_bytes);
}

void ToPythonRemoteData(double scalar, PythonRemoteData* data) {
  data->set_type(PythonRemoteData::DOUBLE);
  data->set_shape_type(PythonRemoteData::SCALAR);
  data->set_rows(1);
  data->set_cols(1);
  int num_bytes = sizeof(double);
  data->set_data(&scalar, num_bytes);
}

void ToPythonRemoteData(int scalar, PythonRemoteData* data) {
  data->set_type(PythonRemoteData::INT);
  data->set_shape_type(PythonRemoteData::SCALAR);
  data->set_rows(1);
  data->set_cols(1);
  int num_bytes = sizeof(int);
  data->set_data(&scalar, num_bytes);
}

void ToPythonRemoteData(const std::string& str, PythonRemoteData* data) {
  data->set_type(PythonRemoteData::CHAR);
  data->set_shape_type(PythonRemoteData::VECTOR);
  data->set_rows(1);
  data->set_cols(str.length());
  int num_bytes = sizeof(char) * str.length();
  data->set_data(str.data(), num_bytes);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(const PythonRemoteVariable& variable,
                   PythonRemoteData* data) {
  ToPythonRemoteData(variable, data);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(double scalar, PythonRemoteData* data) {
  ToPythonRemoteData(scalar, data);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(int scalar, PythonRemoteData* data) {
  ToPythonRemoteData(scalar, data);
}

// DRAKE_DEPRECATED 2019-12-01
void ToMatlabArray(const std::string& str, PythonRemoteData* data) {
  ToPythonRemoteData(str, data);
}

namespace internal {

void ToPythonRemoteDataMatrix(const Eigen::Ref<const MatrixX<bool>>& mat,
                              PythonRemoteData* data, bool is_vector) {
  data->set_type(PythonRemoteData::LOGICAL);
  data->set_shape_type(
      is_vector ? PythonRemoteData::VECTOR : PythonRemoteData::MATRIX);
  data->set_rows(mat.rows());
  data->set_cols(mat.cols());
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  data->set_data(mat.data(), num_bytes);
}

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                              PythonRemoteData* data, bool is_vector) {
  data->set_type(PythonRemoteData::DOUBLE);
  data->set_shape_type(
      is_vector ? PythonRemoteData::VECTOR : PythonRemoteData::MATRIX);
  data->set_rows(mat.rows());
  data->set_cols(mat.cols());
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  data->set_data(mat.data(), num_bytes);
}

void ToPythonRemoteDataMatrix(
    const Eigen::Ref<const Eigen::MatrixXi>& mat,
    PythonRemoteData* data, bool is_vector) {
  data->set_type(PythonRemoteData::INT);
  data->set_shape_type(
      is_vector ? PythonRemoteData::VECTOR : PythonRemoteData::MATRIX);
  data->set_rows(mat.rows());
  data->set_cols(mat.cols());
  int num_bytes = sizeof(int) * mat.rows() * mat.cols();
  data->set_data(mat.data(), num_bytes);
}

}  // namespace internal

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
    const PythonRemoteMessage& message) {
  DRAKE_DEMAND(praw_output);
  auto& raw_output = *praw_output;

  {  // Defines the lifetime of the CodedOutputStream.
    google::protobuf::io::CodedOutputStream output(&raw_output);

    // Write the size.
    const int size = message.ByteSize();
    output.WriteVarint32(size);

    uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
    if (buffer != nullptr) {
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

void internal::PublishCallPython(const PythonRemoteMessage& message) {
  PublishCall(GetPythonOutputStream(), message);
}

}  // namespace common
}  // namespace drake
