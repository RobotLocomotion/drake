#include "drake/common/proto/call_matlab.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace common {

static int globally_unique_id = 0;

MatlabRemoteVariable::MatlabRemoteVariable()
    : unique_id_(globally_unique_id++)
// TODO(russt): replace this with a random int64_t, e.g.
// http://stackoverflow.com/questions/7114043/random-number-generation-in-c11-how-to-generate-how-do-they-work
// TODO(russt): david-german-tri recommended a more robust (but more complex)
// solution was to use e.g. [IP address + process id + time].  We decided this
// was sufficient for now.
{}

void ToMatlabArray(const MatlabRemoteVariable& var, MatlabArray* matlab_array) {
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

void ToMatlabArray(int var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::INT);
  matlab_array->set_shape_type(MatlabArray::SCALAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int);
  matlab_array->set_data(&var, num_bytes);
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

void ToMatlabArray(const std::string& str, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::CHAR);
  matlab_array->set_shape_type(MatlabArray::VECTOR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(str.length());
  int num_bytes = sizeof(char) * str.length();
  matlab_array->set_data(str.data(), num_bytes);
}

namespace internal {

std::unique_ptr<google::protobuf::io::FileOutputStream>
CreateOutputStream(const std::string& filename) {
  // NOTE(russt): This code violates the style-guide by expecting the file to be
  // closed properly at program termination (these streams will be stored as
  // static globals).  Experimentally we found that the output file could be
  // corrupt unless we included *both* the Flush() call and the
  // SetCloseOnDelete() calls below.  Sadly, I cannot explain why.
  auto raw_output =
      std::make_unique<google::protobuf::io::FileOutputStream>(
          open(filename.c_str(), O_WRONLY | O_CREAT, S_IRWXU));
  raw_output->SetCloseOnDelete(true);
  return raw_output;
}

void PublishCallMatlab(const MatlabRPC& message) {
  // TODO(russt): Provide option for setting the filename.
  static auto raw_output = CreateOutputStream("/tmp/matlab_rpc");
  PublishCall(raw_output.get(), message);
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

}  // namespace internal
}  // namespace common
}  // namespace drake
