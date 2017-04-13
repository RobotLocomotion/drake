#include "drake/common/call_matlab.h"

#include <cstring>
#include <fstream>
#include <limits>
#include <string>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

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
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(int64_t);
  int64_t uid = var.unique_id();
  matlab_array->set_data(&uid, num_bytes);
}

void ToMatlabArray(double var, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::DOUBLE);
  matlab_array->set_rows(1);
  matlab_array->set_cols(1);
  int num_bytes = sizeof(double);
  matlab_array->set_data(&var, num_bytes);
}

void ToMatlabArray(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                   MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::DOUBLE);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(double) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void ToMatlabArray(
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>>&
        mat,
    MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::LOGICAL);
  matlab_array->set_rows(mat.rows());
  matlab_array->set_cols(mat.cols());
  int num_bytes = sizeof(bool) * mat.rows() * mat.cols();
  matlab_array->set_data(mat.data(), num_bytes);
}

void ToMatlabArray(const std::string& str, MatlabArray* matlab_array) {
  matlab_array->set_type(MatlabArray::CHAR);
  matlab_array->set_rows(1);
  matlab_array->set_cols(str.length());
  int num_bytes = sizeof(char) * str.length();
  matlab_array->set_data(str.data(), num_bytes);
}

void internal::PublishCallMatlab(const MatlabRPC& message) {
  // TODO(russt): Provide option for setting the filename.
  static google::protobuf::io::FileOutputStream raw_output(
      open("/tmp/matlab_rpc", O_WRONLY | O_CREAT, S_IRWXU));

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

}  // namespace common
}  // namespace drake
