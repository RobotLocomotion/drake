#include <cstring>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <mex.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

#include "drake/common/drake_assert.h"
#include "drake/common/proto/matlab_rpc.pb.h"
#include "drake/matlab/util/mex_util.h"

// Mex client for matlab remote procedure calls (RPCs).

// Known issues: Some functions in matlab do not like to get called from mex.
// For instance, calling "histogram" fails with "Error using inputname...
// Argument number is not valid." because it is trying to extract a variable
// name from the matlab calling stack (and there are none).  Bad matlab.

// TODO(russt): Implement an interface that allows the remote publisher to
// manually delete a client variable.

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  // TODO(russt): Take filename as an optional input.
  const std::string filename = "/tmp/matlab_rpc";
  google::protobuf::io::FileInputStream raw_input(
      open(filename.c_str(), O_RDONLY));
  google::protobuf::io::CodedInputStream input(&raw_input);

  // TODO(russt): Document use of mkfifo?
  mexPrintf("Reading messages from %s ...\n", filename.c_str());

  std::map<int64_t, mxArray*> client_vars_;

  // Read the size.
  uint32_t size;
  while (input.ReadVarint32(&size)) {
    // Tell the stream not to read beyond that size.
    auto limit = input.PushLimit(size);

    // Parse the message.
    drake::common::MatlabRPC message;
    if (!message.MergePartialFromCodedStream(&input)) {
      mexErrMsgTxt("Failed to read from coded stream");
    }
    if (!input.ConsumedEntireMessage()) {
      mexErrMsgTxt("Failed to consume entire message");
    }

    // Release the limit.
    input.PopLimit(limit);

    int i;
    std::vector<mxArray*> lhs(message.lhs_size()), rhs(message.rhs_size());

    // Create the input arguments
    for (i = 0; i < message.rhs_size(); i++) {
      int num_bytes = message.rhs(i).data().size();
      switch (message.rhs(i).type()) {
        case drake::common::MatlabArray::REMOTE_VARIABLE_REFERENCE: {
          int64_t id;
          DRAKE_DEMAND(static_cast<int>(sizeof(int64_t)) == num_bytes);
          memcpy(&id, message.rhs(i).data().data(), num_bytes);
          if (client_vars_.find(id) == client_vars_.end()) {
            mexWarnMsgTxt(
                "rhs referenced unknown local variable.  dropping message.");
            for (int j = 0; j < i; j++) {
              mxDestroyArray(rhs[j]);
            }
            return;
          }
          rhs[i] = client_vars_.at(id);
          break;
        }
        case drake::common::MatlabArray::DOUBLE: {
          rhs[i] = mxCreateDoubleMatrix(message.rhs(i).rows(),
                                        message.rhs(i).cols(), mxREAL);
          DRAKE_DEMAND(static_cast<int>(sizeof(double)) *
                           message.rhs(i).rows() * message.rhs(i).cols() ==
                       num_bytes);
          memcpy(mxGetPr(rhs[i]), message.rhs(i).data().data(), num_bytes);
          break;
        }
        case drake::common::MatlabArray::CHAR: {
          mwSize dims[2];
          dims[0] = message.rhs(i).rows();
          dims[1] = message.rhs(i).cols();
          rhs[i] = mxCreateCharArray(2, dims);
          auto* char_data = static_cast<mxChar*>(mxGetData(rhs[i]));
          DRAKE_DEMAND(message.rhs(i).rows() * message.rhs(i).cols() ==
                       num_bytes);
          const std::string& str = message.rhs(i).data();
          for (int j = 0; j < num_bytes; j++) {  // Note: sizeof(mxChar) == 2.
            char_data[j] = static_cast<mxChar>(str[j]);
          }
          break;
        }
        case drake::common::MatlabArray::LOGICAL: {
          rhs[i] = mxCreateLogicalMatrix(message.rhs(i).rows(),
                                         message.rhs(i).cols());
          DRAKE_DEMAND(message.rhs(i).rows() * message.rhs(i).cols() ==
                       num_bytes);
          auto* logical_data = static_cast<mxLogical*>(mxGetData(rhs[i]));
          const std::string& str = message.rhs(i).data();
          for (int j = 0; j < num_bytes; j++) {
            logical_data[j] = static_cast<mxLogical>(str[j]);
          }
          break;
        }
        default:
          mexWarnMsgTxt("Unknown rhs variable type.");
          for (int j = 0; j < i; j++) {
            mxDestroyArray(rhs[j]);
          }
          return;
      }
    }

    // Make the actual call to MATLAB.
    bool trapped_error = mexCallMATLABsafe(
        static_cast<int>(lhs.size()), lhs.data(), static_cast<int>(rhs.size()),
        rhs.data(), message.function_name().c_str());

    if (!trapped_error) {
      // Assign any local variables that were returned by this call.
      for (i = 0; i < message.lhs_size(); i++) {
        // Zap any old variables that will be overwritten by this call.
        if (client_vars_.find(message.lhs(i)) != client_vars_.end()) {
          mxDestroyArray(client_vars_[message.lhs(i)]);
        }
        client_vars_[message.lhs(i)] = lhs[i];
      }
    }

    // Clean up the input argument data.
    for (i = 0; i < static_cast<int>(rhs.size()); i++) {
      // Make sure it's not a client var.
      if (message.rhs(i).type() !=
          drake::common::MatlabArray::REMOTE_VARIABLE_REFERENCE) {
        mxDestroyArray(rhs[i]);
      }
    }

    if (trapped_error) {
      break;
    }
  }

  // Clean up remaining memory.
  for (const auto& iter : client_vars_) {
    mxDestroyArray(iter.second);
  }
}
