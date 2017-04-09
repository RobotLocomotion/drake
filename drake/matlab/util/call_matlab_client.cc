#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <mex.h>

#include "drake/common/drake_assert.h"
#include "drake/common/matlab_rpc.pb.h"
#include "drake/matlab/util/drakeMexUtil.h"

// Mex client for matlab remote procedure calls (RPCs).

// Known issues: Some functions in matlab do not like to get called from mex.
// For instance, calling "histogram" fails with "Error using inputname...
// Argument number is not valid." because it is trying to extract a variable
// name from the matlab calling stack (and there are none).  Bad matlab.

// TODO(russt): Implement an interface that allows the remote publisher to
// manually delete a client variable.

/*
namespace {

  void FlushMatlabEventBuffer(void) {
    // An ugly hack.  drawnow used to work, but the internet confirms that it
    // stopped working around R2015.
    mxArray * time = mxCreateDoubleScalar(0.001);
    mexCallMATLABsafe(0, nullptr, 1, &time, "pause");
    mxDestroyArray(time);
  }

}  // end namespace
*/

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  // TODO(russt): Take filename as an optional input.
  std::string filename = "/tmp/matlab_rpc";

  // TODO(russt): Document use of mkfifo?
  mexPrintf("Listening for messages on %s...\n", filename.c_str());

//  FlushMatlabEventBuffer();

  std::ifstream input(filename);
  std::map<int64_t, mxArray*> client_vars_;
  drake::common::MatlabRPC msg;
  while (msg.ParseFromIstream(&input)) {
/*
    int i;
    std::vector<mxArray *> lhs(msg.lhs_size()), rhs(msg.rhs_size());

    // Create the input arguments
    for (i = 0; i < msg.rhs_size(); i++) {
      int num_bytes = msg.rhs(i).data().size();
//      if (num_bytes == 0) {
        mexPrintf("rhs %d seems to have zero bytes.  dropping message %s.\n",
                  i, msg.function_name().c_str());
        mexPrintf("type: %d\n", msg.rhs(i).type());
        mexPrintf("rows: %d\n", msg.rhs(i).rows());
        mexPrintf("cols: %d\n", msg.rhs(i).cols());
        for (int j = 0; j < i; j++) {
          mxDestroyArray(rhs[j]);
        }
        return;
//      }

      switch (msg.rhs(i).type()) {
        case drake::common::MatlabArray::REMOTE_VARIABLE_REFERENCE: {
          int64_t id;
          DRAKE_DEMAND(static_cast<int>(sizeof(int64_t)) == num_bytes);
          memcpy(&id, msg.rhs(i).data().data(), num_bytes);
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
          rhs[i] = mxCreateDoubleMatrix(msg.rhs(i).rows(), msg.rhs(i).cols(),
                                        mxREAL);
          DRAKE_DEMAND(static_cast<int>(sizeof(double)) * msg.rhs(i).rows() *
                       msg.rhs(i).cols() ==
                       num_bytes);
          memcpy(mxGetPr(rhs[i]), msg.rhs(i).data().data(),
                 num_bytes);
          break;
        }
        case drake::common::MatlabArray::CHAR: {
          mwSize dims[2];
          dims[0] = msg.rhs(i).rows();
          dims[1] = msg.rhs(i).cols();
          rhs[i] = mxCreateCharArray(2, dims);
          mxChar *char_data = static_cast<mxChar *>(mxGetData(rhs[i]));
          DRAKE_DEMAND(msg.rhs(i).rows() * msg.rhs(i).cols() == num_bytes);
          const std::string &str = msg.rhs(i).data();
          for (int j = 0; j < num_bytes; j++) {  // Note: sizeof(mxChar) == 2.
            char_data[j] = static_cast<mxChar>(str[j]);
          }
          break;
        }
        case drake::common::MatlabArray::LOGICAL: {
          rhs[i] = mxCreateLogicalMatrix(msg.rhs(i).rows(), msg.rhs(i).cols());
          DRAKE_DEMAND(msg.rhs(i).rows() * msg.rhs(i).cols() == num_bytes);
          mxLogical *logical_data =
            static_cast<mxLogical *>(mxGetData(rhs[i]));
          const std::string &str = msg.rhs(i).data();
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

      // Make the actual call to MATLAB.
      bool trapped_error = mexCallMATLABsafe(
        static_cast<int>(lhs.size()), lhs.data(),
        static_cast<int>(rhs.size()), rhs.data(), msg.function_name().c_str());

      if (!trapped_error) {
        // Assign any local variables that were returned by this call.
        for (i = 0; i < msg.lhs_size(); i++) {
          // Zap any old variables that will be overwritten by this call.
          if (client_vars_.find(msg.lhs(i)) != client_vars_.end())
            mxDestroyArray(client_vars_[msg.lhs(i)]);
          client_vars_[msg.lhs(i)] = lhs[i];
        }
      }

      // Clean up the input argument data.
      for (i = 0; i < static_cast<int>(rhs.size()); i++) {
        // Make sure it's not a client var.
        if (msg.rhs(i).type() !=
            drake::common::MatlabArray::REMOTE_VARIABLE_REFERENCE)
          mxDestroyArray(rhs[i]);
      }

      if (trapped_error) {
        break;
      }
    }
*/
  }
//  FlushMatlabEventBuffer();

  // Clean up remaining memory.
  for (const auto& iter : client_vars_) {
    mxDestroyArray(iter.second);
  }
}
